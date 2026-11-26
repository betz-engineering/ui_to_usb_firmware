"""smbus2.SMBus-compatible wrapper around the I2C master of the ui_to_usb board (CH32V203).

    from py_ui_board.ui_smbus import UiBoardSMBus

    with UiBoardSMBus(f_scl=100, scl_pin=6, sda_pin=7) as bus:
        bus.pec = 1
        print(hex(bus.read_word_data(0x58, 0x8B)))

The firmware does one transaction per request: START, addr+W, tx bytes, repeated START, addr+R,
rx bytes, STOP (either part may be empty). That covers all SMBus protocols, the same way the Linux SMBus
emulation does it. Limits: 64 bytes written and 63 bytes read per transaction.

Block reads (``read_block_data``, ``block_process_call``): the firmware cannot read the byte count and then
decide how many bytes follow, so the maximum (count + 32 bytes + PEC) is read and the reply is truncated
to the received count. The slave returns padding bytes past the end of its block.

Errors are raised as ``OSError`` like i2c-dev does: ENXIO on NACK, EBUSY if SDA is held low before the
START, ETIMEDOUT if a slave stretches SCL for longer than 25 ms, EBADMSG on a PEC mismatch.
"""

import errno
import threading

from .ch32 import I2C_RESULT, UiBoard

I2C_SMBUS_BLOCK_MAX = 32
I2C_M_RD = 0x0001
I2C_M_RECV_LEN = 0x0400

ERRORS = {
    I2C_RESULT.ERR_NACK: (errno.ENXIO, "No ACK from slave"),
    I2C_RESULT.ERR_BUS: (errno.EBUSY, "I2C bus busy: SDA held low before START"),
    I2C_RESULT.ERR_TIMEOUT: (errno.ETIMEDOUT, "SCL held low by a slave for > 25 ms"),
    I2C_RESULT.BUSY: (errno.ETIMEDOUT, "Transaction did not finish"),
}


def _crc8(data, crc: int = 0) -> int:
    """SMBus PEC: CRC-8, polynomial x^8 + x^2 + x + 1, initial value 0."""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class UiBoardSMBus:
    """Drop-in replacement for the parts of smbus2.SMBus used with PMBus devices."""

    def __init__(
        self,
        ui: UiBoard | None = None,
        f_scl: int = 100,
        scl_pin: int = 6,
        sda_pin: int = 7,
    ):
        """``ui``: an open UiBoard, or None to open the first one found.
        ``f_scl``: SCL frequency in [kHz]. ``scl_pin``, ``sda_pin``: PORTB pin numbers (0 - 7).
        External pull-ups are needed!"""
        self.ui = UiBoard() if ui is None else ui
        self.ui.i2c_config(f_scl, scl_pin, sda_pin)
        self.pec = 0
        self._lock = threading.Lock()

    def enable_pec(self, enable: bool = True) -> None:
        self.pec = int(bool(enable))

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ------------------------------------------------------------------ helpers

    def _xfer(self, addr: int, out=b"", length: int = 0) -> bytes:
        """START, addr+W, out, Sr, addr+R, length bytes, STOP. Without ``out`` it's just a read."""
        with self._lock:
            result, data = self.ui.i2c_transfer(addr, bytes(out), length)
        if result != I2C_RESULT.OK:
            raise OSError(*ERRORS.get(result, (errno.EIO, f"I2C error {result!r}")))
        return data[:length]

    def _write(self, addr: int, payload) -> None:
        """START, addr+W, payload [, PEC], STOP"""
        payload = bytes(payload)
        if self.pec:
            payload += bytes([_crc8(payload, _crc8([addr << 1]))])
        self._xfer(addr, payload)

    def _write_read(self, addr: int, out, length: int) -> bytes:
        """SMBus read, with command phase if ``out`` is given. PEC checked when enabled."""
        out = bytes(out)
        data = self._xfer(addr, out, length + (1 if self.pec else 0))
        if self.pec:
            prefix = [addr << 1, *out] if out else []
            self._check_pec(data, [*prefix, (addr << 1) | 1])
        return data[:length]

    @staticmethod
    def _check_pec(data: bytes, prefix) -> None:
        """``data`` ends with the PEC byte, ``prefix`` are the preceding bytes on the wire (addresses, command)."""
        if _crc8(data[:-1], _crc8(prefix)) != data[-1]:
            raise OSError(errno.EBADMSG, "PEC mismatch")

    @staticmethod
    def _check_block_len(data) -> None:
        if len(data) > I2C_SMBUS_BLOCK_MAX:
            raise ValueError(f"Data length cannot exceed {I2C_SMBUS_BLOCK_MAX} bytes")

    def _block_read(self, addr: int, out) -> list[int]:
        """Write ``out``, repeated START, then read an SMBus block (count byte + count data bytes [+ PEC])."""
        out = bytes(out)
        pec = 1 if self.pec else 0
        data = self._xfer(addr, out, 1 + I2C_SMBUS_BLOCK_MAX + pec)
        count = data[0]
        # Same check as the kernel I2C_M_RECV_LEN handling: 0 or > 32 is a protocol error
        if not (1 <= count <= I2C_SMBUS_BLOCK_MAX):
            raise OSError(errno.EPROTO, f"Invalid SMBus block length: {count}")
        if pec:
            self._check_pec(data[: count + 2], [addr << 1, *out, (addr << 1) | 1])
        return list(data[1 : 1 + count])

    # ------------------------------------------------------------------ SMBus protocol

    def write_quick(self, i2c_addr: int, force=None) -> None:
        self._xfer(i2c_addr)  # address only, then STOP

    def read_byte(self, i2c_addr: int, force=None) -> int:
        return self._write_read(i2c_addr, b"", 1)[0]

    def write_byte(self, i2c_addr: int, value: int, force=None) -> None:
        self._write(i2c_addr, [value & 0xFF])

    def read_byte_data(self, i2c_addr: int, register: int, force=None) -> int:
        return self._write_read(i2c_addr, [register & 0xFF], 1)[0]

    def write_byte_data(
        self, i2c_addr: int, register: int, value: int, force=None
    ) -> None:
        self._write(i2c_addr, [register & 0xFF, value & 0xFF])

    def read_word_data(self, i2c_addr: int, register: int, force=None) -> int:
        # SMBus words are little-endian (LSB first)
        data = self._write_read(i2c_addr, [register & 0xFF], 2)
        return data[0] | (data[1] << 8)

    def write_word_data(
        self, i2c_addr: int, register: int, value: int, force=None
    ) -> None:
        self._write(i2c_addr, [register & 0xFF, value & 0xFF, (value >> 8) & 0xFF])

    def process_call(self, i2c_addr: int, register: int, value: int, force=None) -> int:
        data = self._write_read(
            i2c_addr, [register & 0xFF, value & 0xFF, (value >> 8) & 0xFF], 2
        )
        return data[0] | (data[1] << 8)

    def read_block_data(self, i2c_addr: int, register: int, force=None) -> list[int]:
        return self._block_read(i2c_addr, [register & 0xFF])

    def write_block_data(
        self, i2c_addr: int, register: int, data: list[int], force=None
    ) -> None:
        self._check_block_len(data)
        self._write(i2c_addr, [register & 0xFF, len(data), *data])

    def block_process_call(
        self, i2c_addr: int, register: int, data: list[int], force=None
    ) -> list[int]:
        self._check_block_len(data)
        return self._block_read(i2c_addr, [register & 0xFF, len(data), *data])

    def read_i2c_block_data(
        self, i2c_addr: int, register: int, length: int, force=None
    ) -> list[int]:
        # Plain I2C transfer: no byte count and (as in the kernel) no PEC
        if length > I2C_SMBUS_BLOCK_MAX:
            raise ValueError(f"Desired block length over {I2C_SMBUS_BLOCK_MAX} bytes")
        return list(self._xfer(i2c_addr, [register & 0xFF], length))

    def write_i2c_block_data(
        self, i2c_addr: int, register: int, data: list[int], force=None
    ) -> None:
        self._check_block_len(data)
        self._xfer(i2c_addr, [register & 0xFF, *data])

    def i2c_rdwr(self, *i2c_msgs) -> None:
        """smbus2.i2c_msg transactions. Only what the firmware can do in one transaction:
        a single write, a single read, or a write followed by a read from the same address."""
        if any(msg.flags & I2C_M_RECV_LEN for msg in i2c_msgs):
            raise NotImplementedError("I2C_M_RECV_LEN is not supported")
        rd = [m for m in i2c_msgs if m.flags & I2C_M_RD]
        wr = [m for m in i2c_msgs if not m.flags & I2C_M_RD]
        if (
            len(i2c_msgs) > 2
            or len(rd) > 1
            or len(wr) > 1
            or (wr and rd and i2c_msgs[0] is not wr[0])
        ):
            raise NotImplementedError("only write, read or write + read are supported")
        if wr and rd and wr[0].addr != rd[0].addr:
            raise NotImplementedError("write + read must go to the same address")
        addr = i2c_msgs[0].addr
        data = self._xfer(addr, bytes(wr[0]) if wr else b"", len(rd[0]) if rd else 0)
        if rd:
            for j, b in enumerate(data):
                rd[0].buf[j] = b
