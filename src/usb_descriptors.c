#include "tusb.h"
#include <stdint.h>
#include <string.h>

//--------------------------------------------------------------------+
// Device Configuration
//--------------------------------------------------------------------+

// Endpoint Addresses
#define EPNUM_OUT 0x01  // Host -> Device (Bulk, LEDs + Display)

// Total length: Config + Interface + Bulk endpoint
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 9 + 7)

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+

// Standard Device Descriptor
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,

    // Use 0xFF to indicate Vendor Specific device
    // This tells Linux "Don't load standard drivers, wait for libusb"
    .bDeviceClass = TUSB_CLASS_VENDOR_SPECIFIC,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = 0x16c0,
    .idProduct = 0x05dc,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01};

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
// clang-format off
uint8_t const desc_configuration[] =
{
    // 1. Config Descriptor
    // Config number, interface count, string index, total length, attributes, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00, 500),

    // 2. Interface Descriptor (Vendor Specific Class)
    // bLength, bDescriptorType, bInterfaceNumber, bAlternateSetting, bNumEndpoints, bInterfaceClass, bInterfaceSubClass, bInterfaceProtocol, iInterface
    9, TUSB_DESC_INTERFACE, 0, 0, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 0,

    // 3. Endpoint Descriptor (OUT - Bulk - for Display)
    // bLength, bDescriptorType, bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval
    7, TUSB_DESC_ENDPOINT, EPNUM_OUT, TUSB_XFER_BULK, 64, 0,
};
// clang-format on

// Callbacks required by TinyUSB
uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&desc_device; }

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
// String Descriptor Index
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
};

// Array of pointer to string descriptors
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "betz-engineering.ch",       // 1: Manufacturer
    "ui_to_usb",                 // 2: Product
};

static uint16_t _desc_str[32];

static void put_hex(uint32_t val, uint8_t digits, uint16_t *p) {
    for (int i = (4 * digits) - 4; i >= 0; i -= 4)
        *p++ = ("0123456789ABCDEF"[(val >> i) % 16]);
}

// Will write R1S<unique_id in hex> (27 characters) to desc_str
int ui_to_usb_get_serial(uint16_t *desc_str) {
    uint16_t *p = desc_str;
    *p++ = 'R';
    *p++ = '1';
    *p++ = 'S';
    // Append 12 byte unique ID
    volatile uint32_t *ch32_uuid = ((volatile uint32_t *)0x1FFFF7E8UL);
    put_hex(ch32_uuid[0], 8, p);
    p += 8;
    put_hex(ch32_uuid[1], 8, p);
    p += 8;
    put_hex(ch32_uuid[2], 8, p);

    return 27;
}

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long
// enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;

    switch (index) {
    case STRID_LANGID:
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
        break;

    case STRID_SERIAL:
        chr_count = ui_to_usb_get_serial(_desc_str + 1);
        break;

    default:
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }

        const char *str = string_desc_arr[index];

        // Cap at max char
        chr_count = strlen(str);
        size_t const max_count =
            sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;  // -1 for string type
        if (chr_count > max_count) {
            chr_count = max_count;
        }

        // Convert ASCII string into UTF-16
        for (size_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
        break;
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
