#include "tusb.h"
#include <string.h>

//--------------------------------------------------------------------+
// Device Configuration
//--------------------------------------------------------------------+

// Endpoint Addresses
#define EPNUM_OUT 0x01  // Host -> Device (Bulk, LEDs + Display)
#define EPNUM_IN 0x81   // Device -> Host (Interrupt, Buttons + Encoder)

// Total length: Config + Interface + 2 Endpoints
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 9 + 7 + 7)

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

    .idVendor = 0xCAFE,   // Change to your VID
    .idProduct = 0x0001,  // Change to your PID
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
    9, TUSB_DESC_INTERFACE, 0, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 0,

    // 3. Endpoint Descriptor (OUT - Bulk - for Display)
    // bLength, bDescriptorType, bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval
    7, TUSB_DESC_ENDPOINT, EPNUM_OUT, TUSB_XFER_BULK, 64, 0,

    // 4. Endpoint Descriptor (IN - Interrupt - for Encoders)
    // bmAttributes: 0x03 = Interrupt
    // bInterval: 10 = 10 ms polling (fastest possible is 1 ms on Full Speed)
    7, TUSB_DESC_ENDPOINT, EPNUM_IN, TUSB_XFER_INTERRUPT, 64, 10
};
// clang-format on

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// Array of pointer to string descriptors
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "Betz Engineering",          // 1: Manufacturer
    "ui_to_usb",                 // 2: Product
    "R-S000",                    // 3: Serials
};

// Callbacks required by TinyUSB
uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&desc_device; }

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
            return NULL;
        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31)
            chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }

    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}
