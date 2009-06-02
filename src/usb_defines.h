#ifndef __USB_DEFINES_H__
#define __USB_DEFINES_H__

#include <inttypes.h>

enum PipeType
{
    Control = 0,
    Isochronous = 1,
    Bulk = 2,
    Interrupt = 3
};

enum PipeToken
{
    Setup = 0,
    In = 1,
    Out = 2,
    ReservedToken = 3
};

enum PipeSize
{
    Size8 = 0,
    Size16 = 1,
    Size32 = 2,
    Size64 = 3,
    Size128 = 4, // only for endpoint 1
    Size256 = 5, // only for endpoint 1
    ReservedSize1 = 6,
    ReservedSize2 = 7
};

enum PipeBankNum
{
    OneBank = 0,
    TwoBanks = 1,
    Invalid1 = 2,
    Invalid2 = 3
};

#define PIPE_SIZE(pipe) (8 << (pipe)->size)

enum DescriptorType
{
    DEVICE = 1,
    CONFIGURATION = 2,
    STRING = 3,
    INTERFACE = 4,
    ENDPOINT = 5,
    DEVICE_QUALIFIER = 6,
    OTHER_SPEED_CONFIGURATION = 7,
    INTERFACE_POWER = 8
};

enum StandardRequestCode
{
    GET_STATUS = 0,
    CLEAR_FEATURE = 1,
    RESERVED0 = 2,
    SET_FEATURE = 3,
    RESERVED1 = 4,
    SET_ADDRESS = 5,
    GET_DESCRIPTOR = 6,
    SET_DESCRIPTOR = 7,
    GET_CONFIGURATION = 8,
    SET_CONFIGURATION = 9,
    GET_INTERFACE = 10,
    SET_INTERFACE = 11,
    SYNCH_FRAME = 12
};

enum PidName
{
    OUT = 1,
    IN = 9,
    SOF = 5,
    SETUP = 0xD,
    
    DATA0 = 3,
    DATA1 = 0xB,
    DATA2 = 7,
    MDATA = 0xF,
    
    ACK = 2,
    NAK = 0xA,
    STALL = 0xE,
    NYET = 6,
    
    PRE_OR_ERR = 0xC,
    SPLIT = 8,
    PING = 4,
    RESERVED = 0
};

typedef struct pipe_descriptor
{
    // UPCFG0X
    unsigned id : 3;
    enum PipeType type;
    enum PipeToken token;
    unsigned endpoint_target : 4;
    
    // UPCFG1X
    enum PipeSize size;
    enum PipeBankNum num_banks;
    unsigned int_freq : 8;
    
} pipe_descriptor_t;

typedef struct usb_setup_data
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_data_t;

typedef struct usb_device_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;    // DEVICE
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubclass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} usb_device_descriptor_t;

typedef struct usb_config_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;    // CONFIGURATION
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} usb_config_descriptor_t;

typedef struct usb_interface_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;    // INTERFACE
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} usb_interface_descriptor_t;

typedef struct usb_endpoint_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;    // ENDPOINT
    uint8_t pEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterface;
} usb_endpoint_descriptor_t;

#endif // __USB_DEFINES_H__
