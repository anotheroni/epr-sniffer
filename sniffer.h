/**
  * .h file for the Mobitex sniffer program.
  *
  * @version 20030624
  * @author Oskar Nilsson
  */

#include "d11.h"

// Program constants
#define IDLE 0 
#define CHANGING_ADDRESS 9

// Timer
#define TIMER_START_1024 0x05
#define TIMER_START_256 0x04
#define TIMER_STOP 0x00

// USART states
#define USART_IDLE 0
#define USART_RECEIVING 1

// USART buffers
#define USART_HEADER_LIST_SIZE 2
#define USART_PACKET_HEADER_DATA_SIZE 8
#define USART_BUFFER_SIZE 12
#define USART_BUFFER_ROW_LENGTH 10

// USB
#define PROGRESS_IDLE 0
#define PROGRESS_ADDRESS 1

// Mobitex states
#define MOBITEX_IDLE 0
#define MOBITEX_SEND 1

#define MOBITEX_BUFFER_ROW_LENGTH 8

// Mobitex commands
#define MOBITEX_DATA_PACKET 0x0001
#define MOBITEX_QUEUE_EMPTY 0x0002
#define MOBITEX_END_OF_DATA 0x0003
#define MOBITEX_COMMAND_ERROR 0x000F
#define MOBITEX_GET_PACKET 0x0010
#define MOBITEX_ACK 0x0011
#define MOBITEX_NACK 0x0012

// ---- USART data structures ----

struct usart_packet_header
{
   void *addr;
   void *next;
   uint16_t size;
   uint8_t error;
   uint8_t reserved8;
   uint16_t reserved16;
};

// ---- Mobitex data structures ---

struct mobitex_packet_header
{
   uint16_t cmd;
   uint16_t size;
   uint8_t error;
   uint8_t reserved8;
   uint16_t reserved16;
};

// ---- USB data structures ----
const USB_DEVICE_DESCRIPTOR DeviceDescriptor =
{
   sizeof(USB_DEVICE_DESCRIPTOR),   // bLength
   TYPE_DEVICE_DESCRIPTOR,          // bDescriptorType
   0x0110,                          // bcdUSB USB Version 1.1
   0,                               // bDeviceClass (using bulkusb driver)
   0,                               // bDeviceSubclass (No subclass)
   0,                               // bDeviceProtocoll (None)
   8,                               // bMaxPacketSize for CTRL (8 Bytes)
   0x045e,  // Intel //0x04cc,      // idVendor (Philips semiconductors)
   0x930a,  // IO test board //0x174c, // idProduct (Random)
   0x0000,                          // bcdDevice release number
   1,                               // iManufacturer String Index
   2,                               // iProduct String Index
   0,                               // iSerialNumber String Index
   1                                // bNumberConfigurations
};

const USB_CONFIG_DATA ConfigurationDescriptor =
{
   {  // Configuration descriptor
      sizeof(USB_CONFIGURATION_DESCRIPTOR),  // bLength
      TYPE_CONFIGURATION_DESCRIPTOR,         // bDescriptorType
      sizeof(USB_CONFIG_DATA),   // wTotalLength
      1,                         // bNumInterfaces
      1,                         // bConfigurationValue
      0,                         // iConfiguration String Index
      0x40,                      // bmAttributes; Self Powered, No Remote Wakeup
      0x32                       // bMaxPower, 100mA
   },
   {  // Interface descriptor
      sizeof(USB_INTERFACE_DESCRIPTOR),   // bLength
      TYPE_INTERFACE_DESCRIPTOR, // bDescriptorType
      0,                         // bInterface Number
      0,                         // bAlternateSetting
      2,                         // bNumEndpoints
      0xff,     // bInterfaceClass ((Vendor specific)
      0xff,                      // bInterfaceSubClass
      0xff,                      // bInterfaceProtocol
      0                          // Interface String Index
   },
/*   {  // HID descriptor
      sizeof (USB_HID_DESCRIPTOR),  // bLength
      TYPE_HID_DESCRIPTOR,       // bDescriptorType
      0x0001,                    // HID class specification
      0,                         // Country localization (=none)
      1,                         // Number of descriptors to follow
      0x22,                      // it's a report descriptor
      sizeof (USB_HID_REPORT)    // HID report size
   },*/
   {  // Endpoint descriptor
      sizeof(USB_ENDPOINT_DESCRIPTOR),    // bLength
      TYPE_ENDPOINT_DESCRIPTOR,  // bDescriptorType
      0x01,                      // bEndpoint Address (EP1 OUT)
      0x02,                      // bmAttributes (Interrupt)
      0x0008,                    // wMaxPacketSize (8 Bytes)
      0x00//0x8f                       // bInterval for polling (127 frame (ms))
   },
   {  // Endpoint descriptor
      sizeof(USB_ENDPOINT_DESCRIPTOR),    // bLength
      TYPE_ENDPOINT_DESCRIPTOR,  // bDescriptorType
      0x81,                      // bEndpoint Address (EP1 IN)
      0x02,                      // bmAttributes (Interrupt)
      0x0008,                    // wMaxPacketSize (8 Bytes)
      0x00//0x8f                       // bInterval for polling (127 frame (ms))
   }
};

const LANGID_DESCRIPTOR Langid_Descriptor =
{
   sizeof(LANGID_DESCRIPTOR),    // bLength
   TYPE_STRING_DESCRIPTOR,       // bDescriptorType
   0x0409                        // Langid (English)
};

const MANUFACTURER_DESCRIPTOR Manufacturer_Descriptor =
{
   sizeof(MANUFACTURER_DESCRIPTOR), // bLength
   TYPE_STRING_DESCRIPTOR,          // bDescriptorType
   "N\0&\0N\0"                      // ManufacturerString in UNICODE
};

const PRODUCT_DESCRIPTOR Product_Descriptor =
{
   sizeof(PRODUCT_DESCRIPTOR),      // bLength
   TYPE_STRING_DESCRIPTOR,          // bDescriptorType
   "M\0o\0b\0i\0t\0e\0x\0 \0s\0n\0i\0f\0f\0e\0r\0" // Productstring in UNICODE
};
