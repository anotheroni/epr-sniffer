/*
 * Include file defining structures and constants used byt the PDIUSBD12 chip.
 *
 * @version 20030614
 * @author Oskar Nilsson
 */

/* Initialization commands */
#define D11_SET_ADDRESS_ENABLE         0xD1
#define D11_SET_HUB_ADDRESS            0xD0
#define D11_SET_ENDPOINT_ENABLE        0xD8
#define D11_SET_MODE   		            0xF3

/* Data flow commands */
#define D11_READ_INTERRUPT_REGISTER    0xF4

#define D11_SELECT_ENDPOINT_EP0_OUT 	0x02
#define D11_SELECT_ENDPOINT_EP0_IN 	   0x03
#define D11_SELECT_ENDPOINT_EP1_OUT 	0x05
#define D11_SELECT_ENDPOINT_EP1_IN 		0x04
#define D11_SELECT_ENDPOINT_EP2_OUT 	0x06
#define D11_SELECT_ENDPOINT_EP2_IN 		0x07

#define D11_RD_ENDP_STATUS_EP0_IN      0x82
#define D11_RD_ENDP_STATUS_EP0_OUT     0x83
#define D11_RD_ENDP_STATUS_EP1_IN      0x85
#define D11_RD_ENDP_STATUS_EP1_OUT     0x84
#define D11_RD_ENDP_STATUS_EP2_IN      0x86
#define D11_RD_ENDP_STATUS_EP2_OUT     0x87

#define D11_RD_TRANS_STATUS_EP0_OUT    0x42
#define D11_RD_TRANS_STATUS_EP0_IN     0x43
#define D11_RD_TRANS_STATUS_EP1_OUT    0x45
#define D11_RD_TRANS_STATUS_EP1_IN     0x44
#define D11_RD_TRANS_STATUS_EP2_OUT    0x46
#define D11_RD_TRANS_STATUS_EP2_IM     0x47

#define D11_READ_BUFFER			         0xF0
#define D11_WRITE_BUFFER 		         0xF0

#define D11_SET_ENDP_STATUS_EP0_OUT    0x42
#define D11_SET_ENDP_STATUS_EP0_IN     0x43
#define D11_SET_ENDP_STATUS_EP1_OUT    0x45
#define D11_SET_ENDP_STATUS_EP1_IN     0x44
#define D11_SET_ENDP_STATUS_EP2_OUT    0x46
#define D11_SET_ENDP_STATUS_EP2_IN     0x47

#define D11_ACK_SETUP 			         0xF1
#define D11_CLEAR_BUFFER		         0xF2
#define D11_VALIDATE_BUFFER	  	      0xFA

/* General commands */
#define D12_SEND_RESUME                0xF6
#define D12_RD_CURRENT_FRAME_NUMBER    0xF5

/* Set mode */
#define D12_SM_ENDPOINT_MODE_0         0x00
#define D12_SM_ENDPOINT_MODE_1         0x40
#define D12_SM_ENDPOINT_MODE_2         0x80
#define D12_SM_ENDPOINT_MODE_3         0xC0
#define D12_SM_SOFTCONNECT             0x10
#define D12_SM_INTERRUPT_MODE          0x08
#define D12_SM_CLOCK_RUNNING           0x04
#define D12_SM_NO_LAZYCLOCK            0x02

#define D12_SCD_SOF_ONLY_INTERRUPT     0x80
#define D12_SCD_SET_TO_ONE             0x40

/* Read interrupt register */
#define D11_INT_EP0_OUT               0x0004
#define D11_INT_EP0_IN                0x0008
#define D11_INT_EP1_OUT               0x0020
#define D11_INT_EP1_IN                0x0010
#define D11_INT_EP2_OUT               0x0040
#define D11_INT_EP2_IN                0x0080
#define D11_INT_EP3_OUT               0x0100
#define D11_INT_EP3_IN                0x0200
#define D11_INT_BUS_RESET             0x4000

#define D12_IREG_DMA_EOT               0x01

/* Select Endpoint */
#define D12_SE_BUFFER_FULL             0x01
#define D12_SE_STALLED                 0x02

/* Read endpoint status */
#define D12_EPS_SETUP_PACKET           0x04
#define D12_EPS_BUFFER_0_FULL          0x20
#define D12_EPS_BUFFER_1_FULL          0x40
#define D12_EPS_ENDPOINT_STALLED       0x80

/* Read last transaction status register */
#define D12_TSR_DATA_SUCCESS           0x01
#define D12_TSR_ERROR_CODE             0x1E
#define D11_TSR_SETUP_PACKET           0x20
#define D12_TSR_DATA_PACKET_NUM        0x40
#define D12_TSR_PREV_STATUS_NOT_READ   0x80

/* Misc */
#define D11_STALL_ENDPOINT             0x01

/* USB constants */
#define ENDPOINT_HALT			         0
#define TYPE_DEVICE_DESCRIPTOR         1
#define TYPE_CONFIGURATION_DESCRIPTOR  2
#define TYPE_STRING_DESCRIPTOR         3
#define TYPE_INTERFACE_DESCRIPTOR      4
#define TYPE_ENDPOINT_DESCRIPTOR       5
#define TYPE_HID_DESCRIPTOR		      0x21

/* USB request types */
#define STANDARD_DEVICE_REQUEST		   0x00
#define STANDARD_INTERFACE_REQUEST	   0x01
#define STANDARD_ENDPOINT_REQUEST	   0x02
#define VENDOR_DEVICE_REQUEST		      0x40
#define VENDOR_ENDPOINT_REQUEST		   0x42

/* USB request codes */
#define GET_STATUS  			            0x00
#define CLEAR_FEATURE     		         0x01
#define SET_FEATURE                 	0x03
#define SET_ADDRESS                 	0x05
#define GET_DESCRIPTOR              	0x06
#define SET_DESCRIPTOR              	0x07
#define GET_CONFIGURATION           	0x08
#define SET_CONFIGURATION           	0x09
#define GET_INTERFACE               	0x0A
#define SET_INTERFACE               	0x0B
#define SYNCH_FRAME                 	0x0C

typedef struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} USB_SETUP_REQUEST, *PUSB_SETUP_REQUEST;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned short bcdUSB;
		unsigned char bDeviceClass;
		unsigned char bDeviceSubClass;
		unsigned char bDeviceProtocol;
		unsigned char bMaxPacketSize0;
		unsigned short idVendor;
		unsigned short idProduct;
		unsigned short bcdDevice;
		unsigned char iManufacturer;
		unsigned char iProduct;
		unsigned char iSerialNumber;
		unsigned char bNumConfigurations;
} USB_DEVICE_DESCRIPTOR, *PUSB_DEVICE_DESCRIPTOR;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned char bEndpointAddress;
		unsigned char bmAttributes;
		unsigned short wMaxPacketSize;
		unsigned char bInterval;
} USB_ENDPOINT_DESCRIPTOR, *PUSB_ENDPOINT_DESCRIPTOR;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned short wTotalLength;
		unsigned char bNumInterfaces;
		unsigned char bConfigurationValue;
		unsigned char iConfiguration;
		unsigned char bmAttributes;
		unsigned char MaxPower;
} USB_CONFIGURATION_DESCRIPTOR, *PUSB_CONFIGURATION_DESCRIPTOR;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned char bInterfaceNumber;
		unsigned char bAlternateSetting;
		unsigned char bNumEndpoints;
		unsigned char bInterfaceClass;
		unsigned char bInterfaceSubClass;
		unsigned char bInterfaceProtocol;
		unsigned char iInterface;
} USB_INTERFACE_DESCRIPTOR, *PUSB_INTERFACE_DESCRIPTOR;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned short wHIDClassSpecComp;
		unsigned char bCountry;
		unsigned char bNumDescriptors;
		unsigned char b1stDescType;
		unsigned short w1stDescLength;
} USB_HID_DESCRIPTOR, *PUSB_HID_DESCRIPTOR;

typedef struct {
	USB_CONFIGURATION_DESCRIPTOR ConfigDescriptor;
	USB_INTERFACE_DESCRIPTOR InterfaceDescriptor;
//   USB_HID_DESCRIPTOR HidDescriptor;
  	USB_ENDPOINT_DESCRIPTOR EndpointDescriptor0;
	USB_ENDPOINT_DESCRIPTOR EndpointDescriptor1;
} USB_CONFIG_DATA, *PUSB_CONFIG_DATA;

typedef struct {
   unsigned char bLenght;
   unsigned char bDescriptorType;
   char bString[24];
} MANUFACTURER_DESCRIPTOR, *PMANUFACTURER_DESCRIPTOR;

typedef struct {
   unsigned char bLenght;
   unsigned char bDescriptorType;
   char bString[46];
} PRODUCT_DESCRIPTOR, *PPRODUCT_DESCRIPTOR;

typedef struct {
   unsigned char bLenght;
   unsigned char bDescriptorType;
   unsigned short wLANGID0;
} LANGID_DESCRIPTOR, *PLANGID_DESCRIPTOR;

typedef struct {
   unsigned char bID;
   unsigned char data;
} USB_HID_REPORT, *PUSB_HID_REPORT;
