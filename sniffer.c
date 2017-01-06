/**
 * Main file for the Mobitex sniffer program.
 *
 * @version 20030630
 * @author Oskar Nilsson
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "sniffer.h"
#include "twi.h"

#define false 0
#define true 1

// System clock in Hz
#define SYSCLK 3690000UL

#define EEPROM_SIZE 0x3FFF
#define EEPROM_START_ADDRESS 0x2008 // Skip the first row to avoid addres 0

#ifndef __AVR_ATmega16__
#  error "The MCU type you are using is not defined in the source"
#endif

// --- Function Headers ---

void initfun(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));
void process_ep0_out_interrupt (void);
void process_ep1_out_interrupt (void);
void getDescriptor (USB_SETUP_REQUEST *setupPacket);
void stall_control_endpoint (void);
void get_next_twi_command (uint8_t stop);
int uart_putchar(char);
void d11CmdDataWrite (uint8_t *cmd, uint8_t *data, uint8_t size);
void d11CmdDataRead (uint8_t *cmd, uint8_t *data, uint8_t size);
void d11WriteEndpoint (uint8_t endpoint, uint8_t *data, uint8_t size);
uint8_t d11ReadEndpoint (uint8_t endpoint, uint8_t *data);
void writeBufferToEndpoint (void);
uint8_t * twi_add_queue_command (uint8_t cmd, uint8_t size, uint8_t *data);
void init_usb (void);

// --- Global Variables ---

// - TWI
uint8_t twi_state;
uint8_t twi_byte_to_process;
// A pointer to the buffer that the TWI is currently working with.
uint8_t *twi_current_data;
// The position in the current buffer.
uint8_t twi_current_pos;
// TWI command queue
struct twi_queue_entry twi_command_queue[TWI_QUEUE_SIZE];
uint8_t twi_queue_start;
uint8_t twi_queue_end;
// NACK received from last command
uint8_t twi_nack;

// - USART
uint8_t usart_state;
// Packet headers
struct usart_packet_header usart_header_list[USART_HEADER_LIST_SIZE];
uint8_t usart_current_header;
// Data buffer
uint8_t usart_buffer[USART_BUFFER_SIZE][USART_BUFFER_ROW_LENGTH];
uint8_t usart_buffer_current_pos;
uint8_t usart_buffer_current_line;

// - EEPROM
// Addresses in eeprom
void * eeprom_start;  // The first block's header
void * eeprom_last;   // The last block's header
void * eeprom_current_start;   // The current block's header
void * eeprom_free;   // The free line to use

// - USB
uint8_t ctrlTransferInProgress;  // Flag set if set address is in progress
uint8_t usb_address;  // USB address
uint8_t usb_deviceConfigured; // Flag set if the USB chip is configured
uint8_t * usb_sendbuffer;  // Data that is to be sent by usb
uint8_t usb_bytestosend;   // The number of bytes in the send buffer

// - Mobitex
uint8_t mobitex_state;  // State of the mobitex state machine
uint8_t mobitex_buffer[MOBITEX_BUFFER_ROW_LENGTH];  // Data buffer
struct mobitex_packet_header *pmobitex_header = 
(struct mobitex_packet_header *)mobitex_buffer; // Pointer to the data buffer
void * mobitex_current_address;  // address of eeprom line to transmit
void * mobitex_last_address;    // address of the last epprom line to transmit
uint8_t mobitex_eeprom_buffer [USART_BUFFER_ROW_LENGTH];//Buf for data fr eeprom
uint8_t mobitex_data_in_eeprom_buffer; // Flag set if there is data in buffer

// --- Functions ---

/**
  * The first code executed when the MCU is started.
  */
void initfun(void) \
{
}

/**
 * Function handling USB chip interrupts (external interrupt 1, PD 3).
 * SIGNAL() is a macro that marks the function as an interrupt routine.
 */
SIGNAL(INT1_vect)
{
   PORTB = PORTB | 0x03;

   uint16_t irq;
   uint8_t buffer;
   uint8_t cmd;

//   enable_external_int (_BV (INT0));   // Disable INT1
   GICR = GICR & 0x0F;  // Disable all external INT

   // Read Interrupt register
   cmd = D11_READ_INTERRUPT_REGISTER;
   d11CmdDataRead (&cmd, (uint8_t *)&irq, 2);

   if (irq & D11_INT_BUS_RESET)
   {
      //printf ("BusReset\n");
      init_usb ();
   }
   if (irq & D11_INT_EP0_OUT)
   {
      //printf ("EP0_Out\n");
      process_ep0_out_interrupt ();
   }
   if (irq & D11_INT_EP0_IN)
   {
      //printf ("EP0_In\n");
      if (ctrlTransferInProgress == PROGRESS_ADDRESS)
      {
         cmd = D11_SET_ADDRESS_ENABLE;
         d11CmdDataWrite (&cmd, &usb_address, 1);
         cmd = D11_RD_TRANS_STATUS_EP0_IN;
         d11CmdDataRead (&cmd, &buffer, 1);
         ctrlTransferInProgress = PROGRESS_IDLE;
      }
      else
      {
         cmd = D11_RD_TRANS_STATUS_EP0_IN;
         d11CmdDataRead (&cmd, &buffer, 1);
         writeBufferToEndpoint ();
      }
   }
   if (irq & D11_INT_EP1_OUT)
   {
      //printf ("EP1_Out\n");
      process_ep1_out_interrupt ();
   }
   if (irq & D11_INT_EP1_IN)
   {
      //printf ("EP1_In\n");
      cmd = D11_RD_TRANS_STATUS_EP1_IN;
      d11CmdDataRead (&cmd, &buffer, 1);
      if (mobitex_data_in_eeprom_buffer)
      {
         d11WriteEndpoint (D11_SELECT_ENDPOINT_EP1_IN,
              &mobitex_eeprom_buffer[2], 8);
         mobitex_data_in_eeprom_buffer = 0;
      }
   }
   
   //enable_external_int (_BV (INT1) | _BV (INT0));  // Enable INT1
   GICR = GICR | 0xC0;  // Enable INT 0 and 1

   PORTB = PORTB & 0xFC;
}

/**
  * Function called by the USB interrupt function to handle ep0_out interrupts.
  */
void process_ep0_out_interrupt (void)
{
   uint8_t buffer[2];
   uint8_t cmd;
   USB_SETUP_REQUEST setupPacket;

   // Check of packet received is setup or data, also clears IRQ
   cmd = D11_RD_TRANS_STATUS_EP0_OUT;
   d11CmdDataRead (&cmd, (uint8_t *) &setupPacket, 1);

   if (setupPacket.bmRequestType & D11_TSR_SETUP_PACKET)
   {
      // Read the setup packet
      d11ReadEndpoint (D11_SELECT_ENDPOINT_EP0_OUT, (uint8_t *) &setupPacket);

      // Acknowledge setup packet to EP0_OUT & Clear Buffer
      cmd = D11_ACK_SETUP;
      d11CmdDataWrite (&cmd, NULL, 0);
      cmd = D11_CLEAR_BUFFER;
      d11CmdDataWrite (&cmd, NULL, 0);

      // Acknowledge setup packet to EP0_IN
      cmd = D11_SELECT_ENDPOINT_EP0_IN;
      d11CmdDataWrite (&cmd, NULL, 0);
      cmd = D11_ACK_SETUP;
      d11CmdDataWrite (&cmd, NULL, 0);

      // Parse bmRequestType
      switch (setupPacket.bmRequestType & 0x7F)
      {
         case STANDARD_DEVICE_REQUEST:
            //printf ("stdDevReq %x\n", setupPacket.bRequest);
            switch (setupPacket.bRequest)
            {
               case GET_STATUS:
                  buffer[0] = 0x01; // Self powered
                  buffer[1] = 0x00; // Reserved
                  d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, buffer, 2);
                  break;
               case CLEAR_FEATURE:
               case SET_FEATURE:
                  // No features supported
                  stall_control_endpoint ();
                  break;
               case SET_ADDRESS:
                  //printf ("Set addr\n");
                  usb_address = setupPacket.wValue | 0x80;
                  d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, NULL, 0);
                  // Can't change address until the cmd is acked and a second
                  // packet is received.
                  ctrlTransferInProgress = PROGRESS_ADDRESS;
                  break;
               case GET_DESCRIPTOR:
                  getDescriptor (&setupPacket);
                  break;
               case GET_CONFIGURATION:
                  cmd = D11_SELECT_ENDPOINT_EP0_IN;
                  d11CmdDataWrite (&cmd, &usb_deviceConfigured, 1);
                  break;
               case SET_CONFIGURATION:
                  usb_deviceConfigured = setupPacket.wValue & 0xFF;
                  d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, NULL, 0);
/*                  if (usb_deviceConfigured)
                     //printf ("Dev configured\n");
                  else
                     //printf ("Dev not configured\n");
*/
                  break;
               default:
                  //printf ("UnsupDevReq\n");
                  stall_control_endpoint ();
            }
            break;

         case STANDARD_INTERFACE_REQUEST:
            //printf ("StdInterfReq\n");
            switch (setupPacket.bRequest)
            {
               case GET_STATUS:
                  buffer[0] = 0x00; // Reserved
                  buffer[1] = 0x00; // Reserved
                  d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, buffer, 2);
                  break;
               case SET_INTERFACE:
                  // The device only supports the default setting
                  if (setupPacket.wIndex == 0 && setupPacket.wValue == 0)
                     d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, NULL, 0);
                  else
                     stall_control_endpoint ();
                  break;
               case GET_INTERFACE:
                  if (setupPacket.wIndex == 0)  // Interface Zero
                  {
                     buffer[0] = 0; // Alternative setting
                     d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, buffer, 1);
                     break;
                  }  // else fall through as request error
               default:
                  stall_control_endpoint ();
            }
            break;

         case STANDARD_ENDPOINT_REQUEST:
            //printf ("StdEndpReq\n");
            switch (setupPacket.bRequest)
            {
               case CLEAR_FEATURE:
               case SET_FEATURE:
                  if (setupPacket.wValue == ENDPOINT_HALT)
                  {
                     if (setupPacket.bRequest == CLEAR_FEATURE)
                        buffer[0] = 0x00;
                     else
                        buffer[0] = 0x01;
                     switch (setupPacket.wIndex & 0xFF)
                     {
                        case 0x01:
                           cmd = D11_SET_ENDP_STATUS_EP1_OUT;
                           break;
                        case 0x81:
                           cmd = D11_SET_ENDP_STATUS_EP1_IN;
                           break;
                        case 0x02:
                           cmd = D11_SET_ENDP_STATUS_EP2_OUT;
                           break;
                        case 0x82:
                           cmd = D11_SET_ENDP_STATUS_EP2_IN;
                           break;
                        default:
                           cmd = 0;
                           stall_control_endpoint ();
                     }
                     if (cmd != 0)
                        d11CmdDataWrite (&cmd, buffer, 1);
                     d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, NULL, 0);
                  }
                  else
                     stall_control_endpoint ();
                  break;
               case GET_STATUS:
                  switch (setupPacket.wIndex & 0xFF)
                  {
                     case 0x01:
                        cmd = D11_RD_ENDP_STATUS_EP1_OUT;
                        break;
                     case 0x81:
                        cmd = D11_RD_ENDP_STATUS_EP1_IN;
                        break;
                     case 0x02:
                        cmd = D11_RD_ENDP_STATUS_EP2_OUT;
                        break;
                     case 0x82:
                        cmd = D11_RD_ENDP_STATUS_EP2_IN;
                        break;
                     default:
                        cmd = 0;
                        stall_control_endpoint ();
                  }
                  if (cmd != 0)
                     d11CmdDataRead (&cmd, buffer, 1);
                  
                  if (buffer[0] & 0x08)
                     buffer[0] = 0x01;
                  else
                     buffer[0] = 0x00;
                  buffer [1] = 0x00;
                  d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, buffer, 2);
                  break;
               default:
                  stall_control_endpoint ();
            }
            break;

         default:
            //printf ("UnsupReq%x\n", setupPacket.bmRequestType);
            stall_control_endpoint ();
            break;
      }
   }
   else
   {
      //printf ("DataPak\n");
   }
}

/**
  * Function that handels interrupts on ep1_out (data from the PC).
  */
void process_ep1_out_interrupt (void)
{
   uint8_t cmd;
   uint8_t buffer;
   volatile uint8_t *res;
   struct usart_packet_header *head;
   head = (struct usart_packet_header *) mobitex_eeprom_buffer;

   cmd = D11_RD_TRANS_STATUS_EP1_OUT;
   d11CmdDataRead (&cmd, &buffer, 1);
   cmd = D11_SELECT_ENDPOINT_EP1_OUT;
   d11ReadEndpoint (cmd, mobitex_buffer);

   switch (pmobitex_header->cmd)
   { 
      case MOBITEX_GET_PACKET:
         if (eeprom_start == NULL)   // Nothing to send
            head->next = (void *) MOBITEX_QUEUE_EMPTY;
         else  // Start transmitting the first packet in the buffer
         {
            mobitex_state = MOBITEX_SEND;
            head->addr = eeprom_start;

            res = twi_add_queue_command (TWI_READ_EEPROM_ADDR, 10,
                  mobitex_eeprom_buffer);

            sei ();  // Enable interrupts
            while (*res != TWI_COMMAND_FINISHED)
               ;   // Wait until the command is finished
            cli ();  // Disable interrupts

            mobitex_current_address = eeprom_start + 8;
            mobitex_last_address = head->next;

            head->next = (void *) MOBITEX_DATA_PACKET;
         }
         break;

      case MOBITEX_ACK:
         if (mobitex_state != MOBITEX_SEND)
            head->next = (void *) MOBITEX_COMMAND_ERROR;
         // No more data, delete the packet and send an and of data message
         else if (mobitex_current_address == mobitex_last_address)
         {
            mobitex_state = MOBITEX_IDLE;
            head->next = (void *) MOBITEX_END_OF_DATA;
            
            // Last packet
            if (eeprom_start == eeprom_last)
               eeprom_start = NULL;
            else  // More packets exist
               eeprom_start = mobitex_last_address;
            
         }  
         else  // Send the next packet 
         {
            head->addr = mobitex_current_address;
//            printf ("snd %x\n", mobitex_current_address);
            res = twi_add_queue_command (TWI_READ_EEPROM_ADDR, 10,
                  mobitex_eeprom_buffer);

            sei ();  // Enable interrupts
            while (*res != TWI_COMMAND_FINISHED)
               ;   // Wait until the command is finished
            cli ();  // Disable interrupts

//            printf ("rd %x %x %x %x\n", mobitex_eeprom_buffer[0],
//                 mobitex_eeprom_buffer[1], mobitex_eeprom_buffer[2],
//                 mobitex_eeprom_buffer[3]);

            mobitex_current_address += 8;
         }
        break;

      case MOBITEX_NACK:
         if (mobitex_state != MOBITEX_SEND)
            head->next = (void *) MOBITEX_COMMAND_ERROR;
         else
         {
            head->addr = mobitex_current_address - 8;
            res = twi_add_queue_command (TWI_READ_EEPROM_ADDR, 10,
                  mobitex_eeprom_buffer);

            sei ();  // Enable interrupts
            while (*res != TWI_COMMAND_FINISHED)
               ;   // Wait until the command is finished
            cli ();  // Disable interrupts
         }
         break;

      default:
         //printf ("ERR mob %x\n", pmobitex_header->cmd);
         return;
   }

   // Send the data 
   cmd = D11_SELECT_ENDPOINT_EP1_IN;
   d11CmdDataRead (&cmd, &buffer, 1);

   if (buffer == 0)  // Buffer is empty, write the data
      d11WriteEndpoint (cmd, &mobitex_eeprom_buffer[2], 8);
   else  // Set data exists flag, send it when the buffer is empty
      mobitex_data_in_eeprom_buffer = 1;
}

/**
  * Function handling USB get descriptor requests.
  */
void getDescriptor (USB_SETUP_REQUEST *setupPacket)
{
   switch ((setupPacket->wValue & 0xFF00) >> 8)
   {
      case TYPE_DEVICE_DESCRIPTOR:
         //printf ("DevDesc\n");
         usb_sendbuffer = (uint8_t *) &DeviceDescriptor;
         usb_bytestosend = DeviceDescriptor.bLength;
         if (usb_bytestosend > setupPacket->wLength)
            usb_bytestosend = setupPacket->wLength;
         writeBufferToEndpoint ();
         break;
      case TYPE_CONFIGURATION_DESCRIPTOR:
         //printf ("ConfDesc\n");
         usb_sendbuffer = (uint8_t *) &ConfigurationDescriptor;
         usb_bytestosend = sizeof (ConfigurationDescriptor);
         if (usb_bytestosend > setupPacket->wLength)
            usb_bytestosend = setupPacket->wLength;
         writeBufferToEndpoint ();
         break;
      case TYPE_STRING_DESCRIPTOR:
         //printf ("StrDesc %x\n", (setupPacket->wValue & 0xFF));
         switch (setupPacket->wValue & 0xFF)
         {
            case 0:
               usb_sendbuffer = (uint8_t *) &Langid_Descriptor;
               usb_bytestosend = sizeof (Langid_Descriptor);
               break;
            case 1:
               usb_sendbuffer = (uint8_t *) &Manufacturer_Descriptor;
               usb_bytestosend = sizeof (Manufacturer_Descriptor);
               break;
            case 2:
               usb_sendbuffer = (uint8_t *) &Product_Descriptor;
               usb_bytestosend = sizeof (Product_Descriptor);
               break;
            default:
               usb_sendbuffer = NULL;
               usb_bytestosend = 0;
         }
         if (usb_bytestosend > setupPacket->wLength)
            usb_bytestosend = setupPacket->wLength;
         writeBufferToEndpoint ();
         break;
      default:
         stall_control_endpoint ();
   }
}

/**
  * Function handling errors by closing the control endpoints.
  */
void stall_control_endpoint (void)
{
   uint8_t buffer = D11_STALL_ENDPOINT;
   uint8_t cmd;
   // Retrun STALL PID in response to next DATA stage transaction
   cmd = D11_SET_ENDP_STATUS_EP0_IN;
   d11CmdDataWrite (&cmd, &buffer, 1);
   // or in the status stage of the message.
   cmd = D11_SET_ENDP_STATUS_EP0_OUT;
   d11CmdDataWrite (&cmd, &buffer, 1);
} 

/**
  * Function handling timer 1 overflow interrupts.
  * Closes the current USART receive buffer.
  */
SIGNAL(TIMER1_OVF_vect)
{
   // Stop timer 1
   TCCR1B = TIMER_STOP;

   if (eeprom_start == NULL)
      eeprom_start = eeprom_current_start;
  
   // Set the next address, will point to the address were the next packet
   // will be stored. 
   usart_header_list[usart_current_header].next = eeprom_free;
   //printf ("h %x %x\n", usart_header_list[usart_current_header].addr,usart_header_list[usart_current_header].next);

   // Write the usart packet header to the eeprom by adding it to the TWI
   // command queue.
   twi_add_queue_command (TWI_WRITE_EEPROM, USART_PACKET_HEADER_DATA_SIZE + 2,
         (uint8_t *) &usart_header_list[usart_current_header]);

   // Save the received data
   twi_add_queue_command (TWI_WRITE_EEPROM, USART_BUFFER_ROW_LENGTH,
         (uint8_t *) &usart_buffer[usart_buffer_current_line]);

   eeprom_last = eeprom_current_start;

   // Prepare a new header for the next block
   usart_current_header ++;
   if (usart_current_header >= USART_HEADER_LIST_SIZE) // Circular buffer
      usart_current_header = 0;

   usart_state = USART_IDLE;
   PORTB = PORTB & 0xF3;
}

/**
  * Function handling UART receive interrupts.
  */
SIGNAL(USART_RXC_vect)
{
   PORTB = PORTB | 0x30;
   uint8_t status, res;

   // Reset timer 1
   TCNT1 = 0;

   // The recive buffer may contain more than one byte, read them all
   do {
      // Get status, then data from buffer
      status = UCSRA;
      res = UDR;

      // It's a new message. Create a new packet.
      if (usart_state == USART_IDLE)
      {
          PORTB = PORTB | 0x0C;
          usart_state = USART_RECEIVING;

         // Start timer 1
         TCCR1B = TIMER_START_256;
         
         usart_header_list[usart_current_header].addr = eeprom_free;
         usart_header_list[usart_current_header].next = NULL;
         usart_header_list[usart_current_header].size = 0;
         usart_header_list[usart_current_header].error = 0;
         usart_header_list[usart_current_header].reserved8 = 0;
         usart_header_list[usart_current_header].reserved16 = 0;

         eeprom_current_start = eeprom_free;

         // Allocate space in eeprom for the packet header
         eeprom_free += (USART_BUFFER_ROW_LENGTH - 2);
         if (eeprom_free >= (void *) EEPROM_SIZE)   // Circular memory buffer
            eeprom_free = (void *) EEPROM_START_ADDRESS;

         usart_buffer_current_line ++;
         if (usart_buffer_current_line >= USART_BUFFER_SIZE) // Circular buffer
            usart_buffer_current_line = 0;
         usart_buffer_current_pos = 2;

         struct usart_packet_header *temp;
         temp = (struct usart_packet_header *)
            &usart_buffer[usart_buffer_current_line];
         temp->addr = eeprom_free;

         // Allocate space for the first data line
         eeprom_free += (USART_BUFFER_ROW_LENGTH - 2);
         if (eeprom_free >= (void *) EEPROM_SIZE)   // Circular memory buffer
            eeprom_free = (void *) EEPROM_START_ADDRESS;
         
         //printf ("l %x\n", &usart_buffer[usart_buffer_current_line]);
     }

      // If error, increase error flag
      if ( status & ((1<<FE) | (1<<DOR) | (1<<PE)) )
         usart_header_list[usart_current_header].error ++;

      // The current buffer line is full, change to the next
      if (usart_buffer_current_pos == USART_BUFFER_ROW_LENGTH)
      {
         // Write the line to the eeprom by adding it to the TWI command queue
         twi_add_queue_command (TWI_WRITE_EEPROM, USART_BUFFER_ROW_LENGTH,
               (uint8_t *)&usart_buffer[usart_buffer_current_line]);

         usart_buffer_current_line ++;
         if (usart_buffer_current_line >= USART_BUFFER_SIZE) // Circular buffer
            usart_buffer_current_line = 0;
         usart_buffer_current_pos = 2;

         struct usart_packet_header *temp;
         temp = (struct usart_packet_header *)
            &usart_buffer[usart_buffer_current_line];
         temp->addr = eeprom_free;

         // Allocate space for the new data line
         eeprom_free += (USART_BUFFER_ROW_LENGTH - 2);
         if (eeprom_free >= (void *) EEPROM_SIZE)   // Circular buffer
            eeprom_free = (void *) EEPROM_START_ADDRESS;
         
         //printf ("l %x\n", &usart_buffer[usart_buffer_current_line]);
      }

      // Add the byte to the buffer
      usart_buffer[usart_buffer_current_line][usart_buffer_current_pos] = res;
      usart_buffer_current_pos ++;

      // Count size
      usart_header_list[usart_current_header].size ++;
      
   } while (UCSRA & (1 << RXC));
   //Continue to read while there's more bytes in receive buffer

   // Reset timer 1
   TCNT1 = 0;

   PORTB = PORTB & 0xCF;
}

/**
 * Function called when a TWI interrupt occures.
 */
SIGNAL(TWI_vect)
{
   PORTB = PORTB | 0xC0;
   uint8_t status;
   uint8_t tmp;

   status = TWSR & TW_STATUS_MASK;

   if (status == TW_BUS_ERROR)
   {
      //printf ("TwiBusErr\n");
      TWCR = TWI_CMD_START;
      PORTB = PORTB & 0x3F;
      return;
   }

   switch (twi_state)
   {
      case TWI_START: // Start command sent.
         // Start command succesfuly sent
         if (status == TW_START || status == TW_REP_START)
         {
//            //printf("TwiSt ");
            twi_state = TWI_SEND_COMMAND;
            twi_current_data =
               twi_command_queue[twi_queue_start].data;
            twi_byte_to_process = twi_command_queue[twi_queue_start].size;
            twi_current_pos = 0;
            // Read the next command from the queue
            switch (twi_command_queue[twi_queue_start].cmd)
            {
               case TWI_WRITE_EEPROM:
                  TWDR = EEPROM_ADDRESS | TW_WRITE;
                  // Swap address bytes, the eeprom stores it in oposite order
                  if (!twi_nack) // Already swaped if it's a retry after a NACK
                  {
                     tmp = twi_current_data[0];
                     twi_current_data[0] = twi_current_data[1];
                     twi_current_data[1] = tmp;
                  }
                  //printf ("wr %x %x\n", twi_current_data[0], tmp);
                  break;
               case TWI_READ_EEPROM_ADDR: // Write address to read from
                  TWDR = EEPROM_ADDRESS | TW_WRITE;
                  twi_byte_to_process = 1;
                  // Swap address bytes, the eeprom stores it in oposite order
                  if (!twi_nack) // Already swaped if it's a retry after a NACK
                  {
                     tmp = twi_current_data[0];
                     twi_current_data[0] = twi_current_data[1];
                     twi_current_data[1] = tmp;
                  }
                  //printf ("rd %x %x\n", twi_current_data[0], tmp);
                 break;
               case TWI_READ_EEPROM_DATA: // Read data from address
                  TWDR = EEPROM_ADDRESS | TW_READ;
                  // The first two bytes are the address,
                  twi_current_pos = 2; 
                  TWCR = TWI_CMD_CONTINUE_WITH_ACK;
                  PORTB = PORTB & 0x3F;
                  twi_nack = false;
                  return;
               case TWI_WRITE_USB_CMD:
                  TWDR = USB_CMD_ADDRESS | TW_WRITE;
                  break;
               case TWI_WRITE_USB_DATA:
                  TWDR = USB_DATA_ADDRESS | TW_WRITE;
                  break;
               case TWI_READ_USB_DATA:
                  TWDR = USB_DATA_ADDRESS | TW_READ;
                  if (twi_byte_to_process == 0) // Don't ack
                     break;
                  TWCR = TWI_CMD_CONTINUE_WITH_ACK;
                  PORTB = PORTB & 0x3F;
                  twi_nack = false;
                  return;
               default:
                  //printf ("unknown cmd %x\n",
                        //twi_command_queue[twi_queue_start].cmd);
                  get_next_twi_command (0);
                  twi_nack = false;
                  return;
            }
            twi_nack = false;
            TWCR = TWI_CMD_CONTINUE;
         }
         else  // Start command failed
         {
            twi_nack = false;
            TWCR = TWI_CMD_START;
            //printf ("STfailed\n");
         }
         break;

      case TWI_SEND_COMMAND: // Get results from write of address and command
         if (status == TW_MT_SLA_ACK || status == TW_MR_SLA_ACK)
         {
            switch (twi_command_queue[twi_queue_start].cmd)
            {
               case TWI_WRITE_EEPROM:
               case TWI_READ_EEPROM_ADDR:
                  //printf ("S %x", twi_current_data[twi_current_pos]);
               case TWI_WRITE_USB_CMD:
               case TWI_WRITE_USB_DATA:
//                  //printf ("S %x", twi_current_data[twi_current_pos]);
                  twi_state = TWI_SEND;
                  TWDR = twi_current_data[twi_current_pos];
                  TWCR = TWI_CMD_CONTINUE;
                  break;
               case TWI_READ_USB_DATA:
               case TWI_READ_EEPROM_DATA:
//                  //printf ("R");
                  twi_state = TWI_READ;
                  // Only send ack if there's more than one byte left to read
                  // The last byte shouldn't be acked.
                  if (twi_current_pos >= twi_byte_to_process)
                     TWCR = TWI_CMD_CONTINUE;
                  else
                     TWCR = TWI_CMD_CONTINUE_WITH_ACK;
                  break;
            }
         }
         else  // failed TW_MT_SLA_NACK
         {
            twi_nack = true;
            //printf ("SDCMD NACK %x state:%x\n", status, twi_state);
            twi_state = TWI_START;
            TWCR = TWI_CMD_START;
         }
         break;

      case TWI_SEND: // Get results from the last send, and send the next byte.
         if (status == TW_MT_DATA_ACK)
         {
            // More bytes left to send
            if (twi_current_pos < twi_byte_to_process)
            {
               twi_current_pos++;
               TWDR = twi_current_data[twi_current_pos];
               TWCR = TWI_CMD_CONTINUE;
            }
            // Sending EEPROM address to read from is complete
            else if (twi_command_queue[twi_queue_start].cmd ==
                  TWI_READ_EEPROM_ADDR)
            {
               twi_command_queue[twi_queue_start].cmd =
                  TWI_READ_EEPROM_DATA;   // Change command to read
               twi_state = TWI_START;
               TWCR = TWI_CMD_START;
            }
            // Last byte sent, writing to EEPROM
            else if (twi_command_queue[twi_queue_start].cmd == TWI_WRITE_EEPROM)
            {
               get_next_twi_command (1);  // Send a stop command
            }
            else  // Last byte sent
            {
               get_next_twi_command (0);
            }
         }
         else  // TW_MT_DATA_NACK
         {
            TWCR = TWI_CMD_START;
            twi_state = TWI_START;
            //printf ("SND:NACK\n");
         }
         break;

      case TWI_READ: // Get the result and read the next byte
         if (status == TW_MR_DATA_ACK) // More bytes left to read
         {
            twi_current_data[twi_current_pos] = TWDR;
            //            //printf (" %x", twi_current_data[twi_current_pos]);
            twi_current_pos ++;
            // Don't ack the last byte
            if (twi_current_pos >= twi_byte_to_process)
               TWCR = TWI_CMD_CONTINUE;
            else
               TWCR = TWI_CMD_CONTINUE_WITH_ACK;
         }
         // This is the last byte
         else if (status == TW_MR_DATA_NACK)
         {
            twi_current_data[twi_current_pos] = TWDR;
//            //printf (" %x", twi_current_data[twi_current_pos]);
            get_next_twi_command (0);
         }
         else  // Error
         {
            twi_state = TWI_START;
            TWCR = TWI_CMD_START;
            //printf ("ErrTwiRead\n");
         }
         break;
   }
   PORTB = PORTB & 0x3F;
}

/**
  * Function that fetches the next twi command from the command queue.
  * The bus is restarted if there is a new command else it is stoped.
  * @param stop Flag set if a stop command should be sent before the bus
  * is restarted.
  */
void get_next_twi_command (uint8_t stop)
{
   twi_command_queue[twi_queue_start].cmd =
      TWI_COMMAND_FINISHED;
   twi_queue_start ++;
   if (twi_queue_start >= TWI_QUEUE_SIZE)
      twi_queue_start = 0;

   // No more commands in the queue
   if (twi_queue_start == twi_queue_end)
   {
//      //printf (" Stop\n");
      TWCR = TWI_CMD_STOP;
      twi_state = TWI_IDLE;
   }
   else  // Start the next command
   {
      if (stop)
      {
         TWCR = TWI_CMD_STOP; // Stop command is needed for the eeprom
//         //printf (" stop ");// TODO add a wait when removing the //printf
         uint8_t i;
         for (i=0 ; i < 255 ; i++)   // Delay
            ;
      }
      twi_state = TWI_START;
      TWCR = TWI_CMD_START;
//      //printf ("TWIrestart\n");
   }
}
 
/**
  * Method that prints a char to the UART.
  * @param c The character to print.
  * @return 0 if all ok.
  */
int uart_putchar(char c)
{
   if (c == '\n')
   {
      loop_until_bit_is_set(UCSRA, UDRE);
      //outb(UDR, '\r');
      UDR = '\r';
   }
   loop_until_bit_is_set(UCSRA, UDRE);
   //outb(UDR, c);
   UDR = c;
   if (c == '\r')
   {
      loop_until_bit_is_set(UCSRA, UDRE);
      //outb(UDR, '\n');
      UDR = '\n';
   }
   return 0;
}

/**
  * Function that handles write commands to the PDIUSBD11.
  * Interrupts will be diabled when returning from the function.
  * @param cmd The D11 USB command.
  * @param data A pointer to an array containing the data to write.
  * @param size The number of bytes to write.
  */
void d11CmdDataWrite (uint8_t *cmd, uint8_t *data, uint8_t size)
{
   volatile uint8_t *res;

   // Write the USB command
   res = twi_add_queue_command (TWI_WRITE_USB_CMD, 1, cmd);

   sei ();  // Enable interrupts
   while (*res != TWI_COMMAND_FINISHED)
      ;   // Wait until the command is finished
   cli ();  // Disable interrupts

   // Write the data
   if (size > 0)
   {
      res = twi_add_queue_command (TWI_WRITE_USB_DATA, size, data);
      sei ();  // Enable interrupts
      while (*res != TWI_COMMAND_FINISHED)
         ;   // Wait until the command is finished
      cli ();  // Disable interrupts
   }
}

/**
  * Function that handles read commands to the PDIUSBD11.
  * Interrupts will be diabled when returning from the function.
  * @param cmd The D11 USB command.
  * @param data A pointer to the array where the results should be placed.
  * @param size The number of bytes to read.
  */
void d11CmdDataRead (uint8_t *cmd, uint8_t *data, uint8_t size)
{
   volatile uint8_t *res;

   // Write the USB command
   res = twi_add_queue_command (TWI_WRITE_USB_CMD, 1, cmd);
 
   sei ();  // Enable interrupts
   while (*res != TWI_COMMAND_FINISHED)
      ;   // Wait until the command is finished
   cli ();  // Disable interrupts
 
   // Read the data
   if (size > 0)
   {
      res = twi_add_queue_command (TWI_READ_USB_DATA, size, data);
      sei ();  // Enable interrupts
      while (*res != TWI_COMMAND_FINISHED)
         ;   // Wait until the command is finished
      cli ();  // Disable interrupts
   }
}

/**
  * Function that writes data to an USB endpoint.
  * Interrupts will be diabled when returning from the function.
  * @param endpoint The endpoint to write to.
  * @param data An Array containing the data.
  * @param size The number of bytes to write.
  */
void d11WriteEndpoint (uint8_t endpoint, uint8_t *data, uint8_t size)
{
   uint8_t d11Header[2];
   uint8_t bufferStatus = 0;
   uint8_t cmd;

   d11Header[0] = 0x00;
   d11Header[1] = size;

   // Select endpoint
   cmd = endpoint;
   d11CmdDataRead (&cmd, &bufferStatus, 1);

   // Write header
   cmd = D11_WRITE_BUFFER;
   d11CmdDataWrite (&cmd, d11Header, 2);

   // Write packet
   if (size)
   {
      cmd = D11_WRITE_BUFFER;
      d11CmdDataWrite (&cmd, data, size);
   }

   // Validate buffer
   cmd = D11_VALIDATE_BUFFER;
   d11CmdDataWrite (&cmd, NULL, 0);
}

/**
  * Function that read data from the selected endpoint.
  * @param endpoint The endpoint to read from.
  * @param data The buffer to store the data in.
  * @return The numnber of bytes read from the endpoint.
  */
uint8_t d11ReadEndpoint (uint8_t endpoint, uint8_t *data)
{
   uint8_t d11Header[2];
   uint8_t bufferStatus = 0;
   uint8_t cmd;

   // Select endpoint
   cmd = endpoint;
   d11CmdDataRead (&cmd, &bufferStatus, 1);

   // Check if buffer is full
   if (bufferStatus & 0x01)
   {
      // Reda dummy header - d11 buffer pointer is incremented on each read
      // and is only reset by a Select endpoint command.
      cmd = D11_READ_BUFFER;
      d11CmdDataRead (&cmd, d11Header, 2);
 
      if (d11Header[1])
      {
         cmd = D11_READ_BUFFER;
         d11CmdDataRead (&cmd, data, d11Header[1]);
      }

      // Allow new packets to be accepted
      cmd = D11_CLEAR_BUFFER;
      d11CmdDataWrite (&cmd, NULL, 0);

   }
   return d11Header[1];
}

/**
  * Function that sends the contents of usb_sendbuffer.
  */
void writeBufferToEndpoint (void)
{
   if (usb_bytestosend == 0)
      d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, NULL, 0);
   else if (usb_bytestosend >= 8)
   {
      d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, usb_sendbuffer, 8);
      usb_sendbuffer += 8;
      usb_bytestosend -= 8;
   }
   else
   {
      d11WriteEndpoint (D11_SELECT_ENDPOINT_EP0_IN, usb_sendbuffer,
           usb_bytestosend);
      usb_bytestosend = 0;
   }
}

/**
  * Function that adds a new command to the twi command queue.
  * @param cmd The TWI command.
  * @param size The number of bytes in data.
  * @param data The data to send to / receive from the TWI bus.
  * @return A pointer to the command in the queue.
  */
uint8_t * twi_add_queue_command (uint8_t cmd, uint8_t size, uint8_t *data)
{
   uint8_t *addr = &(twi_command_queue[twi_queue_end].cmd);
   twi_command_queue[twi_queue_end].cmd = cmd;
   twi_command_queue[twi_queue_end].size = size - 1;  // Counting starts from 0
   twi_command_queue[twi_queue_end].data = data;

   twi_queue_end ++;
   if (twi_queue_end >= TWI_QUEUE_SIZE)
      twi_queue_end = 0;

   if (twi_state == TWI_IDLE) // Start the TWI bus if it's idle
   {
      twi_state = TWI_START;
      TWCR = TWI_CMD_START;
   }

   return addr;
}

/**
  * This routine gets called by main to initialize the chip.
  */
void init_io (void)
{
   // Enable PORT B as output
   DDRB = 0xff;
   
   //--- Init USART
   // Set baud rate
   UBRRH = 0x00;
   UBRRL = 0x19;
   // 1 = 115.2k at 3.68MHz
   // 23 (0x17) = 9600 at 3.68MHz
   // 25 (0x19) = 38.4k at 16MHz

   // Enable receiver, transmitter and receiver interrupt
   UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);

   // Set frame format: 8 data, 1 stop bit
   UCSRC = (1 << URSEL) | (3 << UCSZ0);
}

/**
  * Function called by main to initialize the TWI (I2C) interface
  */
void init_twi (void)
{
   // Set state machine
   twi_state = TWI_IDLE;

   // Set status register
   TWSR = 0x00;   // TWPS = 0

   // Set clock rate
   TWBR = 0x0c;   // 12 = 400kHz @ 16MHz

   // Enable interface and interrupt
   TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE);

   // Reset the nack flag
   twi_nack = false;
}

/**
  * Function called by main to initialize the PDIUSBD11 USB chip.
  */
void init_usb (void)
{
   uint8_t buffer[2];
   uint8_t cmd;

   // Disable the hub function
   buffer[0] = 0x00;
   cmd = D11_SET_HUB_ADDRESS;
   d11CmdDataWrite (&cmd, buffer, 1);

   // Set address to zero and enable function
   buffer[0] = 0x80;
   cmd = D11_SET_ADDRESS_ENABLE;
   d11CmdDataWrite (&cmd, buffer, 1);

   // Enable function generic endpoints
   buffer[0] = 0x02;
   cmd = D11_SET_ENDPOINT_ENABLE;
   d11CmdDataWrite (&cmd, buffer, 1);

   // Set mode - enable soft connect
   buffer[0] = 0x97; // Soft connect, Clk run, No Lazy clk, remote wakeup
   buffer[1] = 0x0B; // ClkOut = 4Mhz
   cmd = D11_SET_MODE;
   d11CmdDataWrite (&cmd, buffer, 2);

   sei ();  // d11CmdDataWrite disables interrupts, enable them again
}

/**
  * Main function. Calls the initialization functions and waits for interrupts.
  */
int main (void)
{
    init_io ();
    init_twi ();

    eeprom_start = NULL;
    eeprom_last = NULL;
    eeprom_current_start = NULL;
    eeprom_free = (void *) EEPROM_START_ADDRESS;

    mobitex_state = MOBITEX_IDLE;

    //timer_enable_int (_BV(TOIE1)); // Enable timer 1 interrupts
    TIMSK |= (1 << TOIE1);

    // Open an out pipe to enable //printf.
    //fdevopen(uart_putchar, NULL, 0);

    //printf("--Start--\n");

    // enable interrupts
    sei ();

    PORTB = 0x00;

    init_usb ();

    // Enable interrupts from USB
    //enable_external_int (_BV (INT1) | _BV (INT0));
    GICR |= (1<<INT0) | (1<<INT1);
    MCUCR = 0x03; // INT0 is trigered by a rising edge

    //printf ("-R-\n");

    /* loop forever, the interrupts are doing the rest.
     * If this was a real product, we'd probably put a SLEEP instruction
     * in this loop to conserve power.*/
    for (;;)
        ;

    return 0;
}
