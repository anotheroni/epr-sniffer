/**
  * Header file defining constants ets for the TWI bus.
  *
  * @version 20030624
  * @author Oskar Nilsson
  */

#include <util/twi.h>
#include <inttypes.h>

#ifndef _TWI_H_
#define _TWI_H_

// Addresses
#define EEPROM_ADDRESS 0xA0
#define USB_CMD_ADDRESS 0x36
#define USB_DATA_ADDRESS 0x34

// Standard TWCR command
#define TWI_CMD_CONTINUE (1 << TWINT) | (1 << TWEN) | (1 << TWIE)
#define TWI_CMD_CONTINUE_WITH_ACK (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE)
#define TWI_CMD_START (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE)
#define TWI_CMD_STOP (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (1 << TWIE)

#define TWI_QUEUE_SIZE 30

#define TWI_COMMAND_FINISHED 0xFF

// TWI queue commands
#define TWI_READ_EEPROM_ADDR 1
#define TWI_READ_EEPROM_DATA 2
#define TWI_WRITE_EEPROM 3
#define TWI_READ_USB_DATA 10
#define TWI_WRITE_USB_CMD 11
#define TWI_WRITE_USB_DATA 12

// TWI state machine
#define TWI_IDLE 0
#define TWI_START 1
#define TWI_SEND_COMMAND 2
#define TWI_SEND 3
#define TWI_READ 4

// ---- Function headers ----
__inline__ void twi_start (void);
__inline__ void twi_stop (void);

// ---- Data structures ----
struct twi_queue_entry
{
   uint8_t cmd;
   uint8_t size;  /// Counting starts from 0. To write 4 bytes set size = 3
   void* data;
};

#endif // _TWI_H_
