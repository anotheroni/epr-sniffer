EPR - Electronic Patient Record Masters Thesis

A serial sniffer with a USB connection running on a Atmel ATmega16

# Requirements

## Debian

apt-get install gcc-avr avr-libc

# Build

make

# Design

The sniffer hardware is built around an Atmel AVR ATmega16
(http://www.atmel.com/products/microcontrollers/avr/)
microcontroller. It has no integrated USB functionality; therefore a USB chip
PDIUSB11 from Philips is used to handle USB connectivity. There is also an
external memory chip used by the sniffer to store messages as needed.
ATmega16 only have 1KB of SRAM and 512 Bytes of EEPROM.,there is 16KB of Flash memory but it is only used to store program code.
To ensure that messages are not lost due to the memory being full an external EEPROM memory chip with greater capacity is used.

(See doc/design.png)

The receiving pin of the UART on the ATmega16 is connected to the serial cable
connecting the printer with the computer. When a byte is sent
to the printer by the computer it is also received by the sniffer.
The bytes are stored in a RAM buffer in ATmega16 in blocks of 8 bytes.

(see doc/memory-architecture.png)

When 8 bytes has been received the block is transferred to the external EEPROM.
To connect the EEPROM to the ATmega16 the I2C-bus is used (called TWI by Atmel).
The blocks that belong to each message is stored in a sequence beginning with a
special header block that contains the size of the message and the address to
first block after the message.

To communicate with the PC client a PDIUSB11 chip
from Philips is used. The USB chip is connected to the ATmega16 with the same
I2C-bus as the EEPROM. When the sniffer is connected to the PC client
and the USB-handshake is completed, the client application can ask
the sniffer for stored messages.

If there are messages the first message is transferred block by block from the
EEPROM to the buffer in PDIUSB11. The USB send buffer is 8 bytes; this is the
reason for choosing a block size of 8 bytes. If the transfer succeeds the
PC application returns an ACK and the message is removed from the
EEPROM. The PC application can now ask for the next message. If no
message exists a special "no messages" block is returned.

On the PC client an example USB device driver from Intel is used. The
driver is called Bulkusb.sys and is a USB bulk I/O sample mini driver that
transfers asynchronous data packets.

## AVR Firmware

The firmware is written in C and compiled with AVR-GCC. It is built
using two state machines, one for USB and one for the I2C bus. Most of the code
uses interrupts to prevent the firmware from locking by waiting for an event in
a loop. This is important because the UART buffer must not overflow. An
overflow will mean that a part of a message is lost. The device being a passive
sniffer; has no way to ask for a retransmission. The USB stack also has timing
constraints that must be fulfilled.

The use of interrupts forces the use of data buffers and command queues. An
operation with a higher priority can interrupt another operation or force it to
wait. The sharing of the I2C-bus between the USB chip and the EEPROM means
that operations may have to wait for the bus to be free. A write to the EEPROM
may have to wait until a read from the USB-chip is completed etc.

The EEPROM is organized as a circular buffer with a pointer to the first
message header, the last message header, the current message header and the
first free block. When a new message is received by the UART a new message
header is allocated and the current message header pointer is set to point to
it. Data blocks are allocated after the header and the pointer to the next free
block is incremented. When the last byte has arrived the header is written and
the last message pointer is updated to point to the current message header. An
important note is that there is no check for overflow, if the EEPROM is not
emptied there is a risk that the memory will overflow and old messages are
overwritten.

## I2C/TWI code

When the firmware needs to access the I2C-bus (called
TWI-bus by Atmel) a command is added to the I2C command queue. If the queue is
empty and the bus is free it can be started with the new command, else the
command has to wait until the commands ahead in the queue are completed. When
the bus start/restart command has been executed, the firmware waits for an
interrupt that signals that the microcontroller has become master of the I2C
-bus. What happens next depend on the state of the I2C-state machine.

If it is in the START state the next command is read from the command queue.
Depending on the command (read from EEPROM, write to USB etc.) the correct
address and write/read command is written to the bus. The state of the
I2C-state machine is set to Send Command and the firmware waits for the next
I2C-interrupt.

If it is in the SEND COMMAND state the result from the I2C-command is checked,
if it was a success a byte will be written on the bus from the current buffer.
Even read operations do this write, the address to read from has to be sent.
The state is set to SEND. The exception being when reading from an USB data
buffer or when in the read phase and reading from the EEPROM; then a
continue command is sent to the bus and the state is set to READ. If the
command failed the bus will be restarted and the state set to START.

If it is in the SEND state the result from the last write is checked, if it was
a failure the bus is restarted and the state set to START. If it succeeded four
things may happen:
* There is more data to write; write the next byte and send a continue command.
* Writing the address to read from is completed; restart the bus and set the
* state to START to prepare to start reading data.
* The last byte was written to the EEPROM; send a stop command to the bus and read the next command in the command-queue.
* The last byte was written to the USB-chip; send a restart command to the bus and read the next command from the command-queue.

If it is in the READ state the result form the last read is checked, if
it was a failure the bus is restarted and the state set to START. If it
succeeded three things may happen:
* There are many bytes left to read: Copy the new byte to the buffer and send a continue command.
* There is only one byte left to read: Copy the new byte to the buffer and send a continue without ACK command.  This is the last byte: Copy the new byte to the buffer, send a restart command to the bus and read the next command from the command-queue.

## PDIUSBD11

PDIUSBD11 is an USB chip from Philips (http://www.semiconductors.philips.com/pip/PDIUSBD11.html) compliant with the USB 1.1 specification.
It is designed to be connected to a microcontroller via
the I2C-bus. It is a low voltage chip operating with a 3.3 V supply. I/O is 5 V
tolerant which makes it simple to use with a 5 V microcontroller. The chip
contain many features that reduces the number of peripheral components, this
simplifies the hardware design and reduces the total price. There are four
endpoints, including the control endpoint. Each has its own 8 byte buffer.

It implements the USB physical-layer and link-layer in hardware
leaving the protocol-layer to be implemented in firmware in the
microcontroller.

The PDIUSBD11 chip has a slave I2C-interface for communication with a
microcontroller. The chip has two addresses on the bus one command address and
one data address. e.g. To read data from an endpoint buffer a command is sent
to the command address, then data from that buffer can be read from the data
address.

## USB stack

The implementation for handling the USB stack is based on a simple state
machine and requests from the USB-chip. The code is based on a stack from
beyond logic (http://retired.beyondlogic.org/usb/usbhard.htm) designed for a PIC16 microcontroller. When the
hardware is started the USB-chip is initialized and configured, and ready to be
plugged in to a computer. The lower USB-stack layers are implemented in
hardware but some parts that are needed during handshaking when the device is
plugged in must be implemented in the firmware. When the USB-chip needs
attention from the microcontroller an interrupt (external interrupt 1) is sent.
The microcontroller then reads the interrupt register from the command address
and takes appropriate action.
Five types of interrupts are handled:
* Bus reset, the USB-bus has been reset, reinitialize the USB-chip.
* Endpoint 0 out, control commands received.
* Endpoint 0 in, control commands sent.
* Endpoint 1 out, application data received.
* Endpoint 1 in, application data sent.

During handshaking bus resets and control endpoint traffic occurs. On the
control endpoint a number of different get descriptor requests are sent;
* Device descriptor
* Configuration descriptor
* String descriptor
These contain information about the device (device class, endpoint
configurations, manufacturer name etc.). The replies are sent on endpoint 1 in.
At this point of the handshake process the device driver is also loaded on the
host, so that applications on the host can access the device.

After this the device is ready to handle application request on endpoint 1.
There are three commands the host can send to the device:
* Get packet; to get the first message in the EEPROM.
* ACK; If the message was received OK.
* NACK; if receiving the message failed.

# Hardware

## Components

* ATmega16
* PDIUSBD11
TODO

## Schematics

TODO
