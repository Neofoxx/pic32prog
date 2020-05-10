/*
 * Interface to PIC32 JTAG port using FT2232-based USB adapter.
 * Supported hardware:
 *  - Olimex ARM-USB-Tiny adapter
 *  - Olimex ARM-USB-Tiny-H adapter
 *  - Olimex ARM-USB-OCD-H adapter
 *  - Olimex MIPS-USB-OCD-H adapter
 *  - Bus Blaster v2 from Dangerous Prototypes
 *  - TinCanTools Flyswatter adapter
 *
 * Copyright (C) 2011-2013 Serge Vakulenko
 *
 * This file is part of PIC32PROG project, which is distributed
 * under the terms of the GNU General Public License (GPL).
 * See the accompanying file "COPYING" for more details.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include "adapter.h"
#include "pic32.h"
#include "serial.h"

typedef struct {
    // Common part
    adapter_t adapter;
	char name [64];	// Make const char * again later or something.	

    // Transmit buffer
    uint8_t output [2048];	// TODO - change to match actual MCU code.
    int bytes_to_write;

    // Receive buffer.
    uint8_t input [2048]; 
    int bytes_to_read;
	int bytes_in_buffer;
	int current_pos_input;

	uint32_t speed;
	unsigned interface;	// Remove later?
    uint32_t use_executive;
    uint32_t serial_execution_mode;
	uint32_t way;	// old or new

	// These should be int8_t, 'cos chars.
	uint8_t capabilities_mcu[64];
	uint8_t capabilities_version_fw[64];
	uint8_t capabilities_version_hw[64];


} neofoxx_adapter_t;


/* TMS header and footer defines */
#define TMS_HEADER_COMMAND_NBITS        4
#define TMS_HEADER_COMMAND_VAL          0b0011
#define TMS_HEADER_XFERDATA_NBITS       3
#define TMS_HEADER_XFERDATA_VAL         0b001
#define TMS_HEADER_XFERDATAFAST_NBITS   3
#define TMS_HEADER_XFERDATAFAST_VAL     0b001
#define TMS_HEADER_RESET_TAP_NBITS      6
#define TMS_HEADER_RESET_TAP_VAL        0b011111

#define TMS_FOOTER_COMMAND_NBITS        2
#define TMS_FOOTER_COMMAND_VAL          0b01
#define TMS_FOOTER_XFERDATA_NBITS       2
#define TMS_FOOTER_XFERDATA_VAL         0b01
#define TMS_FOOTER_XFERDATAFAST_NBITS   2
#define TMS_FOOTER_XFERDATAFAST_VAL     0b01

#define SET_MODE_TAP_RESET  0
#define SET_MODE_EXIT       1
#define SET_MODE_ICSP_SYNC  2

#define PACKET_BYTE			0x70	// 'p'	

#define COMMAND_GET_INFO		0
#define COMMAND_SET_SPEED		1
#define COMMAND_SET_PROG_MODE	2
#define COMMAND_SET_PIN_IO_MODE	3	// Input/output. These three are useful for custom stuff
#define COMMAND_SET_PIN_WRITE	4	// Write to pin
#define COMMAND_SET_PIN_READ	5	// Read pin value
#define COMMAND_SEND			6	// Send command
#define COMMAND_XFER_INSTRUCTION	7	// xferInstruction, but performed on MCU

#define PROG_MODE_TRISTATE	0
#define PROG_MODE_JTAG		1
#define PROG_MODE_ICSP		2

#define PIN_TMS		0
#define PIN_TCK		1
#define PIN_TDI		2
#define PIN_TDO		3
#define PIN_MCLR	4

#define PIN_DIR_OUTPUT	0
#define PIN_DIR_INPUT	1
#define PIN_VAL_LOW		0
#define PIN_VAL_HIGH	1

// The above are condensed into these.
#define PIN_SET_OUTPUT_LOW	0
#define PIN_SET_OUTPUT_HIGH	1
#define PIN_SET_INPUT		2

// For benchmarking old vs. new way.
#define WAY_OLD	0
#define WAY_NEW 1


/*
 * Calculate checksum.
 */
static unsigned calculate_crc(unsigned crc, unsigned char *data, unsigned nbytes)
{
    static const unsigned short crc_table [16] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    };
    unsigned i;

    while (nbytes--) {
        i = (crc >> 12) ^ (*data >> 4);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        i = (crc >> 12) ^ (*data >> 0);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        data++;
    }
    return crc & 0xffff;
}

static void add_to_packet(neofoxx_adapter_t *a, uint8_t *data, uint32_t nbytes){
	// NB! Increase a-> bytes-to-write or something...

	// If packet not created, create packet
	if (0 == a->bytes_to_write){
		a->output[a->bytes_to_write++] = PACKET_BYTE;
		a->output[a->bytes_to_write++] = 0;	// Length byte 1. Init at 0.
		a->output[a->bytes_to_write++] = 0;	// Length byte 0. Init at 0.
	}

	// If created, add to packet
	memcpy(&(a->output[a->bytes_to_write]), data, nbytes);
	a->bytes_to_write = a->bytes_to_write + nbytes;

	// Should also check for maxLength, but can do that later.

	// In write function, add CRC and send
} 

/*
 * Send a packet to USB device.
 */
static void bulk_write(neofoxx_adapter_t *a, unsigned char *output, int nbytes){

	// Add CRC to packet. Send.
	uint32_t position = 0;
	uint32_t counter = 0;
	int32_t temp;
	uint16_t shortTemp;
	uint32_t i;

	if (debug_level > 2) {
		fprintf(stderr, "Neofoxx: In function bulk write\n");
	}

	if (4 > a->bytes_to_write){
		fprintf(stderr, "ERR: Less than 4 bytes in bulk write\n");
		if (debug_level > 2){
			for (i=0; i<a->bytes_to_write; i++){
				fprintf(stderr, "%c%02x", i ? '-' : ' ', a->output[i]);
			}
		}
		return;
	}


	uint32_t crc = 0;
	for (counter = 2; counter < a->bytes_to_write; counter++){
		crc = crc + a->output[counter];
	}
	a->output[a->bytes_to_write++] = (uint8_t)(crc & 0xFF);	// Add CRC, increase counter by 1


	// First save length of data to be sent. Length is total packet - 2
	shortTemp = a->bytes_to_write - 3;// Minus 1B for 'p', minus 2B for length
	//memcpy(&(a->output[1]), &shortTemp, sizeof(shortTemp)); 
	// Endiannes ><. Maybe change later or something.
	a->output[2] = shortTemp >> 8;
	a->output[1] = shortTemp & 0xFF;

	

	if (debug_level > 1) {
		int32_t i;
		fprintf(stderr, "Bulk write %d bytes:", a->bytes_to_write);
		for (i=0; i<a->bytes_to_write; i++){
			fprintf(stderr, "%c%02x", i ? '-' : ' ', a->output[i]);
		}
		fprintf(stderr, "\n");
	}

	while (0 < a->bytes_to_write){
		temp = serial_write(&(a->output[position]), a->bytes_to_write);
		if (0 > temp){
			// Error occured during writing.
			fprintf(stderr, "Error during writing\n");
		}
		else{
			position = position + temp;
			a->bytes_to_write = a->bytes_to_write - temp;
		}
	}
}

/*
 * If there are any data in transmit buffer -
 * send them to device.
 */
static void neofoxx_flush_output(neofoxx_adapter_t *a)
{
    int32_t bytes_read;

    if (a->bytes_to_write <= 0){
        return;
	}

	// Write what we have to write
    bulk_write(a, a->output, a->bytes_to_write);
    a->bytes_to_write = 0;
    if (a->bytes_to_read <= 0){
        return;
	}
	
	//fprintf(stderr, "Reading data\n");
    /* Get reply. */
    a->bytes_in_buffer = 0;
    while (a->bytes_in_buffer < a->bytes_to_read) {
 		// Get data from device.
		// Don't forget to increment some bytes_in_buffer counter
		bytes_read = serial_read(&(a->input[a->bytes_in_buffer]),
						 (a->bytes_to_read - a->bytes_in_buffer), 1000);
//		fprintf(stderr, "Read %d bytes\n", bytes_read);
		a->bytes_in_buffer = a->bytes_in_buffer + bytes_read;
	}
    a->bytes_to_read = 0;
	a->current_pos_input = 0;
}

//
//	Get capabilities of the debug probe
//
static void neofoxx_get_capabilities(neofoxx_adapter_t *a){
	// Send the info packet, receive it, and decode it.
	// In it should be:
	// - The MCU itself,
	// - Speed support (limited, full)
	// - VERSION of FW - important for SW support!
	// - Something something something.

	// First FLUSH the output -> we'll leave that to the user/prograammer
	// Reset incoming buffer

	// Send proper packet
	uint8_t data[8];
	uint32_t nbytes = 0;
	data[nbytes++] = COMMAND_GET_INFO;
	add_to_packet(a, data, nbytes);

	a->bytes_to_read = 128;	// Fixed length read.
	neofoxx_flush_output(a);
	
	

	// Receive data - already done in flush_output
	
	// Decode and display :D
	if (debug_level > 1){
		fprintf(stderr, "Got %d data from device:\n%s\n", a->bytes_in_buffer, a->input);
	}
	char* token = strtok((char*)a->input, "\n");
	while (token != NULL){
		//printf("%s\n", token);
		if (0 == strncmp(token, "INFO", 4)){
			// INFO part / first line, nothing to do
		}
		else if (0 == strncmp(token, "MCU:", 4)){
			// MCU part
			uint8_t * position = (uint8_t *)strchr(token, ' ');
			strcpy((char*)a->capabilities_mcu, (char*)position+1);	// Remove ' '
			printf("MCU is %s\n", a->capabilities_mcu);
		}
		else if (0 == strncmp(token, "MODE:", 5)){
			uint8_t * position = (uint8_t *)strchr(token, ' ');
			strcpy((char*)a->capabilities_mcu, (char*)position+1);	// Remove ' '
			printf("MODE is %s\n", a->capabilities_mcu);			
		}
		else if(0 == strncmp(token, "NAME:", 5)){
			uint8_t * position = (uint8_t *)strchr(token, ' ');
			strcpy((char*)a->name, (char*)position+1);	// Remove ' '
			printf("NAME is %s\n", a->name);
		}
		// Others maybe later

		token = strtok(NULL, "\n");
	}

	a->bytes_in_buffer = 0;
}


// Poke the adapter into the proper pin state.
static void neofoxx_setProgMode(neofoxx_adapter_t *a, int mode, int immediate){
	uint8_t data[8];
	uint32_t nbytes = 0;
	data[nbytes++] = COMMAND_SET_PROG_MODE;
	data[nbytes++] = mode;
	add_to_packet(a, data, nbytes);
	
	// No read-back

	if (immediate){
		neofoxx_flush_output(a);
	}
}




// Control pins individually, which can be quite handy.
// Just don't forget which state you're in :D
static void neofoxx_setPins(neofoxx_adapter_t *a, int preset, int pin,
							int pin_dir, int pin_val, int immediate) {

	// In this function, if called with preset, it calls the TRISTATE/JTAG/ICSP
	// setup function. If called with -1, it'll send the pin_dir and pin_val.
	
	uint8_t data[8];
	uint32_t nbytes = 0;
	data[nbytes++] = COMMAND_SET_PIN_IO_MODE;
	data[nbytes++] = pin;
	if (PIN_DIR_INPUT == pin_dir){
		data[nbytes++] = PIN_SET_INPUT;
	}
	else{	// PIN_DIR_OUTPUT
		if (PIN_VAL_LOW == pin_val){
			data[nbytes++] = PIN_SET_OUTPUT_LOW;
		}
		else{
			data[nbytes++] = PIN_SET_OUTPUT_HIGH;
		}
	}

	add_to_packet(a, data, nbytes);
	
	// No read-back

	if (immediate){
		neofoxx_flush_output(a);
	}

}

static void neofoxx_send(neofoxx_adapter_t *a,
    uint32_t tms_prolog_nbits, uint32_t tms_prolog,
    uint32_t tdi_nbits, uint64_t tdi,
    uint32_t tms_epilog_nbits, uint32_t tms_epilog, int32_t read_flag){

	// Just construct a proper command and add to packet.
	uint8_t data[64];
	uint32_t nbytes = 0;
	data[nbytes++] = COMMAND_SEND;

	// TMS epilog
	memcpy(&(data[nbytes]), &tms_prolog_nbits, sizeof(tms_prolog_nbits));
	nbytes = nbytes + sizeof(tms_prolog_nbits);
	memcpy(&(data[nbytes]), &tms_prolog, sizeof(tms_prolog));
	nbytes = nbytes + sizeof(tms_prolog);

	// TDI
	memcpy(&(data[nbytes]), &tdi_nbits, sizeof(tdi_nbits));
	nbytes = nbytes + sizeof(tdi_nbits);
	memcpy(&(data[nbytes]), &tdi, sizeof(tdi));
	nbytes = nbytes + sizeof(tdi);

	// TMS epilog
	memcpy(&(data[nbytes]), &tms_epilog_nbits, sizeof(tms_epilog_nbits));
	nbytes = nbytes + sizeof(tms_epilog_nbits);
	memcpy(&(data[nbytes]), &tms_epilog, sizeof(tms_epilog));
	nbytes = nbytes + sizeof(tms_epilog);

	// read flag
	memcpy(&(data[nbytes]), &read_flag, sizeof(read_flag));
	nbytes = nbytes + sizeof(read_flag);

	add_to_packet(a, data, nbytes);
	if (read_flag){
		a->bytes_to_read = a->bytes_to_read + sizeof(uint64_t);	// Important!
	}
	
	// assert here, but uint64_t should == 8
}

static uint64_t neofoxx_recv(neofoxx_adapter_t *a){
    uint64_t word;

    /* Send a packet if there is anything to send */
    neofoxx_flush_output(a);

	uint32_t i;

/*
	fprintf(stderr, "In _recv\n");
	for (i=0; i<a->bytes_in_buffer; i++){
		fprintf(stderr, "%c%02x", i ? '-' : ' ', a->input[i]);
	}
*/	

	/* Todo here, check if there are more than just 8 bytes in the receive buffer
	 * 'cos currently this works for just one uint64_t */

    /* Process a reply: one 64-bit word. */
	// Increase/decrease counter or something!
	// Also, all packet reconstruction already done on device.
	// Can check if current_pos_input equals bytes_in_buffer.
    memcpy(&word, &(a->input[a->current_pos_input]), sizeof(word));
	a->current_pos_input = a->current_pos_input + sizeof(word);
    return(word);
}

static uint32_t neofoxx_bitReversal(uint32_t input){
    uint32_t output = 0;
    uint32_t counter = 0;
    for(counter=0; counter<32; counter++){
        if(input & (1<<counter)){
            output = output | (1<<(31-counter));
        }
    }
    return output;
}

static void neofoxx_setMode(neofoxx_adapter_t *a, uint32_t mode, uint32_t immediate){
    
	if (debug_level > 2){
		fprintf(stderr, "In _setMode\n");
	}
    if (SET_MODE_TAP_RESET == mode){
        /* TMS 1-1-1-1-1-0 */
        neofoxx_send(a, TMS_HEADER_RESET_TAP_NBITS, TMS_HEADER_RESET_TAP_VAL, 0, 0, 0, 0, 0);
    }
    else if (SET_MODE_EXIT == mode){
        /* TMS 1-1-1-1-1 */
        neofoxx_send(a, 5, 0x1F, 0, 0, 0, 0, 0);
    }
    else if (SET_MODE_ICSP_SYNC == mode && (INTERFACE_JTAG == a->interface || INTERFACE_DEFAULT == a->interface)){
        // In JTAG mode, send the MCHP key to enter ICSP on the TMS line.
		uint32_t entryCode = 0x4D434850;   // MCHP in ascii.
		entryCode = neofoxx_bitReversal(entryCode);
		neofoxx_send(a, 8, entryCode & 0xFF, 0, 0, 0, 0, 0);
		neofoxx_send(a, 8, (entryCode >> 8) & 0xFF, 0, 0, 0, 0, 0);
		neofoxx_send(a, 8, (entryCode >> 16) & 0xFF, 0, 0, 0, 0, 0);
		neofoxx_send(a, 8, (entryCode >> 24) & 0xFF, 0, 0, 0, 0, 0);

    }
    else{
        fprintf(stderr, "neofoxx_setMode called with invalid mode, quitting\n");
        exit(-1);
    }
    if (immediate){
        neofoxx_flush_output(a);
    }
	if (debug_level > 2){
		fprintf(stderr, "End of _setMode\n");
	}
}

static void neofoxx_sendCommand(neofoxx_adapter_t *a, uint32_t command, uint32_t immediate){
    // All 5-bit commands. The 8-bit ones are command_DR, go through XferData
    if (MTAP_COMMAND != command && TAP_SW_MTAP != command       // MTAP commands
        && TAP_SW_ETAP != command && MTAP_IDCODE != command     // MTAP commands
        && ETAP_ADDRESS != command && ETAP_DATA != command      // ETAP commands
        && ETAP_CONTROL != command && ETAP_EJTAGBOOT != command // ETAP commands
        && ETAP_FASTDATA != command && ETAP_NORMALBOOT != command){ // ETAP commands
        fprintf(stderr, "neofoxx_sendCommand called with invalid command 0x%02x, quitting\n", command);
        exit(-1);   // TODO make exit procedure
    }

    if (MTAP_COMMAND != command && TAP_SW_MTAP != command 
        && TAP_SW_ETAP != command && MTAP_IDCODE != command){   // MTAP commands
        neofoxx_send(a, TMS_HEADER_COMMAND_NBITS, TMS_HEADER_COMMAND_VAL,
                    MTAP_COMMAND_NBITS, command,
                    TMS_FOOTER_COMMAND_NBITS, TMS_FOOTER_COMMAND_VAL,
                    0);
    }
    else if (ETAP_ADDRESS != command && ETAP_DATA != command    // ETAP commands
            && ETAP_CONTROL != command && ETAP_EJTAGBOOT != command 
            && ETAP_FASTDATA != command && ETAP_NORMALBOOT != command){
        neofoxx_send(a, TMS_HEADER_COMMAND_NBITS, TMS_HEADER_COMMAND_VAL,
                    ETAP_COMMAND_NBITS, command,
                    TMS_FOOTER_COMMAND_NBITS, TMS_FOOTER_COMMAND_VAL,
                    0);
    }
    if (immediate){
        neofoxx_flush_output(a);
    }

}


static uint64_t neofoxx_xferData(neofoxx_adapter_t *a, uint32_t nBits, 
                uint32_t iData, uint32_t readFlag, uint32_t immediate){
    neofoxx_send(a, TMS_HEADER_XFERDATA_NBITS, TMS_HEADER_XFERDATA_VAL,
                    nBits, iData, 
                    TMS_FOOTER_XFERDATA_NBITS, TMS_FOOTER_XFERDATA_VAL,
                    readFlag);
    if (readFlag){
        /* Flushes the output data, and returns the data */
        return neofoxx_recv(a);
    }
    if (immediate){
        neofoxx_flush_output(a);
    }
    return 0;
}

// In the PC_slow implementation, immediate is implied 
// (we get PrACC, even though we more or less ignore it :p).
// We could receive at the end and check the result?
// Hm... better not, maintain same functionality, make MCU_fast do that.
static uint64_t neofoxx_xferFastData_PC_slow(neofoxx_adapter_t *a, unsigned word, uint32_t readFlag)
{
    uint64_t temp;
    neofoxx_send(a, TMS_HEADER_XFERDATAFAST_NBITS, TMS_HEADER_XFERDATAFAST_VAL,
                    33, (unsigned long long) word << 1,
                    TMS_FOOTER_XFERDATAFAST_NBITS, TMS_FOOTER_XFERDATAFAST_VAL,
                    1);
    temp = neofoxx_recv(a);
    if (!(temp & 0x01)){
        fprintf(stderr, "Warning: PrACC not set in xferFastData\n");
    }
    if (readFlag){
        return temp;
    }
    return 0;
}

// In the MCU_fast implementation, this will execute on the programmer
// This means we can queue these instructions, and save on the bandwidth :)
// Also function is now void, you are responsible for popping bytes off of it.
static void neofoxx_xferFastData_MCU_fast(neofoxx_adapter_t *a, unsigned word, uint32_t readFlag){
	neofoxx_send(a, TMS_HEADER_XFERDATAFAST_NBITS, TMS_HEADER_XFERDATAFAST_VAL,
                    33, (unsigned long long) word << 1,
                    TMS_FOOTER_XFERDATAFAST_NBITS, TMS_FOOTER_XFERDATAFAST_VAL,
                    readFlag);
	
	// Err, that's not right, is it?
	// Oh, yes it is, just get the data from recv_ and readout the PrACC. Derp.
	
}


// In the PC_slow implementation, immediate is implied, 
// since it needs to pass data back and fort (CONTROL_PROBEN).
// This means that we can't really queue these instructions == no speedup == slow.
static void neofoxx_xferInstruction_PC_slow(neofoxx_adapter_t *a, uint32_t instruction)
{
    unsigned ctl;
    unsigned maxCounter = 0;

    if (debug_level > 1){
        fprintf(stderr, "%s: xfer instruction PC_slow %08x\n", a->name, instruction);
	}

    /* Select Control Register */
    neofoxx_sendCommand(a, ETAP_CONTROL, 1); // ETAP_CONTROL, immediate

    // Wait until CPU is ready
    // Check if Processor Access bit (bit 18) is set
    do {
        ctl = neofoxx_xferData(a, 32, (CONTROL_PRACC | CONTROL_PROBEN 
            | CONTROL_PROBTRAP | CONTROL_EJTAGBRK), 1, 1);    // Send data, readflag, immediate don't care
        // For MK family, PRACC doesn't cut it.
//        if ((! (ctl & CONTROL_PRACC))){
        if (!(ctl & CONTROL_PROBEN)){  
            fprintf(stderr, "xfer instruction, ctl was %08x\n", ctl);
            maxCounter++;
            if (maxCounter > 40){
                fprintf(stderr, "Processor still not ready. Quitting\n");
                exit(-1);   // TODO exit procedure
            }
            mdelay(1000);
        }
//    } while (! (ctl & CONTROL_PRACC));
    } while (! (ctl & CONTROL_PROBEN));

    /* Select Data Register */
    neofoxx_sendCommand(a, ETAP_DATA, 1);    // ETAP_DATA, immediate

    /* Send the instruction */
    neofoxx_xferData(a, 32, instruction, 0, 1);  // Send instruction, don't read, immediate

    /* Tell CPU to execute instruction */
    neofoxx_sendCommand(a, ETAP_CONTROL, 1); // ETAP_CONTROL, immediate
    /* Send data. */
    neofoxx_xferData(a, 32, (CONTROL_PROBEN | CONTROL_PROBTRAP), 0, 1);   // Send data, no readback, immediate
}

// In the MCU_fast implementation, this will execute on the programmer
// This means we can queue these instructions, and save on the bandwidth :)
static void neofoxx_xferInstruction_MCU_fast(neofoxx_adapter_t *a, uint32_t instruction){
	if (debug_level > 1){
        fprintf(stderr, "%s: xfer instruction MCU_fast %08x\n", a->name, instruction);
	}

	uint8_t data[64];
	uint32_t nbytes = 0;
	data[nbytes++] = COMMAND_XFER_INSTRUCTION;
	memcpy(&(data[nbytes]), &instruction, sizeof(instruction));
	nbytes = nbytes + sizeof(instruction);
	add_to_packet(a, data, nbytes);

	// Also read 4B of response.
	// 0 is success, MSB set is FAIL
	a->bytes_to_read = a->bytes_to_read + sizeof(uint32_t);	// Important!
}

static void neofoxx_speed(neofoxx_adapter_t *a, int khz)
{
	fprintf(stderr, "Function %s not implemented yet\n", __func__); 
	
	// Todo, if at all
	// For some devices, speed will be variable (in khz/divisor steps).
	// For some (bitbangers) there'll be just fast/maybe medium/slow. Maybe.
}

static void neofoxx_close(adapter_t *adapter, int power_on)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;

    neofoxx_sendCommand(a, TAP_SW_ETAP, 1);  
    neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);   // Send TAP reset, immediate
    mdelay(10);

    // Toggle MCLR.
    neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // Reset, immediate
    mdelay(100);    // Hold in reset for a bit, so it auto-runs afterwards 
    neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // No reset, immediate

    serial_close();
    free(a);
}

/*
 * Read the Device Identification code
 */
static unsigned neofoxx_get_idcode(adapter_t *adapter)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
    unsigned idcode;

    /* Reset the JTAG TAP controller: TMS 1-1-1-1-1-0.
     * After reset, the IDCODE register is always selected.
     * Read out 32 bits of data. */
    neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);    /* Send TAP reset, immediate */
    idcode = neofoxx_xferData(a, 32, 0, 1, 1);    /* Send 32 0s, receive, immediate (don't care) */
    return idcode;
}

/* Sends the special command to enter ICSP mode */
static void neofoxx_enter_icsp(neofoxx_adapter_t *a)
{
	// TODO how to do this section
	// Fuck it, implement it on the MCU when its stable. Try bitbanging it from here first.
	
	uint32_t entryCode = 0x4D434850;   // MCHP in ascii.
                                       // Data is normally sent LSB first,
                                       // so we need to bit reverse this 
	unsigned tempInterface = a->interface;	// Save, this might come in handy in JTAG too.
	uint32_t counter = 0;
	entryCode = neofoxx_bitReversal(entryCode);

	neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // Reset, immediate
	neofoxx_setPins(a, -1, PIN_TMS, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // TMS LOW, immediate
	neofoxx_setPins(a, -1, PIN_TCK, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // TCK LOW, immediate
	mdelay(10);
	neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // No reset, immediate
	mdelay(10);
	neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // Reset, immediate
	mdelay(10);

	// Send the 32 bits here.
	for (counter=0; counter<32; counter++){
		neofoxx_setPins(a, -1, PIN_TMS, PIN_DIR_OUTPUT, 
						(entryCode & (0x01<<counter))?PIN_VAL_HIGH:PIN_VAL_LOW, 1); // TMS [val], immediate
		neofoxx_setPins(a, -1, PIN_TCK, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // TCK HIGH, immediate
		neofoxx_setPins(a, -1, PIN_TCK, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // TCK LOW, immediate
	}
	
	mdelay(5);
	neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // No reset, immediate
}

/*
 * Put device to serial execution mode.
 */
static void serial_execution(neofoxx_adapter_t *a)
{
    uint32_t counter = 20;
    uint32_t counterPre = 0;

    if (a->serial_execution_mode){
        return;
	}
    a->serial_execution_mode = 1;

    /* Enter serial execution. */
    if (debug_level > 0)
        fprintf(stderr, "%s: enter serial execution\n", a->name);

    /* Send command. */
    neofoxx_sendCommand(a, TAP_SW_MTAP, 0);  // Command, not immediate
    /* Reset TAP */
    neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);    // Reset TAP, immediate
    /* Send command. */
    neofoxx_sendCommand(a, MTAP_COMMAND, 0);
    /* Xfer data. */
    uint64_t status = neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_STATUS, 1, 1);
    if(!(status & MCHP_STATUS_CPS)){
        fprintf(stderr, "CPS bit is SET, please erase MCU first. Status: 0x%08x\n", (uint32_t)status);
        exit(-1);
    }

    do{

        if (INTERFACE_ICSP == a->interface){
            neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_ASSERT_RST, 0, 1);    // Data, don't read, immediate
        }
        if (INTERFACE_JTAG == a->interface || INTERFACE_DEFAULT == a->interface){
			neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // Reset, immediate
            neofoxx_flush_output(a);
        }

        //neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);    // Reset TAP, immediate
        /* Switch to ETAP */
        neofoxx_sendCommand(a, TAP_SW_ETAP, 1);   // Send command, immediate
        /* Reset TAP */
        neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);    // Reset TAP, immediate
        /* Put CPU in Serial Exec Mode */
        neofoxx_sendCommand(a, ETAP_EJTAGBOOT, 1); // Send command, immediate
        
        if (INTERFACE_JTAG == a->interface || INTERFACE_DEFAULT == a->interface){
			neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // No reset, immediate
            neofoxx_flush_output(a);
        }
        else{
            /* Else ICSP */
            /* Send command. */
            neofoxx_sendCommand(a, TAP_SW_MTAP, 1);
            /* Reset TAP */
            neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);    // Reset TAP, immediate
            /* Send command. */
            neofoxx_sendCommand(a, MTAP_COMMAND, 1);
            /* Send command. */
            neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_DEASSERT_RST, 0, 1);
            if (FAMILY_MX1 == a->adapter.family_name_short
                || FAMILY_MX3 == a->adapter.family_name_short){
                /* Send command, only for PIC32MX */
                neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_FLASH_ENABLE, 0, 1);
            }
            /* Switch to ETAP */
            neofoxx_sendCommand(a, TAP_SW_ETAP, 1);
            neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);    // Reset TAP, immediate
        }
    
        /* What is the value of ECR, after trying to connect */
        mdelay(10);
        neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);
        neofoxx_sendCommand(a, TAP_SW_ETAP, 1);
        neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);
        neofoxx_sendCommand(a, ETAP_CONTROL, 1);

        // At least on the MK chip, the first read is negative. Read again, and it's ok.
        counterPre = 11;
        do{
            status = neofoxx_xferData(a, 32, (CONTROL_PRACC | CONTROL_PROBEN | CONTROL_PROBTRAP), 1, 1);    // Send data, readflag, immediate, don't care
        }while(!(status & CONTROL_PROBEN) && counterPre-- > 1); 

//        if (!(status & CONTROL_PRACC)){   
        if (!(status & CONTROL_PROBEN)){   
            fprintf(stderr, "Failed to enter serial execution. Status was %08x\n", (uint32_t)status);
            if (INTERFACE_JTAG == a->interface || INTERFACE_DEFAULT == a->interface){  
                /* For these chips, ICSP & JTAG pins are (sometimes) shared. Can do a trick */
                if (FAMILY_MX1 == a->adapter.family_name_short
                    || FAMILY_MX3 == a->adapter.family_name_short)
                {
				    fprintf(stderr, "In JTAG mode, trying to recover automatically\n");
				    // MCLR is currently 1.
				    // We need to go 0, enter ICSP, go 1, go 0, repeat this do-while loop.
				    // NOTE!! This needs to be done on the TMS LINE! TDI should be held low, probably.
					neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // Reset, immediate
               		neofoxx_flush_output(a);
				    mdelay(5);
				    
				    neofoxx_setMode(a, SET_MODE_ICSP_SYNC, 1);
				    mdelay(5);

					neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // No reset, immediate
               		neofoxx_flush_output(a);
				    mdelay(5);
                }
                else{
                    fprintf(stderr, "In JTAG mode, only recovery is through a power-cycle, or reset via ICSP. Quitting.\n");
                    exit(-1);
                }

				// Reset will be asserted in the beginning of the loop again.
				mdelay(100);			
            }
        }
        
    }while(!(status & CONTROL_PROBEN) && counter-- > 1);    // Repeat, until we sucessefully enter serial execution. Extend if necessary, to more than PROBEN!
    
    if (counter == 0){
        fprintf(stderr, "Couldn't enter serial execution, quitting\n");
    }

    mdelay(10);
}

static unsigned get_pe_response(neofoxx_adapter_t *a)
{
    unsigned ctl, response;

    // Select Control Register
    /* Send command. */
    neofoxx_sendCommand(a, ETAP_CONTROL, 1);	// Command, immediate

    // Wait until CPU is ready
    // Check if Processor Access bit (bit 18) is set
    do {
        ctl = neofoxx_xferData(a, 32, (CONTROL_PRACC | CONTROL_PROBEN 
                | CONTROL_PROBTRAP | CONTROL_EJTAGBRK), 1, 1);    // Send data, readflag, immediate don't care
    } while (! (ctl & CONTROL_PRACC));

    // Select Data Register
    // Send the instruction
    /* Send command. */
    neofoxx_sendCommand(a, ETAP_DATA, 1);

    /* Get data. */
    response = neofoxx_xferData(a, 32, 0, 1, 1);  // Send 32 zeroes, read response, immediate don't care

    // Tell CPU to execute NOP instruction
    /* Send command. */
    neofoxx_sendCommand(a, ETAP_CONTROL, 1);
    /* Send data. */
    neofoxx_xferData(a, 32, (CONTROL_PROBEN | CONTROL_PROBTRAP), 0, 1);

    if (debug_level > 1)
        fprintf(stderr, "%s: get PE response %08x\n", a->name, response);
    return response;
}

/*
 * Read a word from memory (without PE).
 */
static unsigned neofoxx_read_word(adapter_t *adapter, unsigned addr)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
    unsigned addr_lo = addr & 0xFFFF;
    unsigned addr_hi = (addr >> 16) & 0xFFFF;
    unsigned word = 0;

	uint32_t workaround = 1;

    /* Workaround for PIC32MM. If not in serial execution mode yet,
     * read word twice after enering serial execution,
     * as first word will be garbage. */
    unsigned times = (a->serial_execution_mode)?0:1;

    serial_execution(a);
	if (WAY_OLD == a->way || workaround){
		if (workaround){
			fprintf(stderr, "WORKAROUND in %s active\n", __func__); 
		}

		do{
		    if (FAMILY_MX1 == a->adapter.family_name_short
		                    || FAMILY_MX3 == a->adapter.family_name_short
		                    || FAMILY_MK == a->adapter.family_name_short
		                    || FAMILY_MZ == a->adapter.family_name_short)
		    {
		        //fprintf(stderr, "%s: read word from %08x\n", a->name, addr);

		        neofoxx_xferInstruction_PC_slow(a, 0x3c13ff20);            // lui s3, FASTDATA_REG_ADDR(31:16)
		        neofoxx_xferInstruction_PC_slow(a, 0x3c080000 | addr_hi);  // lui t0, addr_hi
		        neofoxx_xferInstruction_PC_slow(a, 0x35080000 | addr_lo);  // ori t0, addr_lo
		        neofoxx_xferInstruction_PC_slow(a, 0x8d090000);            // lw t1, 0(t0)
		        neofoxx_xferInstruction_PC_slow(a, 0xae690000);            // sw t1, 0(s3)
		        neofoxx_xferInstruction_PC_slow(a, 0);                     // NOP - necessary!

		        /* Send command. */
		        neofoxx_sendCommand(a, ETAP_FASTDATA, 1);
		        /* Get fastdata. */    
		        /* Send zeroes, read response, immediate don't care. Shift by 1 to get rid of PrACC */
		        word = neofoxx_xferFastData_PC_slow(a, 0, 1) >> 1;
		    }
		    else{
		        /* Else PIC32MM */
				neofoxx_xferInstruction_PC_slow(a, 0xFF2041B3);					// lui s3, FAST_DATA_REG(32:16). Set address of fastdata register
				neofoxx_xferInstruction_PC_slow(a, 0x000041A8 | (addr_hi<<16));	// lui t0, DATA_ADDRESS(31:16)
				neofoxx_xferInstruction_PC_slow(a, 0x00005108 | (addr_lo<<16));	// ori t0, DATA_ADDRESS(15:0)
		 		neofoxx_xferInstruction_PC_slow(a, 0x0000FD28);					// lw t1, 0(t0) - read data
		 		neofoxx_xferInstruction_PC_slow(a, 0x0000F933);					// sw t1, 0(s3) - store data to fast register
				neofoxx_xferInstruction_PC_slow(a, 0x0c000c00);					// Nop, 2x
				neofoxx_xferInstruction_PC_slow(a, 0x0c000c00);					// Nop, 2x, again. Without this (4x nop), you will get garbage after a few bytes!
																		// Extra Nops make it even worse, always get 0s
		 		
				/* Send command. */
				neofoxx_sendCommand(a, ETAP_FASTDATA, 1);

				/* Get fastdata. */
				word = neofoxx_xferFastData_PC_slow(a, 0, 1) >> 1; // Send zeroes, read response, immediate don't care. Shift by 1 to get rid of PrACC
		    }

		}while(times-- > 0);

		if (debug_level > 0)
		    fprintf(stderr, "%s: read word at %08x -> %08x\n", a->name, addr, word);
		return word;
	}
	else{
		fprintf(stderr, "WAY_NEW not implemented yet in %s\n", __func__);
		return 0;
	}
}

/*
 * Read a memory block.
 */
static void neofoxx_read_data(adapter_t *adapter,
    unsigned addr, unsigned nwords, unsigned *data)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
    unsigned words_read, i;

	uint32_t workaround = 1;

    if (WAY_OLD == a->way || workaround){
		if (workaround){
			fprintf(stderr, "WORKAROUND in %s active\n", __func__); 
		}
		if (! a->use_executive) {
		    /* Without PE. */
		    for (; nwords > 0; nwords--) {
		        *data++ = neofoxx_read_word(adapter, addr);
		        addr += 4;
		    }
		    return;
		}

		/* Use PE to read memory. */
		for (words_read = 0; words_read < nwords; words_read += 32) {

		    neofoxx_sendCommand(a, ETAP_FASTDATA, 1);
		    neofoxx_xferFastData_PC_slow(a, PE_READ << 16 | 32, 0);       /* Read 32 words */  // Data, don't read, immediate
		    neofoxx_xferFastData_PC_slow(a, addr, 0);                     /* Address */        // Data, don't read, immediate

		    unsigned response = get_pe_response(a);     /* Get response */
		    if (response != PE_READ << 16) {
		        fprintf(stderr, "%s: bad READ response = %08x, expected %08x\n",
		            a->name, response, PE_READ << 16);
		        exit(-1);
		    }
		    for (i=0; i<32; i++) {
		        *data++ = get_pe_response(a);           /* Get data */
		    }
		    addr += 32*4;
		}
	}
	else{
		fprintf(stderr, "WAY_NEW not implemented yet in %s\n", __func__);
	}
}

/*
 * Download programming executive (PE).
 */
static void neofoxx_load_executive(adapter_t *adapter,
    const unsigned *pe, unsigned nwords, unsigned pe_version)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;

	uint32_t workaround = 1;

    a->use_executive = 1;
    serial_execution(a);

    if (debug_level > 0){
        fprintf(stderr, "%s: download PE loader\n", a->name);
	}

    if (WAY_OLD == a->way || workaround){
		if (workaround){
			fprintf(stderr, "WORKAROUND in %s active\n", __func__); 
		}
		if (a->adapter.family_name_short == FAMILY_MX1
		    || a->adapter.family_name_short == FAMILY_MX3
		    || a->adapter.family_name_short == FAMILY_MK
		    || a->adapter.family_name_short == FAMILY_MZ)
		{
		    /* Step 1. */
		    neofoxx_xferInstruction_PC_slow(a, 0x3c04bf88);    // lui a0, 0xbf88
		    neofoxx_xferInstruction_PC_slow(a, 0x34842000);    // ori a0, 0x2000 - address of BMXCON
		    neofoxx_xferInstruction_PC_slow(a, 0x3c05001f);    // lui a1, 0x1f
		    neofoxx_xferInstruction_PC_slow(a, 0x34a50040);    // ori a1, 0x40   - a1 has 001f0040
		    neofoxx_xferInstruction_PC_slow(a, 0xac850000);    // sw  a1, 0(a0)  - BMXCON initialized

		    /* Step 2. */
		    neofoxx_xferInstruction_PC_slow(a, 0x34050800);    // li  a1, 0x800  - a1 has 00000800
		    neofoxx_xferInstruction_PC_slow(a, 0xac850010);    // sw  a1, 16(a0) - BMXDKPBA initialized

		    /* Step 3. */
		    neofoxx_xferInstruction_PC_slow(a, 0x8c850040);    // lw  a1, 64(a0) - load BMXDMSZ
		    neofoxx_xferInstruction_PC_slow(a, 0xac850020);    // sw  a1, 32(a0) - BMXDUDBA initialized
		    neofoxx_xferInstruction_PC_slow(a, 0xac850030);    // sw  a1, 48(a0) - BMXDUPBA initialized

		    /* Step 4. */
		    neofoxx_xferInstruction_PC_slow(a, 0x3c04a000);    // lui a0, 0xa000
		    neofoxx_xferInstruction_PC_slow(a, 0x34840800);    // ori a0, 0x800  - a0 has a0000800

		    /* Download the PE loader. */
		    int i;
		    for (i=0; i<PIC32_PE_LOADER_LEN; i+=2) {
		        /* Step 5. */
		        unsigned opcode1 = 0x3c060000 | pic32_pe_loader[i];
		        unsigned opcode2 = 0x34c60000 | pic32_pe_loader[i+1];

		        neofoxx_xferInstruction_PC_slow(a, opcode1);       // lui a2, PE_loader_hi++
		        neofoxx_xferInstruction_PC_slow(a, opcode2);       // ori a2, PE_loader_lo++
		        neofoxx_xferInstruction_PC_slow(a, 0xac860000);    // sw  a2, 0(a0)
		        neofoxx_xferInstruction_PC_slow(a, 0x24840004);    // addiu a0, 4
		    }

		    /* Jump to PE loader (step 6). */
		    neofoxx_xferInstruction_PC_slow(a, 0x3c19a000);    // lui t9, 0xa000
		    neofoxx_xferInstruction_PC_slow(a, 0x37390800);    // ori t9, 0x800  - t9 has a0000800
		    neofoxx_xferInstruction_PC_slow(a, 0x03200008);    // jr  t9
		    neofoxx_xferInstruction_PC_slow(a, 0x00000000);    // nop

		    /* Switch from serial to fast execution mode. */
		    neofoxx_sendCommand(a, TAP_SW_ETAP, 1);
		    /* TMS 1-1-1-1-1-0 */
		    neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);

		    /* Send parameters for the loader (step 7-A).
		     * PE_ADDRESS = 0xA000_0900,
		     * PE_SIZE */
		    /* Send command. */
		    neofoxx_sendCommand(a, ETAP_FASTDATA, 1);
		    neofoxx_xferFastData_PC_slow(a, 0xa0000900, 0);    /* Don't read, immediate */
		    neofoxx_xferFastData_PC_slow(a, nwords, 0);        /* Don't read, immediate */

		    /* Download the PE itself (step 7-B). */
		    if (debug_level > 0)
		        fprintf(stderr, "%s: download PE\n", a->name);
		    for (i=0; i<nwords; i++) {
		        neofoxx_xferFastData_PC_slow(a, *pe++, 0);     /* Don't read, not immediate */
		    }
		    neofoxx_flush_output(a);
		    mdelay(10);

		    /* Download the PE instructions. */
		    /* Step 8 - jump to PE. */
		    neofoxx_xferFastData_PC_slow(a, 0, 0);             /* Don't read, immediate */                  
		    neofoxx_xferFastData_PC_slow(a, 0xDEAD0000, 0);    /* Don't read, immediate */
		    mdelay(10);
		    neofoxx_xferFastData_PC_slow(a, PE_EXEC_VERSION << 16, 0); /* Don't read, immediate */
		}
		else{
		    /* Else MM family */
		    // Step 1. Setup PIC32MM RAM address for the PE 
			neofoxx_xferInstruction_PC_slow(a, 0xa00041a4);    // lui a0, 0xa000
			neofoxx_xferInstruction_PC_slow(a, 0x02005084);    // ori a0, a0, 0x200 A total of 0xa000_0200

			// Step 2. Load the PE_loader.
			int i;
			for (i=0; i<PIC32_PEMM_LOADER_LEN; i+=2) {
				/* Step 5. */
				unsigned opcode1 = 0x41A6 | (pic32_pemm_loader[i] << 16);
				unsigned opcode2 = 0x50C6 | (pic32_pemm_loader[i+1] << 16);

				neofoxx_xferInstruction_PC_slow(a, opcode1);       // lui a2, PE_loader_hi++
				neofoxx_xferInstruction_PC_slow(a, opcode2);       // ori a2, a2, PE_loader_lo++
				neofoxx_xferInstruction_PC_slow(a, 0x6E42EB40);    // sw  a2, 0(a0); addiu a0, a0, 4;
			}

			// Step 3. Jump to the PE_Loader
			neofoxx_xferInstruction_PC_slow(a, 0xA00041B9);       // lui t9, 0xa000
			neofoxx_xferInstruction_PC_slow(a, 0x02015339);       // ori t9, t9, 0x0800. Same address as at beginning +1.
			neofoxx_xferInstruction_PC_slow(a, 0x0C004599);       // jr t9; nop;

			/* These nops here are MANDATORY. And exactly this many.
			 * Less it doesn't work, more it doesn't work. */
			neofoxx_xferInstruction_PC_slow(a, 0x0C000C00);
			neofoxx_xferInstruction_PC_slow(a, 0x0C000C00);

			// Step 4. Load the PE using the PE_loader
			// Switch to ETAP
			neofoxx_sendCommand(a, TAP_SW_ETAP, 1);
			/* TMS 1-1-1-1-1-0 */
			neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);
			// Set to FASTDATA
			neofoxx_sendCommand(a, ETAP_FASTDATA, 1);

			// Send PE_ADDRESS, Address os PE program block from PE Hex file
			neofoxx_xferFastData_PC_slow(a, 0xA0000300, 0);	// Taken from the .hex file.
			
			// Send PE_SIZE, number as 32-bit words of the program block from the PE Hex file
			neofoxx_xferFastData_PC_slow(a, nwords, 0);        // Data, don't read

			if (debug_level > 0){
				fprintf(stderr, "%s: download PE, nwords = %d\n", a->name, nwords);
				//mdelay(3000);		
			}
			for (i=0; i<nwords; i++) {
				neofoxx_xferFastData_PC_slow(a, *pe++, 0);     // Data, don't read
			}
			neofoxx_flush_output(a);
			mdelay(10);

			// Step 5, Jump to the PE.
			neofoxx_xferFastData_PC_slow(a, 0x00000000, 0);
			neofoxx_xferFastData_PC_slow(a, 0xDEAD0000, 0);

			// Done.
			// Get PE version?
			mdelay(10);
			neofoxx_xferFastData_PC_slow(a, PE_EXEC_VERSION << 16, 0);     // Data, don't read
		}
		
		unsigned version = get_pe_response(a);
		if (version != (PE_EXEC_VERSION << 16 | pe_version)) {
		    fprintf(stderr, "%s: bad PE version = %08x, expected %08x\n",
		        a->name, version, PE_EXEC_VERSION << 16 | pe_version);
		    exit(-1);
		}
		if (debug_level > 0){
		    fprintf(stderr, "%s: PE version = %04x\n", a->name, version & 0xffff);
		}
	}	
	else{
		fprintf(stderr, "WAY_NEW not implemented yet in %s\n", __func__);
	}
}

/*
 * Erase all flash memory.
 */
static void neofoxx_erase_chip(adapter_t *adapter)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
    unsigned status = 0;

    /* Switch to MTAP */
    neofoxx_sendCommand(a, TAP_SW_MTAP, 1);
    /* Reset TAP */
    neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);
    /* Send command. */
    neofoxx_sendCommand(a, MTAP_COMMAND, 1);
    /* Xfer data. */
    neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_ERASE, 0, 1);
    neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_DEASSERT_RST, 0, 1);

    // https://www.microchip.com/forums/m627418.aspx .......
    if (INTERFACE_JTAG == a->interface || INTERFACE_DEFAULT == a->interface){
		neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // No reset, immediate
    }

    do{
        status = neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_STATUS, 1, 1);  // Send data, read response, immediate don't care
        //fprintf(stderr, "Status is 0x%08x ... \n", statuss);
        if (!(status & MCHP_STATUS_CFGRDY) || (status & MCHP_STATUS_FCBUSY)){
            printf(".");
            mdelay(10);
        }
    }while(!(status & MCHP_STATUS_CFGRDY) || (status & MCHP_STATUS_FCBUSY));

    neofoxx_setMode(a, SET_MODE_TAP_RESET, 1); 
    mdelay(25);
}

/*
 * Write a word to flash memory.
 */
static void neofoxx_program_word(adapter_t *adapter,
    unsigned addr, unsigned word)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
	uint32_t workaround = 1;

    if (FAMILY_MM == a->adapter.family_name_short){
        fprintf(stderr, "Program word is not available on MM family. Quitting\n");
    }

    if (debug_level > 0)
        fprintf(stderr, "%s: program word at %08x: %08x\n", a->name, addr, word);
    if (! a->use_executive) {
        /* Without PE. */
        fprintf(stderr, "%s: slow flash write not implemented yet.\n", a->name);
        exit(-1);
    }

	if (WAY_OLD == a->way || workaround){
		if (workaround){
			fprintf(stderr, "WORKAROUND in %s active\n", __func__); 
		}
		/* Use PE to write flash memory. */
		/* Send command. */
		neofoxx_sendCommand(a, ETAP_FASTDATA, 1);

		neofoxx_xferFastData_PC_slow(a, PE_WORD_PROGRAM << 16 | 2, 0); // Data, don't read, immediate
		neofoxx_xferFastData_PC_slow(a, addr, 0);  /* Send address. */ // Data, don't read, immediate
		neofoxx_xferFastData_PC_slow(a, word, 0);  /* Send word. */    // Data, don't read, immediate

		unsigned response = get_pe_response(a);
		if (response != (PE_WORD_PROGRAM << 16)) {
		    fprintf(stderr, "%s: failed to program word %08x at %08x, reply = %08x\n",
		        a->name, word, addr, response);
		    exit(-1);
		}
	}
	else{
		fprintf(stderr, "WAY_NEW not implemented yet in %s\n", __func__);
	}
}

static void neofoxx_program_double_word(adapter_t *adapter, unsigned addr, unsigned word0, unsigned word1){
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
	uint32_t workaround = 1;
    
    if (FAMILY_MM != a->adapter.family_name_short){
        fprintf(stderr, "Program double word is only available on MM family. Quitting\n");
    }

	if (debug_level > 0){
		fprintf(stderr, "%s: program double word at 0x%08x: 0x%08x 0x%08x\n", a->name, addr, word0, word1);
	}
	if (! a->use_executive) {
        /* Without PE. */
        fprintf(stderr, "%s: slow flash write not implemented yet.\n", a->name);
        exit(-1);
    }

	if (WAY_OLD == a->way || workaround){
		if (workaround){
			fprintf(stderr, "WORKAROUND in %s active\n", __func__); 
		}
		/* Use PE to write flash memory. */
		/* Send command. */
		neofoxx_sendCommand(a, ETAP_FASTDATA, 1);
		// TODO - RECHECK the | 2!!!
		neofoxx_xferFastData_PC_slow(a, PE_DOUBLE_WORD_PGRM << 16 | 2, 0); // Data, don't read, immediate
		neofoxx_xferFastData_PC_slow(a, addr, 0);  	/* Send address. */ // Data, don't read, immediate
		neofoxx_xferFastData_PC_slow(a, word0, 0);  	/* Send 1st word. */    // Data, don't read, immediate, was not before
		neofoxx_xferFastData_PC_slow(a, word1, 0);  	/* Send 2nd word. */    // Data, don't read, immediate, was not before
	 
		unsigned response = get_pe_response(a);
		if (response != (PE_DOUBLE_WORD_PGRM << 16)) {
		    fprintf(stderr, "%s: failed to program double words 0x%08x 0x%08x at 0x%08x, reply = %08x\n",
		        a->name, word0, word1, addr, response);
		    exit(-1);
		}
	}	
	else{
		fprintf(stderr, "WAY_NEW not implemented yet in %s\n", __func__);
	}

}

static void neofoxx_program_quad_word(adapter_t *adapter, unsigned addr, 
            unsigned word0, unsigned word1, unsigned word2, unsigned word3){
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
	uint32_t workaround = 1;

    if (FAMILY_MK != a->adapter.family_name_short
        && FAMILY_MZ != a->adapter.family_name_short){
        fprintf(stderr, "Program quad word is only available on MK and MZ families. Quitting\n");
    }

	if (debug_level > 0){
		fprintf(stderr, "%s: program quad word at 0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
                         a->name, addr, word0, word1, word2, word3);
	}
	if (! a->use_executive) {
        /* Without PE. */
        fprintf(stderr, "%s: slow flash write not implemented yet.\n", a->name);
        exit(-1);
    }

	if (WAY_OLD == a->way || workaround){
		if (workaround){
			fprintf(stderr, "WORKAROUND in %s active\n", __func__); 
		}

		/* Use PE to write flash memory. */
		/* Send command. */
		neofoxx_sendCommand(a, ETAP_FASTDATA, 1);

		neofoxx_xferFastData_PC_slow(a, PE_QUAD_WORD_PGRM << 16, 0); // Data, don't read, immediate
		neofoxx_xferFastData_PC_slow(a, addr, 0);  	/* Send address. */ // Data, don't read, immediate
		neofoxx_xferFastData_PC_slow(a, word0, 0);  	/* Send 1st word. */    // Data, don't read, immediate, was not before
		neofoxx_xferFastData_PC_slow(a, word1, 0);  	/* Send 2nd word. */    // Data, don't read, immediate, was not before
		neofoxx_xferFastData_PC_slow(a, word2, 0);  	/* Send 1st word. */    // Data, don't read, immediate, was not before
		neofoxx_xferFastData_PC_slow(a, word3, 0);  	/* Send 2nd word. */    // Data, don't read, immediate, was not before

		unsigned response = get_pe_response(a);
		if (response != (PE_QUAD_WORD_PGRM << 16)) {
			fprintf(stderr, "%s: failed to program quad words 0x%08x 0x%08x 0x%08x 0x%08x at 0x%08x, reply = %08x\n",
				a->name, word0, word1, word2, word3, addr, response);
			exit(-1);
		}
	}
	else{
		fprintf(stderr, "WAY_NEW not implemented yet in %s\n", __func__);
	}
	
}

/*
 * Flash write row of memory.
 */
static void neofoxx_program_row(adapter_t *adapter, unsigned addr,
    unsigned *data, unsigned words_per_row)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
    int i;
	uint32_t workaround = 0;

    if (debug_level > 0)
        fprintf(stderr, "%s: row program %u words at %08x\n",
            a->name, words_per_row, addr);
    if (! a->use_executive) {
        /* Without PE. */
        fprintf(stderr, "%s: slow flash write not implemented yet.\n", a->name);
        exit(-1);
    }

	if (WAY_OLD == a->way || workaround){
		if (workaround){
			fprintf(stderr, "WORKAROUND in %s active\n", __func__); 
		}

		/* Use PE to write flash memory. */
		/* Send command. */
		neofoxx_sendCommand(a, ETAP_FASTDATA, 1); 

		neofoxx_xferFastData_PC_slow(a, PE_ROW_PROGRAM << 16 | words_per_row, 0);  // Data, don't read, immediate
		neofoxx_xferFastData_PC_slow(a, addr, 0);  /* Send address. */             // Data, don't read, immediate

		/* Download data. */
		for (i = 0; i < words_per_row; i++) {
		    if ((i & 7) == 0)
		        neofoxx_flush_output(a);
		    neofoxx_xferFastData_PC_slow(a, *data++, 0);   /* Send word. Don't read, not immediate */
		}
		neofoxx_flush_output(a);

		unsigned response = get_pe_response(a);
		if (response != (PE_ROW_PROGRAM << 16)) {
		    fprintf(stderr, "%s: failed to program row at %08x, reply = %08x\n",
		        a->name, addr, response);
		    exit(-1);
		}
	}
	else{
		fprintf(stderr, "WAY_NEW not implemented yet in %s\n", __func__);
		/* Use PE to write flash memory. */
		/* Send command. */
		neofoxx_sendCommand(a, ETAP_FASTDATA, 1); 
		neofoxx_xferFastData_MCU_fast(a, PE_ROW_PROGRAM << 16 | words_per_row, 0);	
		neofoxx_xferFastData_MCU_fast(a, addr, 0);  // Send address
		
		neofoxx_flush_output(a);

		// Download data
		for (i = 0; i < words_per_row; i++) {
		    if ((i & 3) == 0){
		        neofoxx_flush_output(a);	// Send data every x bytes.
			}
			///fprintf(stderr, "Sending %08x\n", *data);
		    neofoxx_xferFastData_MCU_fast(a, *data++, 0);
		}

		neofoxx_flush_output(a);	// At this point, all data will be sent.
		///mdelay(1000);
		///fprintf(stderr, "getting PE response\n");
		///mdelay(7500);


		unsigned response = get_pe_response(a);
		if (response != (PE_ROW_PROGRAM << 16)) {
		    fprintf(stderr, "%s: failed to program row at %08x, reply = %08x\n",
		        a->name, addr, response);
		    exit(-1);
		}

	}
}

/*
 * Verify a block of memory.
 */
static void neofoxx_verify_data(adapter_t *adapter,
    unsigned addr, unsigned nwords, unsigned *data)
{
    neofoxx_adapter_t *a = (neofoxx_adapter_t*) adapter;
    unsigned data_crc, flash_crc;
	uint32_t workaround = 1;

    //fprintf(stderr, "%s: verify %d words at %08x\n", a->name, nwords, addr);
    if (! a->use_executive) {
        /* Without PE. */
        fprintf(stderr, "%s: slow verify not implemented yet.\n", a->name);
        exit(-1);
    }

	if (WAY_OLD == a->way || workaround){
		if (workaround){
			fprintf(stderr, "WORKAROUND in %s active\n", __func__); 
		}
    	/* Use PE to get CRC of flash memory. */
		/* Send command. */
		neofoxx_sendCommand(a, ETAP_FASTDATA, 1);

		neofoxx_xferFastData_PC_slow(a, PE_GET_CRC << 16, 0);  // Data, don't read, immediate
		 /* Send address. */
		neofoxx_xferFastData_PC_slow(a, addr, 0);              // Data, don't read, immediate
		/* Send length. */
		neofoxx_xferFastData_PC_slow(a, nwords * 4, 0);        // Data, don't read, immediate

		unsigned response = get_pe_response(a);
		if (response != (PE_GET_CRC << 16)) {
		    fprintf(stderr, "%s: failed to verify %d words at %08x, reply = %08x\n",
		        a->name, nwords, addr, response);
		    exit(-1);
		}
		flash_crc = get_pe_response(a) & 0xffff;
		data_crc = calculate_crc(0xffff, (unsigned char*) data, nwords * 4);
		if (flash_crc != data_crc) {
		    fprintf(stderr, "%s: checksum failed at %08x: sum=%04x, expected=%04x\n",
		        a->name, addr, flash_crc, data_crc);
		    //exit(-1);
		}

	}
	else{
		fprintf(stderr, "WAY_NEW not implemented yet in %s\n", __func__);
	}
}

/*
 * Initialize adapter F2232.
 * Return a pointer to a data structure, allocated dynamically.
 * When adapter not found, return 0.
 * Parameters vid, pid and serial are not used.
 */
adapter_t *adapter_open_neofoxx(const char *port, int baudrate, int interface, int speed)
{
    neofoxx_adapter_t *a;

    a = calloc(1, sizeof(*a));
    if (! a) {
        fprintf(stderr, "adapter_open_neofoxx: out of memory\n");
        return 0;
    }
    //a->context = NULL;

	
	if (serial_open(port, baudrate) < 0) {
        /* Failed to open serial port */
        fprintf(stderr, "Unable to open serial port %s\n", port);
        serial_close();
        free(a);
        return 0;
    }

	a->capabilities_mcu[0] = 0;
	neofoxx_get_capabilities(a);	// Populate struct with info
	if (a->capabilities_mcu[0] == '\0'){
		fprintf(stderr, "Unable to get information about adapter\n");
		serial_close();
        free(a);
        return 0;
	}

  
	fprintf(stderr, "Found adapter %s\n:", a->name);
	// TODO printout capabilities

	a->interface = interface;	// TODO do different later or something

	// Have to set, otherwise things don't work :p
	if (a->interface == INTERFACE_DEFAULT || a->interface == INTERFACE_JTAG){
		neofoxx_setProgMode(a, PROG_MODE_JTAG, 1); // Setup for tristate output, immediate
	}
	else if (a->interface == INTERFACE_ICSP){
		neofoxx_setProgMode(a, PROG_MODE_ICSP, 1); // Setup for tristate output, immediate
	}
	else{
		fprintf(stderr, "Unsupported itnerface specified\n");
		serial_close();
        free(a);
        return 0;
	}


    /* By default, use 500 kHz speed, unless specified */
	
	if (0 != speed){
		fprintf(stderr, "Using NEW WAY of communicating\n");
		a->way = WAY_NEW;
	}
	else{
		fprintf(stderr, "Using old way of communicating\n");
		a->way = WAY_OLD;
	}
    neofoxx_speed(a, speed);
	// TODO prinout what's the actual frequency



    /* Activate LED. */
    // Maybe later, or LCD.

    unsigned idcode = 0;
    uint32_t counter = 11;
    do{
        if (INTERFACE_ICSP == a->interface){
            neofoxx_enter_icsp(a);
        }

        /* Delay required for ICSP */
        mdelay(5);     

        /* Reset the JTAG TAP controller: TMS 1-1-1-1-1-0.
         * After reset, the IDCODE register is always selected.
         * Read out 32 bits of data. */
        fprintf(stderr, "aaaa\n");

        neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);
fprintf(stderr, "bbbb\n");
        neofoxx_sendCommand(a, TAP_SW_MTAP, 1);
fprintf(stderr, "cccc\n");
        neofoxx_setMode(a, SET_MODE_TAP_RESET, 1);
fprintf(stderr, "dddd\n");
        neofoxx_sendCommand(a, MTAP_IDCODE, 1);
fprintf(stderr, "eeee\n");

        idcode = neofoxx_xferData(a, 32, 0, 1, 1);
fprintf(stderr, "ffff\n");
        if ((idcode & 0xfff) != 0x053) {
            /* Microchip vendor ID is expected. */
            if (debug_level > 0 || (idcode != 0 && idcode != 0xffffffff))
                fprintf(stderr, "%s: incompatible CPU detected, IDCODE=%08x\n",
                    a->name, idcode);
            fprintf(stderr, "IDCODE not valid, retrying\n");
        }
    }while((idcode & 0xfff) != 0x053 && counter-- > 1);
    if(counter == 0){
		neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // No reset, immediate
        fprintf(stderr, "Couldn't read IDCODE, exiting\n");
		// Instead of goto failed;        
		serial_close();
        free(a);
        return 0;
    }
    printf("      IDCODE=%08x\n", idcode);

    /* Activate /SYSRST and LED. Only done in JTAG mode */
    if (INTERFACE_JTAG == a->interface || INTERFACE_DEFAULT == a->interface)
    {
		neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_LOW, 1); // Reset, immediate

        // So, the MM family's JTAG doesn't work in RESET...      
        // Works like this for all the others as well.  
        mdelay(10);
		neofoxx_setPins(a, -1, PIN_MCLR, PIN_DIR_OUTPUT, PIN_VAL_HIGH, 1); // No reset, immediate

    } 
    mdelay(10);

    /* Check status. */
    /* Send command. */
    neofoxx_sendCommand(a, TAP_SW_MTAP, 1);
    /* Send command. */
    neofoxx_sendCommand(a, MTAP_COMMAND, 1);
    /* Xfer data. */
    neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_FLASH_ENABLE, 0, 1);
    /* Xfer data. */
    unsigned status = neofoxx_xferData(a, MTAP_COMMAND_DR_NBITS, MCHP_STATUS, 1, 1);
    
    if (debug_level > 0)
        fprintf(stderr, "%s: status %04x\n", a->name, status);
    if ((status & (MCHP_STATUS_CFGRDY | MCHP_STATUS_FCBUSY)) != (MCHP_STATUS_CFGRDY)) {
        fprintf(stderr, "%s: invalid status = %04x\n", a->name, status);
        neofoxx_setProgMode(a, PROG_MODE_TRISTATE, 1); // Setup for tristate output, immediate
        // Instead of goto failed;        
		serial_close();
        free(a);
        return 0;
    }
    printf("      Adapter: %s\n", a->name);

    a->adapter.block_override = 0;
    a->adapter.flags = (AD_PROBE | AD_ERASE | AD_READ | AD_WRITE);

    /* User functions. */
    a->adapter.close = neofoxx_close;
    a->adapter.get_idcode = neofoxx_get_idcode;
    a->adapter.load_executive = neofoxx_load_executive;
    a->adapter.read_word = neofoxx_read_word;
    a->adapter.read_data = neofoxx_read_data;
    a->adapter.verify_data = neofoxx_verify_data;
    a->adapter.erase_chip = neofoxx_erase_chip;
    a->adapter.program_word = neofoxx_program_word;
    a->adapter.program_row = neofoxx_program_row;
    a->adapter.program_double_word = neofoxx_program_double_word;
    a->adapter.program_quad_word = neofoxx_program_quad_word;
    return &a->adapter;
}
