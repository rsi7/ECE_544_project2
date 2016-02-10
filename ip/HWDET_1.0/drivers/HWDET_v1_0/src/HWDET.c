/**
*
* @file HWDET.c
*
* @author Rehan Iqbal (riqbal@pdx.edu)
* @copyright Portland State University, 2016
*
* This file implements the driver functions for the custom peripheral "HWDET". 
* This peripheral is a hardware pulse detection module written in Verilog. 
* It implements a simple edge-based counter to report 32-bit values
* for high interval length and low interval length.
*
* Major driver functions:
*
* 	o Initialize the peripheral into the correct mode
*	o Capture a frequency reading
*	o Convert actual ‘count’ measurements to a scaled output corresponding to intensity of the LED
*	o Adjust any scaling factors based on actual characterization measurements
*/

/****************************************************************************/
/***************************** Include Files ********************************/
/****************************************************************************/

// include the header file (HWDET.h)
// this in turn points to the low-level header file (HWDET_l.h),
// which directly reads & writes memory addresses

#include "HWDET.h"
#include "HWDET_selftest.c"

/****************************************************************************/
/************************** Constant Definitions ****************************/
/****************************************************************************/

// If these pre-processors are not defined at the top-level application,
// we can use these definitions instead.
// They will not override pre-existing definitions, though.

#ifndef CPU_CLOCK_FREQ_HZ
#define CPU_CLOCK_FREQ_HZ (100000000) 
#endif

#ifndef LED_SCALING_FACTOR
#define LED_SCALING_FACTOR (1)
#endif

/****************************************************************************/
/***************** Macros (Inline Functions) Definitions ********************/
/****************************************************************************/

// If these pre-processors are not defined at the top-level application,
// we can use these definitions instead.
// They will not override pre-existing definitions, though.

#ifndef MIN(a, b)
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#endif

#ifndef MAX(a, b)
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )
#endif

/****************************************************************************/
/************************** Variable Definitions ****************************/
/****************************************************************************/

// Need to provide the Base Address for the HWDET peripheral
// so that memory reads & writes can be accomplished

u32 HWDET_BaseAddress;

/****************************************************************************/
/************************** Driver Functions ********************************/
/****************************************************************************/

/****************** Initialization & Configuration ************************/
/**
* Initialize the HWDET peripheral driver
*
* Saves the Base address of the HWDET peripheral and runs the self-test
*
* @param	BaseAddr is the base address of the HWDET register set
*
* @return
* 			- XST_SUCCESS	Initialization was successful.
			- XST_FAILURE 	Initialization failed on memory read & write tests.
*
* @note		This function can hang if the peripheral was not created correctly
* @note		The Base Address of the HWDET peripheral will be in xparameters.h
*
*****************************************************************************/

int HWDET_initialize(u32 BaseAddr) {

	HWDET_BaseAddress = BaseAddr;
	return HWDET_Reg_SelfTest(HWDET_BaseAddress);
}

/******************** Get count for high / low interval ********************/	
/**
* Returns the value for the high / low count register in hw_detect.v
* This value corresponds to the length of time the PWM pulse was
* in the 'high' or 'low' state.  
* 
* This works through a simple read on the slv_reg0 / slv_reg_0 memory addresses,
* which is at (BaseAddress + 0) / (BaseAddress + 4) respectively.
*
* @param	Register to be read (valid inputs: HIGH, LOW)
*
* @return	Value of the high / low count register from hw_detect.v
* 			Provided as an unsigned integer in little-endian format.
* 			Restricted to the range (0 , 4MEG) 
*
* @note		See the Verilog file 'hw_detect.v' for specifics on how
* 			the output count values are generated.
*
*****************************************************************************/

unsigned int HWDET_get_count(_HWDET_register reg) {
	
	unsigned int count = 0x00000000;

	switch (reg) {

		case HIGH: 
			count = HWDET_mReadReg(HWDET_BaseAddress, HWDET_HIGH_COUNT_OFFSET);
			break;

		case LOW:
			count = HWDET_mReadReg(HWDET_BaseAddress, HWDET_LOW_COUNT_OFFSET);
			break;

		default:
			count = 0x00000000;
			break;
	}
	
	return count;
}

/*************** Calculate frequency from light intensity ***************/	
/**
* Returns the output frequency of the TSL235R sensor as an unsigned integer.  
* 
* This works through simple arithmetic on the count values generated by 
* the HWDET hardware module. The sensor datasheet specifies a maximum
* frequency of 500kHz without saturation, and a 0.4Hz frequency for
* 0 uW/cm^2 light incidence.
*
* @param	None
*
* @return	output frequency of the TSL235R light sensor.
* 			Provided as an unsigned integer in little-endian format.
* 			Restricted to the range [0 , 10MEG] 
*
* @note		See the TSL235R datasheet for details on output characteristics.
*
*****************************************************************************/

unsigned int HWDET_calc_freq(void) {

	unsigned int high_count = 0x00000000;
	unsigned int low_count 	= 0x00000000;
	unsigned int sum 		= 0x00000000;
	unsigned int freq 		= 0x00000000;

	high_count = HWDET_get_count(HIGH);
	low_count = HWDET_get_count(LOW);

	sum = (high_count + 1) + (low_count + 1);
	freq = (CPU_CLOCK_FREQ_HZ / sum);

	return freq;
}