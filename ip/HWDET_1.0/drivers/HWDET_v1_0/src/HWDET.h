/**
*
* @file HWDET.h
*
* @author Rehan Iqbal (riqbal@pdx.edu)
* @copyright Portland State University, 2016
*
* This header file contains identifiers and driver functions for the
* custom peripheral "HWDET". This peripheral is a hardware pulse detection module
* written in Verilog. It implements a simple edge-based counter to report 32-bit values
* for high interval length and low interval length.
*/

/****************************************************************************/
/**************************** Header Definition  ****************************/
/****************************************************************************/

// check if header definition already exists...
// if not, define with the contents of this file

#ifndef HWDET_H
#define HWDET_H

/****************************************************************************/
/****************************** Include Files *******************************/
/****************************************************************************/

#include "xil_types.h"
#include "xstatus.h"
#include "stdbool.h"
#include "HWDET_l.h"

/****************************************************************************/
/************************** Constant Definitions ****************************/
/****************************************************************************/

/** @name Bit Masks
*
* Bit masks for the HWDET registers.
*
* All of the registers in the HWDET periheral are 32-bits wide
*
* @{
*/

// Masks for hardware detection module

#define		HWDET_UPPER_HALF_MASK 	0xFFFF0000
#define		HWDET_LOWER_HALF_MASK	0x0000FFFF

/* @} */

/****************************************************************************/
/**************************** Type Definitions ******************************/
/****************************************************************************/

/** @name Literals and constants
*
* Literals and constants used for selecting specific registers
*
*/

// Register selectors

enum _HWDET_register {HIGH, LOW};

/****************************************************************************/
/************************** Function Prototypes *****************************/
/****************************************************************************/

// Initialization function
int HWDET_initialize(u32 BaseAddr);

// Get count for high / low interval
unsigned int HWDET_get_count(enum _HWDET_register reg);

// Calculate frequency from light intensity
unsigned int HWDET_calc_freq(void);

#endif