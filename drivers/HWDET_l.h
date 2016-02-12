/**
*
* @file HWDET_l.h
*
* @author Rehan Iqbal (riqbal@pdx.edu)
* @copyright Portland State University, 2016
*
* This header file contains identifiers and low-level driver functions for the
* custom peripheral "HWDET". This peripheral is a hardware pulse detection module
* written in Verilog. It implements a simple edge-based counter to report 32-bit values
* for high interval length and low interval length.
*/

/****************************************************************************/
/**************************** Header Definition  ****************************/
/****************************************************************************/

// check if low-level header definition already exists...
// if not, define with the contents of this file

#ifndef HWDET_L_H
#define HWDET_L_H

/****************************************************************************/
/****************************** Include Files *******************************/
/****************************************************************************/

#include "xil_types.h"
#include "xil_io.h"
#include "xstatus.h"

/****************************************************************************/
/************************** Constant Definitions ****************************/
/****************************************************************************/

/** @name Registers
 *
 * Register offsets for this device.
 * @{
 */
#define HWDET_HIGH_COUNT_OFFSET 0
#define HWDET_LOW_COUNT_OFFSET 4
#define HWDET_RSVD00_OFFSET 8
#define HWDET_RSVD01_OFFSET 12

/* @} */

/****************************************************************************/
/**************************** Type Definitions ******************************/
/****************************************************************************/
/**
 *
 * Write a value to a HWDET register. A 32 bit write is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is written.
 *
 * @param   BaseAddress is the base address of the HWDETdevice.
 * @param   RegOffset is the register offset from the base to write to.
 * @param   Data is the data written to the register.
 *
 * @return  None.
 *
 * @note
 * C-style signature:
 * 	void HWDET_mWriteReg(u32 BaseAddress, unsigned RegOffset, u32 Data)
 *
 */

#define HWDET_mWriteReg(BaseAddress, RegOffset, Data) \
  	Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))

/**
 *
 * Read a value from a HWDET register. A 32 bit read is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is read from the register. The most significant data
 * will be read as 0.
 *
 * @param   BaseAddress is the base address of the HWDET device.
 * @param   RegOffset is the register offset from the base to write to.
 *
 * @return  Data is the data from the register.
 *
 * @note
 * C-style signature:
 * 	u32 HWDET_mReadReg(u32 BaseAddress, unsigned RegOffset)
 *
 */

#define HWDET_mReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))


/****************************************************************************/
/************************** Function Prototypes *****************************/
/****************************************************************************/
/**
 *
 * Run a self-test on the driver/device. Note this may be a destructive test if
 * resets of the device are performed.
 *
 * If the hardware system is not built correctly, this function may never
 * return to the caller.
 *
 * @param   baseaddr_p is the base address of the HWDET instance to be worked on.
 *
 * @return
 *
 *    - XST_SUCCESS   if all self-test code passed
 *    - XST_FAILURE   if any self-test code failed
 *
 * @note    Caching must be turned off for this function to work.
 * @note    Self test may fail if data memory and device are not on the same bus.
 * @note 	This test assumes a Serial port is available for xil_printf.

 */

XStatus HWDET_Reg_SelfTest(u32 baseaddr);

#endif