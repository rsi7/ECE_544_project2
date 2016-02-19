/**
*
* @file test_PmodCtlSys_r4.c
*
* @author Rehan Iqbal (riqbal@pdx.edu)
* @copyright Portland State University, 2016
* 
* Description:
* 
* This program implements the full program for the Control System Pmod
* used in ECE 544 Project 2.  The program uses a Xilinx timer/counter 
* module in PWM mode and a light sensor with the custom HWDET peripheral.
*
*
*		sw[1:0] = 00:		Bang-bang control test. Use rotary encoder to dial in a desired setpoint,
*							then hold the encoder button to start the test. It will upload results
*							via UART for easy plotting.
*
* 		sw[1:0] = 01:		PID control test. Use the left & right pushbuttons to select a parameter,
							and use the up & down pushbuttons to adjust the values. Start the test
							by holding down the rotary encoder button. 
*
*		sw[1:0] = 10:		Unused by the software application.
*
*		sw[1:0] = 11:		Characterizes the response to the system by stepping the PWM duty cycle from
*							min (1%) to max (99%) after allowing the light sensor output to settle. Press and hold
*							the rotary encoder pushbutton to start the test.  Release the button when the
*							"Run" (rightmost) LED turns off to upload the data via serial port to a PC
*/

/****************************************************************************/
/***************************** Include Files ********************************/
/****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "xparameters.h"
#include "xil_types.h"
#include "xil_assert.h"
#include "xtmrctr.h"
#include "xintc.h"
#include "xgpio.h"
#include "Nexys4IO.h"
#include "PMod544IOR2.h"
#include "HWDET.h"
#include "pwm_tmrctr.h"
#include "mb_interface.h"

/****************************************************************************/
/************************** Constant Definitions ****************************/
/****************************************************************************/

// Clock frequencies

#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ
#define PWM_TIMER_FREQ_HZ		XPAR_AXI_TIMER_0_CLOCK_FREQ_HZ

// GPIO parameters

#define GPIO_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_BASEADDR			XPAR_XPS_GPIO_0_BASEADDR
#define GPIO_HIGHADDR			XPAR_XPS_GPIO_0_HIGHADDR
#define GPIO_INPUT_CHANNEL		1
#define GPIO_OUTPUT_CHANNEL		2

// Nexys4IO and Pmod544IO parameters

#define NX4IO_DEVICE_ID			XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR			XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR			XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

#define PMD544IO_DEVICE_ID		XPAR_PMOD544IOR2_0_DEVICE_ID
#define PMD544IO_BASEADDR		XPAR_PMOD544IOR2_0_S00_AXI_BASEADDR
#define PMD544IO_HIGHADDR		XPAR_PMOD544IOR2_0_S00_AXI_HIGHADDR

// HWDET I/O parameters

#define HWDET_DEVICE_ID			XPAR_HWDET_0_DEVICE_ID
#define HWDET_BASEADDR			XPAR_HWDET_0_S00_AXI_BASEADDR
#define HWDET_HIGHADDR			XPAR_HWDET_0_S00_AXI_HIGHADDR

// PWM timer parameters
// Set PWM frequency = 10KHz, duty cycle increments by 5%

#define PWM_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define PWM_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define PWM_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define PWM_FREQUENCY			10000	
#define PWM_VIN					3.3	
#define DUTY_CYCLE_CHANGE		2

// Min and Max duty cycle for step and characterization tests

#define STEPDC_MIN				1
#define STEPDC_MAX				99					
		
// Interrupt Controller parameters

#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define INTC_BASEADDR			XPAR_AXI_INTC_0_BASEADDR
#define INTC_HIGHADDR			XPAR_AXI_INTC_0_HIGHADDR
#define TIMER_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR
				
// Fixed Interval timer - 100MHz input clock, 5KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001

#define FIT_IN_CLOCK_FREQ_HZ	AXI_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		5000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			(FIT_CLOCK_FREQ_HZ / 1000)	

// sample settings	

#define NUM_FRQ_SAMPLES			250	

/****************************************************************************/
/***************** Macros (Inline Functions) Definitions ********************/
/****************************************************************************/

#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/****************************************************************************/
/*************************** Typdefs & Structures ***************************/
/****************************************************************************/
	
typedef enum {TEST_BANGBANG = 0x0, TEST_PID = 0x01, TEST_RSVD = 0x02, 
				TEST_CHARACTERIZE = 0x03, TEST_INVALID = 0xFF} Test_t;

typedef enum {P, I, D, SetMode} Menu;

typedef struct {

	signed int 	pGain;		// gain for proportional term
	signed int 	iGain;		// gain for integral term
	signed int 	dGain;		// gain for derivative term

	signed int 	iState;		// state for integral term
	signed int 	dState; 	// state for derivative term

	signed int 	iMin;		// minimum allowed value for integral term
	signed int 	iMax; 		// maximum allowed value for integral term

} sPID, * sPIDPtr;

/****************************************************************************/
/************************** Variable Definitions ****************************/	
/****************************************************************************/

// Microblaze peripheral instances

XIntc 	IntrptCtlrInst;								// Interrupt Controller instance
XTmrCtr	PWMTimerInst;								// PWM timer instance
XGpio	GPIOInst;									// GPIO instance

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global and declared volatile) -
// these are controlled by the FIT timer interrupt handler

volatile unsigned long	timestamp;					// timestamp since the program began
volatile u32			gpio_port = 0;				// GPIO port register - maintained in program

// The following variables are shared between the functions in the program
// such that they must be global

u16						sample[NUM_FRQ_SAMPLES];	// sample array 	
int						smpl_idx;					// index into sample array
int						frq_smple_interval;			// approximate sample interval			

int						pwm_freq;					// PWM frequency 
int						pwm_duty;					// PWM duty cycle

signed int 				FRQ_min_cnt;				// minimum freq value when duty = 1%
signed int 				FRQ_max_cnt;				// maximum freq value when duty = 99%

sPID * 					testPIDptr;					// pointer to the PID structure
Menu 					menu;						// global menu typedef
				
int						debugen = 0;				// debug level/flag
	
/****************************************************************************/
/************************** Function Prototypes *****************************/
/****************************************************************************/

XStatus 		DoTest_Track(void);										// Perform Tracking test
XStatus			DoTest_Step(int dc_start);								// Perform Step test
XStatus			DoTest_Characterize(void);								// Perform Characterization test
XStatus 		DoTest_BangBang(unsigned int setpoint);					// Perform Bang-Bang control test
XStatus 		DoTest_PID(unsigned int setpoint, sPID * PID);			// Perform PID control test
			
XStatus			do_init(void);											// initialize system
void			voltstostrng(float v, char* s);							// converts volts to a string
float 			freq2volt(short freq); 									// converts sensor frequency --> voltage

void			delay_msecs(u32 msecs);									// busy-wait delay for "msecs" milliseconds
void			FIT_Handler(void);										// fixed interval timer interrupt handler

void			update_lcd(int vin_dccnt, short vout_frqcnt);			// updates the LCD display
unsigned		update_menu(sPID * testPIDptr);							// updates the PID menu interface

/****************************************************************************/
/************************** MAIN PROGRAM ************************************/
/****************************************************************************/

int main() {

	XStatus 			Status;
	volatile u16		sw, old_sw = 0xFFFF;
	int					rotcnt, old_rotcnt = 0x1000;
	Test_t				test, next_test;

	// create & initialize a new PID structure

	sPID * testPIDptr = malloc(sizeof(sPID));

	testPIDptr->pGain 	= 0;
	testPIDptr->iGain 	= 0; 		
	testPIDptr->dGain 	= 0; 		
	testPIDptr->iState 	= 0;
	testPIDptr->dState 	= 0; 		
	testPIDptr->iMin 	= -1000; 		
	testPIDptr->iMax 	= 1000; 
	
	// initialize the menu to SetMode

	menu = SetMode;

	// initialize devices and set up interrupts, etc.

 	Status = do_init();

 	if (Status != XST_SUCCESS) {

 		PMDIO_LCD_setcursor(1,0);
 		PMDIO_LCD_wrstring("****** ERROR *******");
 		PMDIO_LCD_setcursor(2,0);
 		PMDIO_LCD_wrstring("INIT FAILED- EXITING");
 		exit(XST_FAILURE);
 	}

	// initialize the variables

	timestamp = 0;							
	pwm_freq = PWM_FREQUENCY;
	pwm_duty = STEPDC_MIN;
	next_test = TEST_INVALID;

	microblaze_enable_interrupts();
	 	  	
 	// display the greeting   

    PMDIO_LCD_setcursor(1,0);
    PMDIO_LCD_wrstring("PmodCtlSys Test ");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring("R4.0 by Rehan I.");
	NX4IO_setLEDs(0x0000FFFF);
	NX4IO_SSEG_putU32Hex(0x00000000);

	// Run the LED characterization routine to establish sensor min's and max's

    DoTest_Characterize();
	NX4IO_setLEDs(0x00000000);

       
    // main loop - there is no exit except by hardware reset

	while (1) {

		// read sw[1:0] to get the test to perform.

		sw = NX4IO_getSwitches() & 0x03;		
		
		// Test 00 = Bang-Bang Control
		
		if (sw == TEST_BANGBANG) {

			float			v;
			char			s[20];	
			unsigned int 	setpoint;
			
			// write the static info to the display if necessary

			PMDIO_LCD_clrd();
			PMDIO_LCD_setcursor(1,0);
			PMDIO_LCD_wrstring("|BANG|Press RBtn");
			PMDIO_LCD_setcursor(2,0);
			PMDIO_LCD_wrstring("SetPt:");

			// read the rotary encoder for target value
			PMDIO_ROT_readRotcnt(&rotcnt);

			// map the rotary reading to appropriate range
			// based on minimum & maximum frequency counts
			setpoint = MAX(FRQ_min_cnt, MIN(rotcnt, FRQ_max_cnt));

			// convert this to voltage to display on LCD
			v = freq2volt(setpoint);
			voltstostrng(v, s);

			// display on LCD screen				
			PMDIO_LCD_setcursor(2,6);
			PMDIO_LCD_wrstring(s);

			// debugging on 7-segment
			NX4IO_SSEG_putU32Dec(setpoint, 1);	
			
			// start the test on the rising edge of the Rotary Encoder button press
			// the test will write the light detector samples into the global "sample[]"
			// the samples will be sent to stdout when the Rotary Encoder button
			// is released
			
			// test whether the rotary encoder button has been pressed

			if (PMDIO_ROT_isBtnPressed())  {

				// do the step test and dump data 		
				// light "Run" (rightmost) LED to show the test has begun
				// and do the test.  The test will return when the measured samples array
				// has been filled.  Turn off the rightmost LED after the data has been 
				// captured to let the user know he/she can release the button

				NX4IO_setLEDs(0x00000001);

				// perform bang-bang control test

				DoTest_BangBang(setpoint);

				NX4IO_setLEDs(0x00000000);
				
				// wait for the Rotary Encoder button to be released
				// and then send the sample data to stdout

				do {
					delay_msecs(10);
				} while ( PMDIO_ROT_isBtnPressed () );
				
				// light "Transfer" LED to indicate that data is being transmitted
				// Show the traffic on the LCD

				NX4IO_setLEDs(0x00000002);
				PMDIO_LCD_clrd();
				PMDIO_LCD_setcursor(1, 0);
				PMDIO_LCD_wrstring("Sending Data....");
				PMDIO_LCD_setcursor(2, 0);
				PMDIO_LCD_wrstring("S:    DATA:     ");

				// print the descriptive heading followed by the data
				
				xil_printf("\n\rBang-Bang Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);

				// trigger the serial charter program

				xil_printf("===STARTPLOT===\n");

				// start with the second sample.  The first sample is not representative of
				// the data.  This will pretty-up the graph a bit		

				for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++) {

					u16 count;
					
					count = sample[smpl_idx];
					
                    //Convert from count to 'volts'
					v = freq2volt(count);

					voltstostrng(v, s);
					xil_printf("%d\t%d\t%s\n\r", smpl_idx, count, s);
					
					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_wrstring("   ");
					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_putnum(smpl_idx, 10);
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_wrstring("     ");
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_putnum(count, 10);
				}
				
				// stop the serial charter program	

				xil_printf("===ENDPLOT===\n");
				
				NX4IO_setLEDs(0x00000000);								
				next_test = TEST_INVALID;
			}

			else {
				next_test = test;		
			}

			delay_msecs(500);

		} // end Bang-Bang test

		// Test 01 = PID Control
		
		if (sw == TEST_PID) {

			float			v;
			char			s[20];	
			unsigned int 	setpoint;		
			
			// call the update_menu function to get new PID parameters
			// that are used in the DoTest_PID function
			// also updates the setpoint if menu state is 'SetMode'

			setpoint = update_menu(testPIDptr);
			
			// start the test on the rising edge of the Rotary Encoder button press
			// the test will write the light detector samples into the global "sample[]"
			// the samples will be sent to stdout when the Rotary Encoder button
			// is released
			
			// test whether the rotary encoder button has been pressed

			if (PMDIO_ROT_isBtnPressed())  {

				// do the step test and dump data 		
				// light "Run" (rightmost) LED to show the test has begun
				// and do the test.  The test will return when the measured samples array
				// has been filled.  Turn off the rightmost LED after the data has been 
				// captured to let the user know he/she can release the button

				NX4IO_setLEDs(0x00000001);

				// perform PID control test

				DoTest_PID(setpoint, testPIDptr);

				NX4IO_setLEDs(0x00000000);
				
				// wait for the Rotary Encoder button to be released
				// and then send the sample data to stdout

				do {
					delay_msecs(10);
				} while ( PMDIO_ROT_isBtnPressed () );
				
				// light "Transfer" LED to indicate that data is being transmitted
				// Show the traffic on the LCD

				NX4IO_setLEDs(0x00000002);
				PMDIO_LCD_clrd();
				PMDIO_LCD_setcursor(1, 0);
				PMDIO_LCD_wrstring("Sending Data....");
				PMDIO_LCD_setcursor(2, 0);
				PMDIO_LCD_wrstring("S:    DATA:     ");

				// print the descriptive heading followed by the data
				
				xil_printf("\n\rPID Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);

				// trigger the serial charter program

				xil_printf("===STARTPLOT===\n");

				// start with the second sample.  The first sample is not representative of
				// the data.  This will pretty-up the graph a bit		

				for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++) {

					u16 count;
					
					count = sample[smpl_idx];
					
                    //Convert from count to 'volts'

					v = freq2volt(count);

					voltstostrng(v, s);
					xil_printf("%d\t%d\t%s\n\r", smpl_idx, count, s);
					
					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_wrstring("   ");
					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_putnum(smpl_idx, 10);
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_wrstring("     ");
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_putnum(count, 10);
				}
				
				// stop the serial charter program	

				xil_printf("===ENDPLOT===\n");
				
				NX4IO_setLEDs(0x00000000);								
				next_test = TEST_INVALID;
			}

			else {
				next_test = test;		
			}

			delay_msecs(300);

		} // end PID test

		// Test 11 - Characterize Response

		else if (sw == TEST_CHARACTERIZE) {

			PMDIO_LCD_clrd();
			PMDIO_LCD_setcursor(1,0);
			PMDIO_LCD_wrstring("|CHAR|Press RBtn");
			PMDIO_LCD_setcursor(2,0);
			PMDIO_LCD_wrstring("LED OFF-Release ");

			// start the test on the rising edge of the Rotary Encoder button press
			// the test will write the samples into the global "sample[]"
			// the samples will be sent to stdout when the Rotary Encoder button
			// is released

			// test whether the rotary encoder button has been pressed

			if (PMDIO_ROT_isBtnPressed())  { 

				// do the step test and dump data 

				// light "Run" (rightmost) LED to show the test has begun
				// and do the test.  The test will return when the measured samples array
				// has been filled.  Turn off the rightmost LED after the data has been
				// captured to let the user know he/she can release the button

				NX4IO_setLEDs(0x00000001);			
				DoTest_Characterize();
				NX4IO_setLEDs(0x00000000);
				
				// wait for the Rotary Encoder button to be released
				// and then send the sample data to stdout

				do {
					delay_msecs(10);
				} while ( PMDIO_ROT_isBtnPressed() );
				
				// light "Transfer" LED to indicate that data is being transmitted
				// Show the traffic on the LCD

				NX4IO_setLEDs(0x00000002);
				PMDIO_LCD_clrd();
				PMDIO_LCD_setcursor(1, 0);
				PMDIO_LCD_wrstring("Sending Data....");
				PMDIO_LCD_setcursor(2, 0);
				PMDIO_LCD_wrstring("S:    DATA:     ");

				xil_printf("\n\rCharacterization Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);

				// trigger the serial charter program

				xil_printf("===STARTPLOT===\n\r");
				
				for (smpl_idx = STEPDC_MIN; smpl_idx <= STEPDC_MAX; smpl_idx++) {

					u16 		count;
					float		v;
					char		s[10]; 
					
					count = sample[smpl_idx];
					
                    //Convert from count to 'volts'
					v = freq2volt(count);

					voltstostrng(v, s);
					xil_printf("%d\t%d\t%s\n\r", smpl_idx, count, s);

					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_wrstring("   ");
					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_putnum(smpl_idx, 10);
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_wrstring("     ");
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_putnum(count, 10);
				}

				// stop the serial charter program	

				xil_printf("===ENDPLOT===\n\r");
				
				NX4IO_setLEDs(0x00000000);								
				next_test = TEST_INVALID;
			}

			else {
				next_test = test;
			}

			delay_msecs(400);

		} // end Characterize test
	} // end while loop
} // end main loop

/****************************************************************************/
/***************************** Test Functions *******************************/

/****************************************************************************
*
* DoTest_Characterize() - Perform the Characterization test
* 
* This function starts the duty cycle at the minimum duty cycle and
* then sweeps it to the max duty cycle for the test.
* Samples are collected into the global array sample[].  
* The function toggles the TEST_RUNNING signal high for the duration
* of the test as a debug aid and adjusts the global "pwm_duty"
*
* The test also sets the global frequency count min and max counts to
* help limit the counts to the active range for the circuit
*
 ****************************************************************************/

XStatus DoTest_Characterize(void) {

	XStatus		Status;					// Xilinx return status
	unsigned	tss;					// starting timestamp
	int			n;						// number of samples

	// stabilize the PWM output (and thus the lamp intensity) at the
	// minimum before starting the test

	pwm_duty = STEPDC_MIN;
	Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);

	if (Status == XST_SUCCESS) {							
		PWM_Start(&PWMTimerInst);
	}

	else {
		return -1;
	}

	// Wait for the LED output to settle before starting

    delay_msecs(1500);
		
	// sweep the duty cycle from STEPDC_MIN to STEPDC_MAX

	smpl_idx = STEPDC_MIN;
	n = 0;
	tss = timestamp;

	while (smpl_idx <= STEPDC_MAX) {

		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, smpl_idx);
		
		if (Status == XST_SUCCESS) {							
			PWM_Start(&PWMTimerInst);
		}

		else {
			return -1;
		}
		
		// wait for the new PWM duty to settle...
        // then make the light sensor measurement
		
		delay_msecs(50);
		sample[smpl_idx++] = HWDET_calc_freq();
		
		n++;
	}		

	frq_smple_interval = (timestamp - tss) / smpl_idx;

    // Find the min and max values and set the scaling/offset factors
    // these are used in the freq2volt function
	
	FRQ_min_cnt = sample[STEPDC_MIN];
	FRQ_max_cnt = sample[STEPDC_MAX];

    return n;
}
	
/****************************************************************************/
/*************************** Support Functions ******************************/
/****************************************************************************/

/****************************************************************************
 * do_init() - initialize the system
 * 
 * This function is executed once at start-up and after a reset.  It initializes
 * the peripherals and registers the interrupt handlers
 *
 ****************************************************************************/

XStatus do_init(void) {
	
	XStatus 	status;				// status from Xilinx Lib calls
	
	// initialize the Nexys4IO
	
	status = NX4IO_initialize(NX4IO_BASEADDR);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// initialize the PMod544IO
	// rotary encoder is set to increment from 0 by DUTY_CYCLE_CHANGE 

	status = PMDIO_initialize(PMD544IO_BASEADDR);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

 	PMDIO_ROT_init(DUTY_CYCLE_CHANGE, true);
	PMDIO_ROT_clear();

	// initialize the HWDET

	status = HWDET_initialize(HWDET_BASEADDR);

	if (status != XST_SUCCESS) {
		xil_printf("\nFailed on HWDET initialization!\n");
		return XST_FAILURE;
	}

	// initialize the GPIO instance

	status = XGpio_Initialize(&GPIOInst, GPIO_DEVICE_ID);
	
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// GPIO channel 2 is an 8-bit output port that your application can
	// use.  None of the bits are used by this program

	XGpio_SetDataDirection(&GPIOInst, GPIO_OUTPUT_CHANNEL, 0x00);
	XGpio_DiscreteWrite(&GPIOInst, GPIO_OUTPUT_CHANNEL, gpio_port);
	
			
	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts. Clock frequency is the AXI clock frequency

	status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false, CPU_CLOCK_FREQ_HZ);
	
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	
	// initialize the interrupt controller

	status = XIntc_Initialize(&IntrptCtlrInst,INTC_DEVICE_ID);
    
    if (status != XST_SUCCESS) {
       return XST_FAILURE;
    }

	// connect the fixed interval timer (FIT) handler to the interrupt
    
    status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID, (XInterruptHandler)FIT_Handler, (void *)0);

    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }
 
 	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts, specifically real mode so that
	// the the  FIT can cause interrupts thru the interrupt controller.

    status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);

    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    } 
      
 	// enable the FIT interrupt

    XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);

    // all initialization completed successfully... return from function now

	return XST_SUCCESS;
}
		
/****************************************************************************
 * delay_msecs() - delay execution for "n" msecs
 * 
 *		timing is approximate but we're not looking for precision here, just
 *		a uniform delay function.  The function uses the global "timestamp" which
 *		is incremented every msec by FIT_Handler().
 *
 * NOTE:  Assumes that this loop is running faster than the fit_interval ISR (every msec)
 *
 ****************************************************************************/

void delay_msecs(u32 msecs) {

	unsigned long target;

	if ( msecs == 0 ) {
		return;
	}

	target = timestamp + msecs;

	while ( timestamp != target ) {
		// spin until delay is over
	}
}

/****************************************************************************
 * voltstostrng() - converts volts to a fixed format string
 * 
 * accepts an Xfloat32 voltage reading and turns it into a 5 character string
 * of the following format:
 *		(+/-)x.yy 
 * where (+/-) is the sign, x is the integer part of the voltage and yy is
 * the decimal part of the voltage.
 *	
 * NOTE:  Assumes that s points to an array of at least 6 bytes.
 *	
 ****************************************************************************/
 
void voltstostrng(float v, char* s) {

	float		dpf, ipf;
	int			dpi;
	int			ones, tenths, hundredths;

	// form the fixed digits

	dpf = modff(v, &ipf);
	dpi = dpf * 100;
	ones = abs(ipf) + '0';
	tenths = (dpi / 10) + '0';
	hundredths = (dpi - ((tenths - '0') * 10)) + '0';
	 
	// form the string and return

	*s++ = ipf == 0 ? ' ' : (ipf > 0 ? '+' : '-');
	*s++ = (char) ones;
	*s++ = '.';
	*s++ = (char) tenths;
	*s++ = (char) hundredths;
	*s   = 0;

	return ;
}  
	 
/****************************************************************************
* update_lcd() - update the LCD display with a new count and voltage
*
* writes the display with new information.  "vin_dccnt" is the  unsigned PWM duty
* cycle and "frqcnt" is the signed frq_count.  The function assumes that the
* static portion of the display has been written and that the dynamic portion of
* the display is the same for all tests
*
****************************************************************************/
 
void update_lcd(int vin_dccnt, short frqcnt) {

	float			v;
	char			s[10];

	// update the PWM data

	v = vin_dccnt * .01 * PWM_VIN;

	// convert the PWM data to a string

	voltstostrng(v, s);

	// now print that string

	PMDIO_LCD_setcursor(1, 11);
	PMDIO_LCD_wrstring("      ");
	PMDIO_LCD_setcursor(1, 11);
	PMDIO_LCD_wrstring(s);

	// converting frequency --> voltage
	// this returns a float type number
	// that can be used for voltstostrng

	v = freq2volt(frqcnt);

	// call voltstostrng to get a string

	voltstostrng(v, s);

	// print the string to the LCD screen

	PMDIO_LCD_setcursor(2, 3);
	PMDIO_LCD_wrstring("     ");
	PMDIO_LCD_setcursor(2, 3);
	PMDIO_LCD_wrstring(s);
	PMDIO_LCD_setcursor(2, 11);
	PMDIO_LCD_wrstring("     ");
	PMDIO_LCD_setcursor(2, 11);
	PMDIO_LCD_putnum(frqcnt, 10);

	return ;
}

/****************************************************************************
* freq2volt - Converts detected frequency into an estimated applied voltage
*
* Using the FRQ_min_count and FRQ_max_count values from the characterization,
* the function scales the sensor frequency into an applied voltage.
* This is done with simple integer & float math.
*
* Because of the non-linearities around 1% - 10% duty cycle, the calculated
* voltage cannot be guaranteed to be accurate! Use frequency when possible...
* only use the voltage on the LCD screen and for display.
*
****************************************************************************/

float freq2volt(short freq) {

 	float 			v;
 	signed int 		freq_signed;

 	freq_signed = (int) freq;

 	// calculate the voltage
 	// by scaling to get duty cycle (temporary sum)
 	// then multiplying it by +3.3V

 	v = PWM_VIN * ((float) (freq_signed - FRQ_min_cnt) / (float) (FRQ_max_cnt - FRQ_min_cnt));

  	return v;
 }

/****************************************************************************/
/*************************** Interrupt Handlers *****************************/
/****************************************************************************/

/****************************************************************************
 * FIT_Handler() - Fixed interval timer interrupt handler 
 *  
 * updates the global "timestamp" every millisecond.  
 * "timestamp" is used for the delay_msecs() function
 * and as a time stamp for data collection and reporting
 *
 ****************************************************************************/

void FIT_Handler(void) {

	// interval counter for incrementing timestamp
	
	static	int			ts_interval = 0;
			
	// update timestamp

	ts_interval++;	

	if (ts_interval > FIT_COUNT_1MSEC) {
		timestamp++;
		ts_interval = 1;
	}
}

/****************************************************************************
 * DoTest_BangBang() - On/off control loop algorithm
 *  
 * This function executes the bang-bang control test. The initial voltage is
 * set based on the target setpoint, and the LED is given 1.5s to settle to
 * this initial voltage.
 *
 * Then either 1% or 99% duty cycle is applied depending on whether the current
 * frequency is higher/lower than the setpoint. 250 samples are collected into
 * the global sample array. The function returns a success / failure code.
 *
 ****************************************************************************/

XStatus DoTest_BangBang(unsigned int setpoint) {

	XStatus		Status;					// Xilinx return status
	unsigned	tss;					// starting timestamp
	unsigned 	sensor_value;			// frequency count from sensor [10, 400]
		
	// if setpoint is higher than halfway point --> set initial voltage to 0.0V
	// if setpoint is lower than halway point --> set initial voltage to +3.3V

	if (setpoint > FRQ_max_cnt / 2) {
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, STEPDC_MIN);
	}

	else {
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, STEPDC_MAX);
	}

	// start the PWM now (hopefully)...

	if (Status == XST_SUCCESS) {
		PWM_Start(&PWMTimerInst);
	}

	else {
		xil_printf("Died trying to initialize PWM for test...");
		return XST_FAILURE;
	}

	// wait for LED output to settle before starting

	delay_msecs(1500);

	// time to run the test & collect data

	smpl_idx = 0;
	tss = timestamp;

	while (smpl_idx < NUM_FRQ_SAMPLES) {

		// light sensor measurement using HWDET...
		// store values in global array sample[ ]
		// also, increment the sample index

		sensor_value = HWDET_calc_freq();
		sample[smpl_idx++] = sensor_value;

		// bang-bang control algorithm (in one line)

		// use ternary operator to choose between [1,99] for new pwm_duty
		// based on whether sensor reading is higher / lower than setpoint

		pwm_duty = sensor_value > setpoint ? STEPDC_MIN : STEPDC_MAX;

		// update the PWM duty cycle...

		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);

		// make sure new pwm_duty updated correctly...

		if (Status == XST_SUCCESS) {
			PWM_Start(&PWMTimerInst);
		}

		else {
			xil_printf("Died while updating pwm_duty to %d...", pwm_duty);
			return XST_FAILURE;
		}

		// arbitrary sampling delay to make the graphs look better...

		delay_msecs(1);
	}		

	// all samples collected and loop is finished...
	// measure sample time interval by subtracting timestamps

	frq_smple_interval = (timestamp - tss) / NUM_FRQ_SAMPLES;

	// mission accomplished... return to caller

	return XST_SUCCESS;
}

/****************************************************************************
* DoTest_PID() - PID control test
*  
* This function sets up and executes the PID control test. It starts
* by setting the initial condition based on setpoint - if the setpoint is
* high, it initializes to 0.0V, and if setpoint is low, it initializes to 3.3V
*
* Then it runs the PID test. The pTerm, iTerm, and dTerms are calculated
* by loading the new gains from the update_menu function. Then they are multiplied
* by the appropriate error function.
*
* The integral term has restricted to a minimum / maximum range, divided by 128,
* and can only accumulate within +/- 12.5% of the setpoint value. All
* these are an effort to limit integral windup and stabilize long-term output.
*
* The new duty cycle is calculated from the sum of these three terms and 
* then applied. The sensor readings are updated in the global sample array
* and the test finishes after 250 samples.
*
 ****************************************************************************/

XStatus DoTest_PID(unsigned int setpoint, sPID * PID) {

	XStatus		Status;					// Xilinx return status
	unsigned	tss;					// starting timestamp
	unsigned 	sensor_value;			// frequency count from sensor [10, 400]
	signed 		error;					// signed value of the error
	signed 		prev_error;				// signed value of the last iteration's error

	signed 		pTerm;					// proportional term (P = Gp * error)
	signed 		iTerm;					// integral term (I = Gi + error)
	signed 		dTerm;					// derivative term (D = Gd * (error - prev_error))
		
	// if setpoint is higher than halfway point --> set initial voltage to 0.0V
	// if setpoint is lower than halway point --> set initial voltage to +3.3V

	if (setpoint > (FRQ_max_cnt / 2)) {
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, STEPDC_MIN);
	}

	else {
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, STEPDC_MAX);
	}

	// start the PWM now (hopefully)...

	if (Status == XST_SUCCESS) {
		PWM_Start(&PWMTimerInst);
	}

	else {
		return XST_FAILURE;
	}

	// wait for LED output to settle before starting

	delay_msecs(1500);

	// some debugging statements
	
	xil_printf("The setpoint is: %d\n", setpoint);
	xil_printf("The P constant is: %d\n", PID->pGain);
	xil_printf("The I constant is: %d\n", PID->iGain);
	xil_printf("The D constant is: %d\n", PID->dGain);

	// time to run the test & collect data

	smpl_idx = 0;
	tss = timestamp;

	while (smpl_idx < NUM_FRQ_SAMPLES) {

		// light sensor measurement using HWDET...
		// store values in global array sample[ ]
		// also, increment the sample index

		sensor_value = HWDET_calc_freq();
		sample[smpl_idx++] = sensor_value;


		// PID control algorithm

		// first, calculate error and store previous error

		prev_error = error;
		error = setpoint - sensor_value;

		// Proportional term

		pTerm = (PID->pGain) * error;

		// Intergral term
		
		// only accumulate within +/- 12.5% of target value

		PID->iState = (abs(error) < (setpoint / 8)) ? ((PID->iState) + error) : (PID->iState);

		// bound it to maximum / minimum values

		PID->iState = ((PID->iState) > (PID->iMax)) ? (PID->iMax) : (PID->iState);
		PID->iState = ((PID->iState) < (PID->iMin)) ? (PID->iMin) : (PID->iState);

		// scale it down by ~100 
		// so iGain = 1 is limited to +/- 10% effect on duty

		iTerm = ((PID->iGain) * (PID->iState))/128;

		// Derivative term

		(PID->dState) = error - prev_error;
		dTerm = (PID->dGain) * (PID->dState);

		// PID sum converted to applied duty cycle
		// making sure to bound to 1% - 99% range

		pwm_duty = pTerm + iTerm + dTerm;
		pwm_duty = MAX(STEPDC_MIN, MIN(pwm_duty, STEPDC_MAX));

		// apply this new PWM duty cycle

		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);

		// make sure new pwm_duty updated correctly...

		if (Status == XST_SUCCESS) {
			PWM_Start(&PWMTimerInst);
		}

		else {
			return XST_FAILURE;
		}

		// arbitrary sampling delay to make the graph look smoother...

		delay_msecs(1);
	}		

	// all samples collected and loop is finished...
	// measure sample time interval by subtracting timestamps

	frq_smple_interval = (timestamp - tss) / NUM_FRQ_SAMPLES;

	// mission accomplished... return to caller

	return XST_SUCCESS;
}


/****************************************************************************
 * update_menu() - implements the menu interface for setting PID parameters
 *  
 * This function allows the user to adjust the PID test parameters.
 * A case statement evaluates the menu state - for 'SetMode' the rotary
 * encoder is used to dial in a target frequency (similar to bang-bang).
 *
 * For the other three menu states, the buttons are checked with a case statement
 * to determine whether to increment/decrement the gain or switch to a 
 * different menu mode.
 *
 * Function wraps up by updating the gain parameter and writing it to the 
 * PID structure so that the PID control test can use it. It also updates
 * the 7-segment display with this information.
 *
 ****************************************************************************/

unsigned update_menu(sPID * testPIDptr) {

	volatile unsigned 	btns = 0x00;
	static unsigned 	setpoint = 100;

	float				v;
	char				s[20];
	int					rotcnt = 0x1000;

	static int			menuP;
	static int			menuI;
	static int			menuD;

	PMDIO_LCD_clrd();

	switch (menu) {

		case P:

			PMDIO_LCD_setcursor(1,0);
			PMDIO_LCD_wrstring("|P| adjust pGain");
			PMDIO_LCD_setcursor(2,0);
			PMDIO_LCD_wrstring("Use up/down btns");

			btns = NX4IO_getBtns();

			switch (btns) {

				case (0x01) :	menu = SetMode; break;		// left button
				case (0x02) :	menu = I; 		break;		// right button
				case (0x04) : 	menuP -=1; 		break;		// down button
				case (0x08) : 	menuP +=1; 		break;		// up button
			}

			// map the rotary reading to appropriate range
			menuP = MAX(0, MIN(menuP, 100));

			// debugging on 7-segment
			NX4IO_SSEG_putU32Dec(menuP, 1);

			// update PID structure
			(testPIDptr->pGain) = (int) menuP;

			break;

		case I:

			PMDIO_LCD_setcursor(1,0);
			PMDIO_LCD_wrstring("|I| adjust iGain");
			PMDIO_LCD_setcursor(2,0);
			PMDIO_LCD_wrstring("Use up/down btns");

			btns = NX4IO_getBtns();

			switch (btns) {

				case (0x01) :	menu = P; 		break;		// left button
				case (0x02) :	menu = D; 		break;		// right button
				case (0x04) : 	menuI -=1; 		break;		// down button
				case (0x08) : 	menuI +=1; 		break;		// up button
			}

			// map the rotary reading to appropriate range
			menuI = MAX(0, MIN(menuI, 100));

			// debugging on 7-segment
			NX4IO_SSEG_putU32Dec(menuI, 1);

			// update PID structure
			(testPIDptr->iGain) = (int) menuI;

			break;

		case D:

			PMDIO_LCD_setcursor(1,0);
			PMDIO_LCD_wrstring("|D| adjust dGain");
			PMDIO_LCD_setcursor(2,0);
			PMDIO_LCD_wrstring("Use up/down btns");

			btns = NX4IO_getBtns();

			switch (btns) {

				case (0x01) :	menu = I; 			break;			// left button
				case (0x02) :	menu = SetMode; 	break;			// right button
				case (0x04) : 	menuD -=1; 			break;			// up button
				case (0x08) : 	menuD +=1; 			break;			// down button

			}

			// map the rotary reading to appropriate range
			menuD = MAX(0, MIN(menuD, 100));

			// debugging on 7-segment
			NX4IO_SSEG_putU32Dec(menuD, 1);

			// update PID structure
			(testPIDptr->dGain) = (int) menuD;

			break;

		case SetMode:

			PMDIO_LCD_clrd();
			PMDIO_LCD_setcursor(1,0);
			PMDIO_LCD_wrstring("|PID| Press RBtn");
			PMDIO_LCD_setcursor(2,0);
			PMDIO_LCD_wrstring("SetPt:");
			
			btns = NX4IO_getBtns();

			switch (btns) {

				case (0x01) :	menu = D; break;		// left button
				case (0x02) :	menu = P; break;		// right button

			}

			// read the rotary encoder for target value
			PMDIO_ROT_readRotcnt(&rotcnt);

			// map the rotary reading to appropriate range
			// based on minimum & maximum frequency counts

			setpoint = MAX(FRQ_min_cnt, MIN(rotcnt, FRQ_max_cnt));

			// convert this to voltage to display on LCD
			v = freq2volt(setpoint);
			voltstostrng(v, s);

			// display on LCD screen				
			PMDIO_LCD_setcursor(2,6);
			PMDIO_LCD_wrstring(s);

			// debugging on 7-segment
			NX4IO_SSEG_putU32Dec(setpoint, 1);

			break;
	}

	return setpoint;
}