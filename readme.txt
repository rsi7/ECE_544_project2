ECE544 Project 2 Release - Revision 7.0
Feb 2, 2016
(c) Roy Kravitz, Robin Marshall 2014- 2015, 2016
=================================================

This folder contains some of the files you will need to complete Project 2.  Included are C source code to test your control circuit. Also included are the schematic and possible BOMs (for two suppliers) for the control system circuit.  

NOTE:  This project was ported to the Nexys4 board the last time ECE 544 wss taught.  To my knowledge, the project was completed successfully by all of the students last year but a common thread was that the a sensor reading or two would occasionally be way out of line.  Hence the suggestion in the write-up to add digital filtering to sensor reading.  Still, like all projects, this one is a work in progress.  If you find problems or think some clarification is in order please let Roy know so he can make improvements for the next time.

NOTE 2: I have released an application called SerialCharter (written by one of my students) to plot the test results but there were many instances last year when the application would hang or in some way be unreliable. So, I pulled it from the release.  There is extra credit available for a team providing a reliable plotting application (an original work, not downloaded from the Internet) that runs on Windows 7 and above and Linux.  

NOTE 3: You do not have to control an LED with a light sensor.  You could do closed loop control on a motor or fan or a power supply.  There is extra credit avaialbe for the team that successfully demonstrates that.  The docs directory contains several articles on DC to DC converters which rely on closed-loop control to maintain the voltage.


The Project 2 release contains the following files:
-----------------------------------------------

docs directory:
	project2.pdf						Project 2 write-up
	project2_tasks.pdf					Project 2 task list					
	Output Control.pdf					Output Control material presented in class
	PID without a PhD					Excellent article by Tim Wescott on how to tune a PID circuit

	DC_DC Converters directory:				Some background information if you want to attempt a DC-DC converter controller
		Voltage Step-Up Techniques.pdf			Robert Lacoste Circuit Cellar article on DC-DC converters
		DC to DC Converter Basics.pdf			Robert Lacoste Circuit Cellar article (referred to in Voltage Step-Up article
		Microchip PIC App Note_216a.pdf			Microchip PIC article on how to use a microcontroller to do DC-DC conversion (referred											to in Voltage Step-Up article)

hardware directory:
	ControlSystem_hardware directory:
		BOM_Digi-Key.pdf				Component bill of materials for Digi-key
		BOM_Mouser.pdf					Component bill of materials for Mouser
		PMOD_schematic.pdf				Schematics of the suggested control system circuit
		TSL235R-LF.pdf					Datasheet for the light sensor

software directory:
	PmodCtlSys directory:	
		test_PmodCtlSys_r4.c				Test program for the PmodCtlSys.  Can be used as the basis for an application
								to characterize and optimize the operation of your control circuit. Requires your
								control circuit driver to run.	


Good luck
Roy  