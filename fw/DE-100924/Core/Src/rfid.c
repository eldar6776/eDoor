/*
IBEX UK LTD http://www.ibexuk.com
Electronic Product Design Specialists
RELEASED SOFTWARE

The MIT License (MIT)

Copyright (c) 2013, IBEX UK Ltd, http://ibexuk.com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
//Project Name:		HTRC110 EM4102 125KHZ RFID READER DRIVER
//DRIVER C CODE FILE



#include "main.h"					//Global data type definitions (see https://github.com/ibexuk/C_Generic_Header_File )
#define RFID_C
#include "rfid.h"




//******************************
//******************************
//********** READ TAG **********
//******************************
//******************************
//Call this function to start the read tag process
void rfid_read_tag(void)
{
	rfid_read_tag_pending = 1;
}



//*****************************************
//*****************************************
//********** READ TAG GET RESULT **********
//*****************************************
//*****************************************
//Call this function to check for the end of the read tag process
//Returns 0 if still reading, 1 if completed.  Bit 7 will be set if the read failed.
//	After a sucessful read the ASCII tag ID will be in rfid_data_nibbles[0] | rfid_data_nibbles[9]
uint8_t rfid_get_read_tag_result(void)
{
	//----- CHECK FOR STILL READING -----
	if((rfid_read_tag_pending) || (rfid_state != RFID_IDLE))
		return(0x00);

	//----- READ IS COMPLETE -----
	if(rfid_read_tag_success)
		return(0x01);				//Success
	else
		return(0x81);				//Failed
}



//**************************************
//**************************************
//********** FORCE INITIALISE **********
//**************************************
//**************************************
//In typical use this function is not requried as the driver will automatically initialise the HTRC110 IC on powerup.  However if you are using
//this driver with multiple HTRC110 devices (for instance by switching the SCLK, DIN and DOUT lines by using one or more 74HC4053 IC's) then this
//function can be called during your applications initialisation after selecting each HTRC110.
void rfid_force_initialise(void)
{
	rfid_state = RFID_INITIALISE;

	while(rfid_state != RFID_IDLE)
		rfid_process();
}




//**********************************
//**********************************
//********** PROCESS RFID **********
//**********************************
//**********************************
//This function should be called reguarly as part of your applications main loop
void rfid_process(void)
{
	uint8_t tx_data;
	uint8_t response;
	uint16_t t_ant;

	switch(rfid_state)
	{
		case RFID_INITIALISE:
			//----------------------
			//----------------------
			//----- INITIALISE -----
			//----------------------
			//----------------------
			//Pause for >= 10mS for HTRC110 to be ready (specified in section 11.1 of the Philips "Read/Write Devices based on the HITAG Read/Write IC HTRC110" application note AN97070 rev 1.1)
			rfid_1ms_timer = 11;
			rfid_state = RFID_INITIALISE_1;
			break;

		case RFID_INITIALISE_1:
			//------------------------
			//------------------------
			//----- INITIALISE 1 -----
			//------------------------
			//------------------------
			if(rfid_1ms_timer)
				break;

			rfid_state = RFID_SET_IDLE;
			break;

		case RFID_SET_IDLE:
			//----------------------
			//----------------------
			//----- GO TO IDLE -----
			//----------------------
			//----------------------
			//Default to a gain value of 2 (g=500 - this is default).  Permitted values are 0, 1, 2 or 3.
			rfid_gain = 2;
			//SET_CONFIG_PAGE page=0x3 data=0x0 (DISLP1=0, DISSMARTCOMP=0, FSEL=01[8MHz osc])
			rfid_tx_rx_byte(0x71, 0);
			//SET_CONFIG_PAGE page=0x0 data=0xB (GAIN=##, FILTERH=1, FILTERL=1)
			tx_data = 0x43 | (rfid_gain << 2);
			rfid_tx_rx_byte(tx_data, 0);
			//SET_CONFIG_PAGE page=0x2 data=0xF (THRESET=1, ACQAMP=1, FREEZE=11)
			rfid_tx_rx_byte(0x6f, 0);
			//SET_CONFIG_PAGE page=0x1 data=0x5 (PD_MODE=0[active], PD=1[power down], HYSTERESIS=0[off], TXDIS=1[coil driver off])
			rfid_tx_rx_byte(0x55, 0);
			rfid_state = RFID_IDLE;
			break;

		case RFID_IDLE:
			//----------------
			//----------------
			//----- IDLE -----
			//----------------
			//----------------
			//RFID is in inactive low power state
			//----- CHECK FOR READ TAG REQUEST -----
			if(rfid_read_tag_pending)
				rfid_state = RFID_READ_TAG_1;
			break;

		case RFID_READ_TAG_1:
			//----------------------
			//----------------------
			//----- READ TAG 1 -----
			//----------------------
			//----------------------
			//SET_CONFIG_PAGE page=0x3 data=0x0 (DISLP1=0, DISSMARTCOMP=0, FSEL=01[8MHz osc])
			rfid_tx_rx_byte(0x71, 0);
			//SET_CONFIG_PAGE page=0x0 data=0xB (GAIN=##, FILTERH=1, FILTERL=1)
			tx_data = 0x43 | (rfid_gain << 2);
			rfid_tx_rx_byte(tx_data, 0);
			//SET_CONFIG_PAGE page=0x1 data=0x0 (PD_MODE=0[active], PD=0[active], HYSTERESIS=0[off], TXDIS=0[coil driver on])
			rfid_tx_rx_byte(0x50, 0);
			//SET_CONFIG_PAGE page=0x2 data=0xB (THRESET=1, ACQAMP=0, FREEZE=11)
			rfid_tx_rx_byte(0x6b, 0);
			//Pause for >= 5mS for HTRC110 to be ready (specified in section 11.4 of the Philips "Read/Write Devices based on the HITAG Read/Write IC HTRC110" application note AN97070 rev 1.1)
			rfid_1ms_timer = 6;
			rfid_state = RFID_READ_TAG_2;
			break;

		case RFID_READ_TAG_2:
			//----------------------
			//----------------------
			//----- READ TAG 2 -----
			//----------------------
			//----------------------
			if(rfid_1ms_timer)
				break;
			//SET_CONFIG_PAGE page=0x2 data=0x8 (THRESET=1, ACQAMP=0, FREEZE=00[normal operation])
			rfid_tx_rx_byte(0x68, 0);
			//Pause for >= 1mS for HTRC110 to be ready (specified in section 11.4 of the Philips "Read/Write Devices based on the HITAG Read/Write IC HTRC110" application note AN97070 rev 1.1)
			rfid_1ms_timer = 2;
			rfid_state = RFID_READ_TAG_3;
			break;

		case RFID_READ_TAG_3:
			//----------------------
			//----------------------
			//----- READ TAG 3 -----
			//----------------------
			//----------------------
			if(rfid_1ms_timer)
				break;
			//SET_CONFIG_PAGE page=0x2 data=0x0 (THRESET=0, ACQAMP=0, FREEZE=00)
			rfid_tx_rx_byte(0x60, 0);
			//----- CHECK THE ANTENNA IS OK -----
			//GET_CONFIG_PAGE page=2
			response = rfid_tx_rx_byte(0x06, 1);
			if(response & 0x10)
			{
				//ANTFAIL IS SET - Antenna failure
				rfid_state = RFID_READ_FAIL;
				break;
			}
			//----- SET THE SAMPLING TIME -----
			//READ_PHASE
			t_ant = (uint16_t)rfid_tx_rx_byte(0x08, 1);
			t_ant <<= 1;							//Multiply by 2
			t_ant += RFID_T_OC;						//Add the Offset Compensation value
			t_ant &= 0x3f;
			//SET_SAMPLING_TIME d=t_ant
			rfid_tx_rx_byte((0x80 | (uint8_t)t_ant), 0);
			//GET_SAMPLING_TIME
			response = rfid_tx_rx_byte(0x02, 1);
			if(response != t_ant)
			{
				//Error - the sampling time value was not accepted for some reason
				rfid_state = RFID_READ_FAIL;
				break;
			}
			//HTRC110 is now outputting a 125kHz sine wave.  The tag (if present) will be powered by this and it will then start transmitting its 64 bits as a continuous
			//cycle (it transmits in a loop as long as its powered by the RF field).  It transmits its bits by interfearing with the RF sine wave signal which the HTRC110
			//detects for us.  When we now start reading the tag data we will be given the raw data output of the tag after being filtered into a digital signal by the HTRC110.
			//----- START READING THE TAG DATA -----
			rfid_start_read_tag();
			rfid_1ms_timer = 150;					//Set a timeout
			rfid_state = RFID_READ_TAG_READING;
			//----- ENABLE THE MISO RISING EDGE INTERRUPT -----
			rfid_capture_state = RFID_CAPTURE_INITIALISE;
			rfid_tries_count = 0;
			//The interrupt called function deals with reading and demodulating the tag data
			break;

		case RFID_READ_TAG_READING:
			//----------------------------------------
			//----------------------------------------
			//----- READING TAG DEMODULATED DATA -----
			//----------------------------------------
			//----------------------------------------
			//The tag data length is 64 bits with between 64 to 127 bits needing to be received before we have a complete sequence.
			//We also need to initially receive 22 bits to enable us to determin the bit rate.
			//Therefore read can take 149 bits x 512uS maximum bit rate = 76mS.
			//Tags will typically be set to use the slowest 512uS bit rate as slow transitions on the air interface are easier to
			//detect, which means you get additional range and more reliable communications.
			//Note that if no tag is present we expect to receive bad data (the data line will not be idle) and we are likely to see
			//valid header strings.  However the 14 seperate parity checks will only pass when a valid tag is present.

			if((rfid_1ms_timer == 0) || (rfid_capture_state == RFID_CAPTURE_FAILED_GIVEN_UP))
			{
				//------------------------------
				//----- TAG READING FAILED -----
				//------------------------------
				rfid_end_read_tag();
				rfid_read_tag_success = 0;
				rfid_read_tag_pending = 0;
				rfid_state = RFID_SET_IDLE;
			}
			break;

		case RFID_READ_FAIL:
			//---------------------------
			//---------------------------
			//----- READ TAG FAILED -----
			//---------------------------
			//---------------------------
			rfid_read_tag_success = 0;
			rfid_read_tag_pending = 0;
			rfid_state = RFID_SET_IDLE;
			break;
	} //switch (rfid_state)
}

//**********************************
//**********************************
//********** TX / RX BYTE **********
//**********************************
//**********************************
uint8_t rfid_tx_rx_byte(uint8_t tx_data, uint8_t get_response)
{
	uint8_t rx_data = 0;

	RFID_SCLK(0);
	RFID_BIT_DELAY();			//Ensure the low state is long enough
	//INITIALISE THE SERIAL INTERFACE
	//(Low to high transition of DIN while SCLK is high marks the start of a transfer)
	RFID_MOSI(0);
	RFID_SETUP_DELAY();
	RFID_SCLK(1);
	RFID_SETUP_DELAY();
	RFID_MOSI(1);
	RFID_BIT_DELAY();
	//BIT 7
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	if(tx_data & 0x80)
		RFID_MOSI(1);
	else
		RFID_MOSI(0);
	RFID_SCLK(1);		//Data in valid in high state
	RFID_BIT_DELAY();
	//BIT 6
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	if(tx_data & 0x40)
		RFID_MOSI(1);
	else
		RFID_MOSI(0);
	RFID_SCLK(1);
	RFID_BIT_DELAY();
	//BIT 5
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	if(tx_data & 0x20)
		RFID_MOSI(1);
	else
		RFID_MOSI(0);
	RFID_SCLK(1);
	RFID_BIT_DELAY();
	//BIT 4
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	if(tx_data & 0x10)
		RFID_MOSI(1);
	else
		RFID_MOSI(0);
	RFID_SCLK(1);
	RFID_BIT_DELAY();
	//BIT 3
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	if(tx_data & 0x08)
		RFID_MOSI(1);
	else
		RFID_MOSI(0);
	RFID_SCLK(1);
	RFID_BIT_DELAY();
	//BIT 2
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	if(tx_data & 0x04)
		RFID_MOSI(1);
	else
		RFID_MOSI(0);
	RFID_SCLK(1);
	RFID_BIT_DELAY();

	//BIT 1
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	if(tx_data & 0x02)
		RFID_MOSI(1);
	else
		RFID_MOSI(0);
	RFID_SCLK(1);
	RFID_BIT_DELAY();
	//BIT 0
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	if(tx_data & 0x01)
		RFID_MOSI(1);
	else
		RFID_MOSI(0);
	RFID_SCLK(1);
	RFID_BIT_DELAY();


	if(get_response)
	{
		//BIT 7
		RFID_SCLK(0);
		RFID_BIT_DELAY();
		RFID_SCLK(1);		//Data out valid in high state
		RFID_BIT_DELAY();
		if(RFID_MISO)
			rx_data |= 0x80;
		//BIT 6
		RFID_SCLK(0);
		RFID_BIT_DELAY();
		RFID_SCLK(1);
		RFID_BIT_DELAY();
		if(RFID_MISO)
			rx_data |= 0x40;
		//BIT 5
		RFID_SCLK(0);
		RFID_BIT_DELAY();
		RFID_SCLK(1);
		RFID_BIT_DELAY();
		if(RFID_MISO)
			rx_data |= 0x20;
		//BIT 4
		RFID_SCLK(0);
		RFID_BIT_DELAY();
		RFID_SCLK(1);
		RFID_BIT_DELAY();
		if(RFID_MISO)
			rx_data |= 0x10;
		//BIT 3
		RFID_SCLK(0);
		RFID_BIT_DELAY();
		RFID_SCLK(1);
		RFID_BIT_DELAY();
		if(RFID_MISO)
			rx_data |= 0x08;
		//BIT 2
		RFID_SCLK(0);
		RFID_BIT_DELAY();
		RFID_SCLK(1);
		RFID_BIT_DELAY();
		if(RFID_MISO)
			rx_data |= 0x04;
		//BIT 1
		RFID_SCLK(0);
		RFID_BIT_DELAY();
		RFID_SCLK(1);
		RFID_BIT_DELAY();
		if(RFID_MISO)
			rx_data |= 0x02;
		//BIT 0
		RFID_SCLK(0);
		RFID_BIT_DELAY();
		RFID_SCLK(1);
		RFID_BIT_DELAY();
		if(RFID_MISO)
			rx_data |= 0x01;
	}

	RFID_SCLK(0);
	return(rx_data);
}



//************************************
//************************************
//********** START READ_TAG **********
//************************************
//************************************
void rfid_start_read_tag(void)
{

	RFID_SCLK(0);
	RFID_BIT_DELAY();			//Ensure the low state is long enough
	//INITIALISE THE SERIAL INTERFACE
	//(Low to high transition of DIN while SCLK is high marks the start of a transfer)
	RFID_MOSI(0);
	RFID_SETUP_DELAY();
	RFID_SCLK(1);
	RFID_SETUP_DELAY();
	RFID_MOSI(1);
	RFID_BIT_DELAY();
	//BIT 7
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	RFID_MOSI(1);
	RFID_SCLK(1);		//Data in valid in high state
	RFID_BIT_DELAY();
	//BIT 6
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	RFID_MOSI(1);
	RFID_SCLK(1);
	RFID_BIT_DELAY();
	//BIT 5
	RFID_SCLK(0);
	RFID_BIT_DELAY();
	RFID_MOSI(1);
	RFID_SCLK(1);
	RFID_BIT_DELAY();
	RFID_SCLK(0);
	RFID_MOSI(0);
	//THIS HTRC110 NOW IMMEDIATELY SWITCHES TO TRANSPARENT MODE.  THE TRASNPONDER DATA IS DIRECTLY PRESENTED AT DOUT.
	//Delay from RX to DOUT is dependant on demodulator settings.
}



//**********************************
//**********************************
//********** END READ_TAG **********
//**********************************
//**********************************
void rfid_end_read_tag(void)
{
	//----- TERMIATE READ_TAG MODE -----
	//Low to high transition on SCLK
	RFID_SCLK(1);
}
