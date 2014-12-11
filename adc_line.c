/*
 * Copyright (c) 2010, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * ADC + DMA module for line sensor
 *
 * by camrose
 *
 * v.1.0
 *
 * Revisions:
 *  Cameron Rose        2014-12-10      Initial release
 *
 * Notes:
 *   >>>> THIS IS IMAGEPROC2.5 SPECIFIC CODE <<<<
 *  - This module configured ADC1 to scan through AN0, AN8, AN9, AN10, and AN11
 *      sequentially, to read the battery and 4 motor BEMF's.
 *  - DMA is used.
 *  - Values are stored locally, getter functions are provided.
 */

#include "adc.h"
#include "adc_line.h"
#include "p33Fxxxx.h"
#include "ports.h"
#include "utils.h"


//Functions
static void adcLineSetupPeripheral(void);
//DMA related functions
static void initDma6(void);
void __attribute__((__interrupt__)) _DMA6Interrupt(void);


//Variables to store values as they come out of the DMA buffer
static unsigned int adc_line;

void adcLineSetup(void){
	adcLineSetupPeripheral();
	initDma6(); //DMA is needed to read multiple values from the ADC core
}

static void adcLineSetupPeripheral(void) {
    // ADC 2 conig for line sensor (Ignore other comments, needs to be edited

    AD2CON1bits.ADON = 0;       //disable
    AD2CON1bits.ADSIDL = 0;     //continue in idle mode
    AD2CON1bits.AD12B = 1;      //10 bit mode
    AD2CON1bits.FORM = 0b00;    //integer (0000 dddd dddd dddd) format output
    AD2CON1bits.SSRC = 0b111;   //Sample clock source based on PWM
    AD2CON1bits.SIMSAM = 1;     //Sample channels simultaneously
    AD2CON1bits.ASAM = 1;       //Auto sampling on
    AD2CON1bits.ADDMABM = 1;

    AD2CON2bits.VCFG = 0b000;   //Vdd is pos. ref and Vss is neg. ref.
    AD2CON2bits.CSCNA = 1;      //scan inputs
    AD2CON2bits.CHPS = 0b00;    //Convert channels 0 and 1
    AD2CON2bits.SMPI = 0b0000;  //Interrupt after 3 conversions (depends on CHPS and SIMSAM)
    AD2CON2bits.BUFM = 0;       //Always fill conversion buffer from first element
    AD2CON2bits.ALTS = 0;       //Alternate MUXes for analog input selection

    AD2CON3bits.ADRC = 0;       //Derive conversion clock from system clock
//    AD1CON3bits.SAMC = 0b00001; //Auto sampling clock period is one Tad
    AD2CON3bits.ADCS = 0b00000001; // Each TAD is 3 Tcy

    AD2PCFGL = 0xFFDF;          //Enable AN0 - AN3 as analog inputs

    AD2CHS0bits.CH0SA = 0b01011;      //Select AN1 for CH0 +ve input
    AD2CHS0bits.CH0NA = 0;      //Select Vref- for CH0 -ve input

    AD2CSSL = 0x0020;

    //AD1CHS123bits.CH123SA = 0b1;  //Select AN3 for CH1 +ve input
    //AD1CHS123bits.CH123NA = 0b00;  //Select Vref- for CH1 -ve input

    AD2CON1bits.ADON = 1;       //enable

    IFS1bits.AD2IF = 0; // Clear the A/D interrupt flag bit
    IEC1bits.AD2IE = 0; //Disable A/D interrupt

}

unsigned int adcGetLine(){
    return adc_line;
}

//////////////////////////////////////////////////////////////////////
///////////////      DMA Section     /////////////////////////////////
//////////////////////////////////////////////////////////////////////

#define  SAMP_BUFF_SIZE	 		1		// Size of the input buffer per analog input

//Buffers need special attribute to be in DMA memory space
static int  BufferC[1][SAMP_BUFF_SIZE] __attribute__((space(dma)));

static unsigned int DmaBuffer = 0;


/*****************************************************************************
 * Function Name : initDma0
 * Description   : Setup function for DMA0, to read ADC1 into a buffer
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
static void initDma6(void) {
        DMA6CONbits.AMODE = 0;			// Configure DMA for Register Indirect w/ post-increment
	DMA6CONbits.MODE  = 0;			// Configure DMA for Continuous Ping-Pong mode

	DMA6PAD=(int)&ADC2BUF0;
	//DMA0CNT = (SAMP_BUFF_SIZE*2)-1;
	DMA6CNT = 0;  //See dsPIC user's manual. 3 analog reads -> DMA0CNT = 3-1 = 2
	//DMA0CNT = 7;

	DMA6REQ=21; //ADC2 requests

	DMA6STA = __builtin_dmaoffset(BufferC);
	//DMA0STB = __builtin_dmaoffset(BufferB);

	IFS4bits.DMA6IF = 0;			//Clear the DMA interrupt flag bit
        IEC4bits.DMA6IE = 1;			//Set the DMA interrupt enable bit

	DMA6CONbits.CHEN=1;
}
/*****************************************************************************
 * Function Name : _DMA0Interrupt
 * Description   : Interrupt hander for DMA0 , associated with ADC1 here.
                                  Motor BEMF vales are set through setter functions.
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA6Interrupt(void) {
    adc_line = BufferC[0][0];
    IFS4bits.DMA6IF = 0;
}
// End DMA section
