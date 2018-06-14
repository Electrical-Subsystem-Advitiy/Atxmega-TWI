/*
 * I2C_INTERRUPR_XMEGA.c
 *
 * Created: 13-05-2018 19:14:31
 * Author : PRASHANT KURREY
 */ 

#include <avr/io.h>



/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA TWI driver example source.
 *
 *      This file contains an example application that demonstrates the TWI
 *      master and slave driver. It shows how to set up one TWI module as both
 *      master and slave, and communicate with itself.
 *
 *      The recommended test setup for this application is to connect 10K
 *      pull-up resistors on PC0 (SDA) and PC1 (SCL). Connect a 10-pin cable
 *      between the PORTD and SWITCHES, and PORTE and LEDS.
 */
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
                                                                                   //Vai -Include UART.h when to use UART
/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0x55

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        8

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED       2000000
#define BAUDRATE	9600
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)


/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
TWI_Slave_t twiSlave;      /*!< TWI slave module. */


/*! Buffer with test data to send.*/
uint8_t sendBuffer[NUM_BYTES] = {'P','R','A', 'S','H','A', 'N','T'};                          //Bytes sent to slave by master in TWI_MasterReadWrite


/*! Simple function that invert the received value in the sendbuffer. This
 *  function is used in the driver and passed on as a pointer to the driver.
 */

void TWIC_SlaveData(void)
{    uint8_t   Data[6] = {'K','U','R', 'R', 'E', 'Y'} ;  
	int I=0;//uint8_t bufIndex = twiSlave.bytesReceived;                                     //Vai- would be sent by slave when called by master(i.e saved in slave)
while( I<6){ twiSlave.sendData[I] = Data[I] ;
	I++;  }         //
}


/*! /brief Example code
 *
 *  Example code that reads the key pressed and show a value from the buffer,
 *  sends the value to the slave and read back the processed value which will
 *  be inverted and displayed after key release.
 */
int main(void)
{                                                  //Add UART initial setting via UART.c to implement UART
	
	
	
	// Enable internal pull-up on PC0, PC1.. Uncomment if you don't have external pullups
	PORTCFG.MPCMASK = 0x03; // Configure several PINxCTRL registers at the same time
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc; //Enable pull-up to get a defined level on the switches

	

	

	/* Initialize TWI slave. */
	TWI_SlaveInitializeDriver(&twiSlave, &TWIC, TWIC_SlaveData);
	TWI_SlaveInitializeModule(&twiSlave,
	                          SLAVE_ADDRESS,                                                  //First code Slave part later code master part. Comment it when burning code for master
	                          TWI_SLAVE_INTLVL_LO_gc);

	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();
uint8_t BufPos = 0;

	
while(true){
/*	TWI_MasterInit(&twiMaster,
	&TWIC,                                                                      //Initialize master
	TWI_MASTER_INTLVL_LO_gc,
	TWI_BAUDSETTING);
	
	TWI_MasterWriteRead(&twiMaster,                                      //added by me for writing PRASHANT to slave and Receiving KURREY from slave
	SLAVE_ADDRESS,
	&sendBuffer[BufPos],
	8,
	6);
	


	while (twiMaster.status != TWIM_STATUS_READY) {
		// Wait until transaction is complete.
	}*/

	}
	//_delay_ms(500);
	};
		


/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);                                   //Vaibhav- called when master gets something from Slave
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
	
}
                                                                          //Add UART interrupt when using UART



