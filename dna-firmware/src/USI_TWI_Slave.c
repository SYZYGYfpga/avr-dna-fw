// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* File              : USI_TWI_Slave.c
* Compiler          : IAR EWAAVR 4.11A
* Revision          : $Revision: 1.14 $
* Date              : $Date: Friday, December 09, 2005 17:25:38 UTC $
* Updated by        : $Author: jtyssoe $
*
* Support mail      : avr@atmel.com
*
* Supported devices : All device with USI module can be used.
*
* AppNote           : AVR312 - Using the USI module as a I2C slave
*
* Description       : Functions for USI_TWI_receiver and USI_TWI_transmitter.
*
****************************************************************************
*
* Modified by Opal Kelly Incorporated for the following:
*   - Add support for building with GCC
*   - Add ability to detect if a new I2C transaction has taken place
*   - Add functions to gain visibility into the TWI data buffer counts
*   - General bugfixes
*
****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "USI_TWI_Slave.h"

/*! Static Variables
 */

static unsigned char TWI_slaveAddress;
static volatile unsigned char USI_TWI_Overflow_State;


/*! Local variables
 */
static uint8_t TWI_RxBuf[TWI_RX_BUFFER_SIZE];
static volatile uint8_t TWI_RxHead;
static volatile uint8_t TWI_RxTail;

static uint8_t TWI_TxBuf[TWI_TX_BUFFER_SIZE];
static volatile uint8_t TWI_TxHead;
static volatile uint8_t TWI_TxTail;

static unsigned char TWI_newTx; // boolean value indicating that a new transaction has begun

/*! \brief Flushes the TWI buffers
 */
void Flush_TWI_Buffers(void)
{
	TWI_RxTail = 0;
	TWI_RxHead = 0;
	TWI_TxTail = 0;
	TWI_TxHead = 0;
}

//********** USI_TWI functions **********//

/*! \brief
 * Initialise USI for TWI Slave mode.
 */
void USI_TWI_Slave_Initialise( unsigned char TWI_ownAddress )
{
	Flush_TWI_Buffers();
	
	TWI_slaveAddress = TWI_ownAddress;
	
	PORT_USI |=  (1<<PORT_USI_SCL);                                 // Set SCL high
	PORT_USI |=  (1<<PORT_USI_SDA);                                 // Set SDA high
	DDR_USI  |=  (1<<PORT_USI_SCL);                                 // Set SCL as output
	DDR_USI  &= ~(1<<PORT_USI_SDA);                                 // Set SDA as input
	USICR    =  (1<<USISIE)|(0<<USIOIE)|                            // Enable Start Condition Interrupt. Disable Overflow Interrupt.
	            (1<<USIWM1)|(0<<USIWM0)|                            // Set USI in Two-wire mode. No USI Counter overflow prior
	                                                                // to first Start Condition (potentail failure)
	            (1<<USICS1)|(0<<USICS0)|(0<<USICLK)|                // Shift Register Clock Source = External, positive edge
	            (0<<USITC);
	USISR    = 0xF0;                                                // Clear all flags and reset overflow counter
}


/*! \brief Puts data in the transmission buffer.
           Always check if there is room in the buffer before using this function.
*/
void USI_TWI_Transmit_Byte( unsigned char data )
{
	if (USI_TWI_Data_In_Transmit_Buffer() == TWI_TX_BUFFER_SIZE - 1) {
		return; // buffer full
	}
	
	TWI_TxBuf[TWI_TxHead] = data;          // Store data in buffer.
	TWI_TxHead = ( TWI_TxHead + 1 );       // Calculate buffer index.
	
	if (TWI_TxHead >= TWI_TX_BUFFER_SIZE) {
		TWI_TxHead = 0;
	}
}

/*! \brief Returns a byte from the receive buffer, return 0xFF if full.
           Always check if there is data available before using this function.
 */
char USI_TWI_Receive_Byte( )
{
	char temp_data;
	
	if ( USI_TWI_Data_In_Receive_Buffer() == 0 ) {
		return 0xFF; // buffer empty
	}
	temp_data = TWI_RxBuf[TWI_RxTail];       // Return data from the buffer.
	TWI_RxTail = ( TWI_RxTail + 1 );    // Calculate buffer index
	
	if (TWI_RxTail >= TWI_RX_BUFFER_SIZE) {
		TWI_RxTail = 0;
	}
	
	return temp_data;
}

/*! \brief Returns the amount of data in the Transmit Buffer
 */
char USI_TWI_Data_In_Transmit_Buffer( void )
{
	if (TWI_TxHead < TWI_TxTail) {
		return TWI_TX_BUFFER_SIZE + TWI_TxHead - TWI_TxTail;
	}
	return TWI_TxHead - TWI_TxTail;
}

/*! \brief Returns the amount of data in the Receive Buffer
 */
char USI_TWI_Data_In_Receive_Buffer( void )
{
	if (TWI_RxHead < TWI_RxTail) {
		return TWI_RX_BUFFER_SIZE + TWI_RxHead - TWI_RxTail;
	}
	return TWI_RxHead - TWI_RxTail;
}

void USI_TWI_Set_New_Tx( void )
{
	// Clear stop condition flag, we'll wait for this during writes to queue them all up
	USISR = (0 << USI_START_COND_INT) | (0 << USIOIF) | (1 << USIPF) | (0 << USIDC) | (USISR & 0xF);
	TWI_newTx = 1;
}

void USI_TWI_Clear_New_Tx( void )
{
	TWI_newTx = 0;
}

char USI_TWI_Is_New_Tx( void )
{
	return TWI_newTx;
}

/*! \brief Usi start condition ISR
 * Detects the USI_TWI Start Condition and intialises the USI
 * for reception of the "TWI Address" packet.
 */
ISR(USI_START_VECTOR)
{
	// Set default starting conditions for new TWI package
	USI_TWI_Overflow_State = USI_SLAVE_CHECK_ADDRESS;
	//DDR_USI  &= ~(1<<PORT_USI_SDA);                                 // Set SDA as input
	while ( (PIN_USI & (1<<PORT_USI_SCL)) ) {   // Wait for SCL to go low to ensure the "Start Condition" has completed.
		if (PIN_USI & (1 << PORT_USI_SDA)) {    // don't wait forever if SDA goes back high...
			SET_USI_TO_TWI_START_CONDITION_MODE();
			return;
		}
	}
	

	USICR   =   (1<<USISIE)|(1<<USIOIE)|                            // Enable Overflow and Start Condition Interrupt. (Keep StartCondInt to detect RESTART)
	            (1<<USIWM1)|(1<<USIWM0)|                            // Set USI in Two-wire mode.
	            (1<<USICS1)|(0<<USICS0)|(0<<USICLK)|                // Shift Register Clock Source = External, positive edge
	            (0<<USITC);
	USISR  =    (1<<USI_START_COND_INT)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Clear flags
	            (0x0<<USICNT0);                                     // Set USI to sample 8 bits i.e. count 16 external pin toggles.
}


/*! \brief USI counter overflow ISR
 * Handels all the comunication. Is disabled only when waiting
 * for new Start Condition.
 */
ISR(USI_OVERFLOW_VECTOR) 
{
	unsigned char tmpUSIDR;

	switch (USI_TWI_Overflow_State)
	{
    // ---------- Address mode ----------
    // Check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, else reset USI.
	case USI_SLAVE_CHECK_ADDRESS:
		if ((USIDR == 0) || (( USIDR>>1 ) == TWI_slaveAddress)) {
			if ( (USIDR & 0x01) && (USI_TWI_Data_In_Transmit_Buffer() > 0)) {
				USI_TWI_Overflow_State = USI_SLAVE_SEND_DATA;
				SET_USI_TO_SEND_ACK();
			} else if (!(USIDR & 0x01) && ((USI_TWI_Data_In_Receive_Buffer() == 0) || USI_TWI_Is_New_Tx())) {
				// This is a write and we have cleared the last written data
				// or the previous transaction was incomplete (TWI_newTx was 
				// not cleared).
				USI_TWI_Overflow_State = USI_SLAVE_REQUEST_DATA;
				USI_TWI_Set_New_Tx();
				SET_USI_TO_SEND_ACK();
				Flush_TWI_Buffers();
			} else {
				SET_USI_TO_TWI_START_CONDITION_MODE();
			}
		} else {
		  SET_USI_TO_TWI_START_CONDITION_MODE();
		}
	break;

	// ----- Master read data mode ------
	// Check reply and goto USI_SLAVE_SEND_DATA if OK, else reset USI.
	case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
		if ( USIDR ) // If NACK, the master does not want more data.
		{
		  SET_USI_TO_TWI_START_CONDITION_MODE();
		  return;
		}
		// From here we just drop straight into USI_SLAVE_SEND_DATA if the master sent an ACK

	// Copy data from buffer to USIDR and set USI to shift byte. Next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
	case USI_SLAVE_SEND_DATA:
		// Get data from Buffer
		USIDR = TWI_TxBuf[TWI_TxTail];
		TWI_TxTail = ( TWI_TxTail + 1 );
		if (TWI_TxTail >= TWI_TX_BUFFER_SIZE) {
			TWI_TxTail = 0;
		}
		USI_TWI_Overflow_State = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
		SET_USI_TO_SEND_DATA();
		break;

	// Set USI to sample reply from master. Next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
	case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
		USI_TWI_Overflow_State = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
		SET_USI_TO_READ_ACK();
		break;

	// ----- Master write data mode ------
	// Set USI to sample data from master. Next USI_SLAVE_GET_DATA_AND_SEND_ACK.
	case USI_SLAVE_REQUEST_DATA:
		USI_TWI_Overflow_State = USI_SLAVE_GET_DATA_AND_SEND_ACK;
		SET_USI_TO_READ_DATA();
		break;

	// Copy data from USIDR and send ACK. Next USI_SLAVE_REQUEST_DATA
	case USI_SLAVE_GET_DATA_AND_SEND_ACK:
		// Put data into Buffer
		tmpUSIDR = USIDR;             // Not necessary, but prevents warnings
		TWI_RxBuf[TWI_RxHead] = tmpUSIDR;
		TWI_RxHead = ( TWI_RxHead + 1 );
		if (TWI_RxHead >= TWI_RX_BUFFER_SIZE) {
			TWI_RxHead = 0;
		}
		
		USI_TWI_Overflow_State = USI_SLAVE_REQUEST_DATA;
		SET_USI_TO_SEND_ACK();
		break;
	}
}
