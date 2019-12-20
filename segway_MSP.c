/******************************************************************************
MSP430F2272 Project Creator 4.0

ME 461 - S. R. Platt
Fall 2010

Updated for CCSv4.2 Rick Rekoske 8/10/2011

Written by: Steve Keres
College of Engineering Control Systems Lab
University of Illinois at Urbana-Champaign
*******************************************************************************/

#include "msp430x22x2.h"
#include "UART.h"

char newprint = 0;
unsigned long timecnt = 0;
unsigned char RXData[8] = {0,0,0,0,0,0,0,0};
unsigned char TXData[8] = {0,0,0,0,0,0,0,0};
int I2Cindex = 0;
int rx1=0,rx2=0,rx3=0,rx4=0;
long tx1=0,tx2=0,tx3=0,tx4=0;
// turn info vars
int turn_old = 0;
char left = 0;
char right = 0;
char stop = 0;


void main(void) {
	
	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	
	if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);
	                                             
	DCOCTL = CALDCO_16MHZ;                		// Set uC to run at approximately 16 Mhz
	BCSCTL1 = CALBC1_16MHZ; 
		
    P4DIR |= 0xC0;
    P4SEL &= ~0xC0;
    TACCTL0 = CCIE;

    // Timer A Config
    TACCTL0 = CCIE;              // Enable Timer A interrupt
    TACCR0  = 16000;             // period = 1ms
    TACTL   = TASSEL_2 + MC_1;   // source SMCLK, up mode
    TACTL = TASSEL_2 + MC_1 + ID_0;   // source SMCLK, up mode

	//P1IN 				Port 1 register used to read Port 1 pins setup as Inputs
	P1SEL &= ~0xFF; // Set all Port 1 pins to Port I/O function
	P1REN &= ~0xFF; // Disable internal resistor for all Port 1 pins
	P1DIR |= 0x3;  // Set Port 1 Pin 0 and Pin 1 (P1.0,P1.1) as an output.  Leaves Port1 pin 2 through 7 unchanged
	P1OUT &= ~0xFF;  // Initially set all Port 1 pins set as Outputs to zero
	P1IFG &= ~0xFF;  // Clear Port 1 interrupt flags
	P1IES &= ~0xFF; // If port interrupts are enabled a High to Low transition on a Port pin will cause an interrupt 
	P1IE &= ~0xFF; // Disable all port interrupts

	//Add your code here to initialize USCIB0 as an I2C slave
	//Output for LED
	P3SEL &= ~BIT0;
	P3REN &= ~BIT0;
	P3DIR |= BIT0;
	P3OUT &= ~BIT0;

    P3SEL |= BIT1 + BIT2;          // Assign I2C pins to USCI_B0

	UCB0CTL1 = UCSSEL_2 + UCSWRST;           // source SMCLK, hold USCIB0 in reset

    UCB0CTL0 = UCMODE_3 + UCSYNC; // I2C Slave, synchronous mode
    UCB0I2COA = 0x25;             // Own Address is 0x25
    UCB0CTL1 &= ~UCSWRST;         // Clear SW reset, resume operation
    IE2 |= UCB0RXIE;              // Enable RX interrupt only.  TX will be enabled after reception of an I2C RX

	ADC10CTL0 = SREF_0 + ADC10SHT_1 + ADC10ON + ADC10IE;
	ADC10CTL1 = INCH_0 + CONSEQ_0 + ADC10SSEL_0 + SHS_0;
	ADC10AE0 = 0x1;


	// Timer A Config
	TACCTL0 = CCIE;                           // Enable Timer A interrupt
	TACCR0 = 16000;                           // period = 1ms   
	TACTL = TASSEL_2 + MC_1;                  // source SMCLK, up mode
	
	
	Init_UART(9600,1);	// Initialize UART for 9600 baud serial communication

	_BIS_SR(GIE); 	// Enable global interrupt


	while(1) {

		if(newmsg) {
			//my_scanf(rxbuff,&var1,&var2,&var3,&var4);
			newmsg = 0;
		}

		if (newprint)  { 
            //UART_printf("Hello %d Hi again. %x\n\r ",timecnt, P1OUT); //  %d int, %ld long, %c char, %x hex form, %.3f float 3 decimal place, %s null terminated character array

		    //UART_printf("R:yourRXdata\n\r");
		    // UART_send(1,(float)timecnt/500);

		    if (left) {
		        P4OUT ^= 0x80; // Blink LED
		    }
		    if (right) {
		        P4OUT ^= 0x40; // Blink LED
		    }
		    if (stop){
		        P4OUT &= ~0xC0;
		    }

		    newprint = 0;
		    UART_printf("RX1&2: %d, %d\n\r ",rx1, rx2); //  %d int, %ld long, %c char, %x hex form, %.3f float 3 decimal place, %s null
		}


	}
}


// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
	timecnt++; // Keep track of time for main while loop. 

	if ((timecnt%500) == 0) {
		newprint = 1;  // flag main while loop that .5 seconds have gone by.
	}
  
}



// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {


	tx1 = ADC10MEM;
	tx2 = rx1;
	tx3 = tx1;
	tx4 = tx1;

	TXData[0] = tx1>>8;
	TXData[1] = tx1;
	TXData[2] = tx2>>8;
	TXData[3] = tx2;
	TXData[4] = tx3>>8;
	TXData[5] = tx3;
	TXData[6] = tx4>>8;
	TXData[7] = tx4;
	
	P3OUT ^= BIT0;
	IE2 &= ~UCB0RXIE;
	IE2 |= UCB0TXIE;


}



// USCI Transmit ISR - Called when TXBUF is empty (ready to accept another character)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {
  
	if((IFG2&UCA0TXIFG) && (IE2&UCA0TXIE)) {		// USCI_A0 requested TX interrupt
		if(printf_flag) {
			if (currentindex == txcount) {
				senddone = 1;
				printf_flag = 0;
				IFG2 &= ~UCA0TXIFG;
			} else {
			UCA0TXBUF = printbuff[currentindex];
			currentindex++;
			}
		} else if(UART_flag) {
			if(!donesending) {
				UCA0TXBUF = txbuff[txindex];
				if(txbuff[txindex] == 255) {
					donesending = 1;
					txindex = 0;
				} else {
					txindex++;
				}
			}
		}
	    
		IFG2 &= ~UCA0TXIFG;
	}

	if((IFG2&UCB0RXIFG) && (IE2&UCB0RXIE)) {	// USCI_B0 RX interrupt occurs here for I2C
		RXData[I2Cindex] = UCB0RXBUF;           // Read RX buffer

		I2Cindex++;
		if (I2Cindex == 8) {
			rx1 = (((int)RXData[0])<<8) + ((int)RXData[1]);
			rx2 = (((int)RXData[2])<<8) + ((int)RXData[3]);
			rx3 = (((int)RXData[4])<<8) + ((int)RXData[5]);
			rx4 = (((int)RXData[6])<<8) + ((int)RXData[7]);

			// check turning
			if (rx1>turn_old){
			    left = 1;
			    right = 0;
			    stop = 0;
			}
			else if(rx1<turn_old){
			    right = 1;
			    left = 0;
			    stop = 0;
			}
			else if(rx1==turn_old){
			    stop = 1;
			    left = 0;
			    right = 0;
			}
			turn_old = rx1;


			I2Cindex = 0;
			IE2 &= ~UCB0RXIE;
			ADC10CTL0 |= ENC + ADC10SC;
		}

	} else if ((IFG2&UCB0TXIFG) && (IE2&UCB0TXIE)) { // USCI_B0 TX interrupt
		UCB0TXBUF = TXData[I2Cindex];
		I2Cindex++;
		if (I2Cindex == 8) {
			I2Cindex=0;
			IE2 &= ~UCB0TXIE;
			IE2 |= UCB0RXIE;
		}
	}
}


// USCI Receive ISR - Called when shift register has been transferred to RXBUF
// Indicates completion of TX/RX operation
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
  
	if((IFG2&UCA0RXIFG) && (IE2&UCA0RXIE)) {  // USCI_A0 requested RX interrupt (UCA0RXBUF is full)
  	
		if(!started) {	// Haven't started a message yet
			if(UCA0RXBUF == 253) {
				started = 1;
				newmsg = 0;
			}
		} else {	// In process of receiving a message		
			if((UCA0RXBUF != 255) && (msgindex < (MAX_NUM_FLOATS*5))) {
				rxbuff[msgindex] = UCA0RXBUF;
				
				msgindex++;
			} else {	// Stop char received or too much data received
				if(UCA0RXBUF == 255) {	// Message completed
					newmsg = 1;
					rxbuff[msgindex] = 255;	// "Null"-terminate the array
				}
				started = 0;
				msgindex = 0;
			}
		}
		IFG2 &= ~UCA0RXIFG;
	}

	if((UCB0I2CIE&UCNACKIE) && (UCB0STAT&UCNACKIFG)) { // I2C NACK interrupt

		UCB0STAT &= ~UCNACKIFG;
	}
	if((UCB0I2CIE&UCSTPIE) && (UCB0STAT&UCSTPIFG)) { // I2C Stop interrupt

		UCB0STAT &= ~UCSTPIFG;
	}
	if((UCB0I2CIE&UCSTTIE) && (UCB0STAT&UCSTTIFG)) { //  I2C Start interrupt

		UCB0STAT &= ~UCSTTIFG;
	}
	if((UCB0I2CIE&UCALIE) && (UCB0STAT&UCALIFG)) {  // I2C Arbitration Lost interrupt

		UCB0STAT &= ~UCALIFG;
	}
}



