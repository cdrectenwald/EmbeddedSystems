/*
 * File:   UART.h
 * Author: Chris Rectewnald
 * Purpose:  Header file to configure and start UART module.  Also
 * includes strings to be printed to UART upon state update / events.
 */


#define BAUDRATE 19200
#define FCY 7370000/2

void configPins(void);
void configUART1(void);
void outChar1(char c);
void my_puts(char *str);

void my_puts(char *str) {       // my version of puts to print strings to UART
    outChar1('\n');
    outChar1('\r');
    while (*str) {              // if we haven't reached null, keep printing
        outChar1(*str);         // print current character
        str++;                  // increment pointer to next char
    }
}

char *state_update[] = {        // array of pointers to char for string display
    "Screen Update\r", "Two-Button Press\r",
    "Button UP\r", "Button DOWN\r", "Temp Too High\r",
    "Temp Too Low\r", "Settings Updated\r", "Enter Max\r",
    "Enter Min\r", "Leaving User Mode...\r", "Entering User Mode!\r",
    "Invalid!\r", "Temp = "
};

void configPins(void) {
    RPINR18bits.U1RXR = 20;     // map UART1 RX to remappable pin RP20 (pin 12)
    RPOR1bits.RP36R = 1;        // map UART1 TX to remappable pin RP36 (pin 11)          
}

void outChar1(char c){
    while(U1STAbits.UTXBF);     // wait for transmit buffer to be empty
    U1TXREG = c;                // write character to transmit register
}

void configUART1(void) {
    /*  Calculate the Baud Rate generator value     */
    int brg = (FCY/BAUDRATE/4)-1;
    U1MODEbits.BRGH = 1;        // High speed mode
    U1BRG = brg;                // store the value in the register
    
    U1MODEbits.PDSEL = 0;       // 8 bit data, no parity
    U1MODEbits.STSEL = 0;       // 1 stop bit
    
    U1MODEbits.UEN = 0b00;      // UxTX and UxRX pins are enabled and used;
                                // also, no flow control pins
    U1MODEbits.UARTEN = 1;      // enable UART RX/TX
    U1STAbits.UTXEN = 1;        //Enable the transmitter
 
}
© 2019 GitHub, Inc.