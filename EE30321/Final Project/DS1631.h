/*
 * DS1631.h
 * Chris Rectenwald
 * Purpose:  Header file to facilitate easy operation of a DS1631 temperature
 * sensor via I2C on the PIC24 microcontroller.
 *
 * 
 */

#define ZERO 48
#define NINE 58

#define ADDRESS_READ 0x91
#define ADDRESS_WRITE 0x90
#define START_CONVERT 0x51
#define READ_TEMP 0xAA
#define ACCESS_CONFIG 0xAC
#define CONFIG_DATA 0x0C

void startI2C1(void);
int putI2C1(char data);
void restartI2C1(void);
void stopI2C1(void);
void startConvertDS1631(void);
void configI2C1(void);
void configDS1631(void);
unsigned int readTempDS1631(void);
char getI2C1(char ack2send);


char getI2C1(char ack2send) {
    char inByte;
    
    while(I2C1CON & 0x1F);
    I2C1CONbits.RCEN = 1;
    while(!I2C1STATbits.RBF);
    inByte = I2C1RCV;
    
    while(I2C1CON & 0x1F);
    I2C1CONbits.ACKDT = ack2send;
    I2C1CONbits.ACKEN = 1;
    while(I2C1CONbits.ACKEN);
    
    return(inByte);
}

unsigned int readTempDS1631(void) {
    unsigned char temp_hi = 0;
    unsigned char temp_lo = 0;
    
    startI2C1();
    putI2C1(ADDRESS_WRITE);
    putI2C1(READ_TEMP);
    restartI2C1();
    putI2C1(ADDRESS_READ);
    temp_hi = getI2C1(0);
    temp_lo = getI2C1(1);
    stopI2C1();
    return(((temp_hi<<8) | temp_lo)/256);
}

void configDS1631(void) {
    startI2C1();
    putI2C1(ADDRESS_WRITE);
    putI2C1(ACCESS_CONFIG);
    putI2C1(CONFIG_DATA);
    stopI2C1();
}    

void configI2C1(void) {
    I2C1BRG = 0x1F;
    I2C1CONbits.I2CEN = 1;
}

void startConvertDS1631(void) {
    startI2C1();
    putI2C1(ADDRESS_WRITE);
    putI2C1(START_CONVERT);
    stopI2C1();
}

void stopI2C1(void) {
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN);
}

int putI2C1(char data) {
    I2C1TRN = data;
    while(I2C1STATbits.TRSTAT);
    return(I2C1STATbits.ACKSTAT);
}

void startI2C1(void) {
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN);
}

void restartI2C1(void) {
    I2C1CONbits.RSEN = 1;
    while(I2C1CONbits.RSEN);
}