/*
 * File:   FinalChris.c
 * Author: cdrectenwald
 *
 * Created on April 30, 2016, 10:10 AM
 */


#include "xc.h"
#include "myconfig.h" //uses i2c2 communication
#include <stdio.h>

#define BAUDRATE 19200
#define I2C_ACK 0
#define I2C_NACK 1
#define FCY 7370000/2 // Frc runs at 7.37MHz
#define FSCL 100 // SPI clock frequency in kHz
#define ADDRESS 0b10010000
//define states
#define ON 1
#define OFF 0
#define IN 1
#define OUT 0
#define HIGH 1
#define LOW 0
#define LCD_COMMAND LOW
#define LCD_DATA HIGH

#define LCD_SCE LATBbits.LATB10
#define LCD_SCE_PIN TRISBbits.TRISB10
#define LCD_RESET LATBbits.LATB11
#define LCD_RESET_PIN TRISBbits.TRISB11
#define LCD_DC LATBbits.LATB12
#define LCD_DC_PIN TRISBbits.TRISB12

#define UpButton PORTBbits.RB0
#define DownButton PORTBbits.RB1

#define LED_RED LATBbits.LATB13
#define LED_RED_PIN TRISBbits.TRISB13
#define LED_RED_PULLUP CNPUBbits.CNPUB13

#define LED_GREEN LATBbits.LATB14
#define LED_GREEN_PIN TRISBbits.TRISB14
#define LED_GREEN_PULLUP CNPUBbits.CNPUB14

#define LED_BLUE LATBbits.LATB15
#define LED_BLUE_PIN TRISBbits.TRISB15
#define LED_BLUE_PULLUP CNPUBbits.CNPUB15

// Pins for SPIx 
#define USESPI1 // define to USESPI1/USESPI2 depending on what set you are using
#ifdef USESPI1
// Pin 16 is used for SCK1, pin 17 for SDO1, and pine 18 for SDI1
#else
    #define USESCK2 RPOR2bits.RP38R = 9; // RP38 is SCK2
    #define USESDO2 RPOR1bits.RP37R = 8; // RP39 is SDO2
    #define USESDI2 RPINR22bits.SDI2R = 20; // RP20 is SDI2
#endif

typedef unsigned char LCDBYTE;

void configPins();
void initInterrupts();
void initTimer();
void configUART1();
void configDS1631();
void startConvertDS1631();
unsigned int readTempDS1631();
void configI2C();
void startI2C2();
void rstartI2C2();
void stopI2C2();
int putI2C2(char data);
int getI2C2(char ack2send);
void LCDWrite(LCDBYTE dc, LCDBYTE data);
void LCDCharacter(char);
void intToChar(float ints);
void printNumber();
void getTemp();


int State=1; //used to determine what is displayed on the UART and LCD
float tempF;     //Holds temperature value in F                       
float tempC;     //Holds temperature value in C                       
float tempMax=77;      //Defines  high bounds for temperature           
float tempMin=76;    //Defines the low bound for the temperature
char stringa[100]; //string used to place temps to be displayed to the moniter thorugh uart
float oldValue=0;  //used as a comparison value against the current temperature read. This will make the UARt and LCD display the temperature only when it is changed.
int i; //used to iterate through for loops in main

//LCD config stuff that professor stevenson provided the class 
static const LCDBYTE ASCII[][5] =
{
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20  
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j 
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};

float ints;
int tensPlace; 
int onesPlace;
int tenthsPlace;
int hundredthsPlace;
char tensString;
char onesString;
char tenthsString;
char hundredthsString;


int main(void) {
   //Used to initialize the LCD and RGB outputs.
    LCD_SCE_PIN = OUT;
    LCD_RESET_PIN = OUT;  
    LCD_DC_PIN = OUT;
    
    LED_RED_PIN = OUT;
    LED_RED_PULLUP = ON;
    LED_RED = OFF;
    
    LED_GREEN_PIN = OUT;
    LED_GREEN_PULLUP = ON;
    LED_GREEN = OFF;
    
    LED_BLUE_PIN = OUT;
    LED_BLUE_PULLUP = ON;
    LED_BLUE = OFF;
    
 //Initialize necessary functions in order to properly run the program
    initInterrupts();
    initTimer();
    configPins();
    configUART1(); //Learned that one should configure the communication systems before the DS1631
    configI2C();
    configDS1631();
    startConvertDS1631();
 
    

#ifdef USESPI2
    USESCK2;
    USESDO2;
    USESDI2;
#endif
// more LCD configuration code that Professor Stevenson provided the classe
#ifdef USESPI1                              
    SPI1CON1bits.MSTEN = 1;                 // make master
    SPI1CON1bits.PPRE = 0b11;               // 1:1 primary prescale
    SPI1CON1bits.SPRE = 0b111;              // 1:1 secondary prescale
    SPI1CON1bits.MODE16 = 0;                // 8-bit transfer mode
    SPI1CON1bits.SMP = 0;                   // sample in middle
    SPI1CON1bits.CKE = 1;                   // Output on falling edge
    SPI1CON1bits.CKP = 0;                   // CLK idle state low
    SPI1STATbits.SPIEN = 1;                 // enable SPI1
#else
    SPI2CON1bits.MSTEN = 1;                 // make master
    SPI2CON1bits.PPRE = 0b11;               // 1:1 primary prescale
    SPI2CON1bits.SPRE = 0b111;              // 1:1 secondary prescale
    SPI2CON1bits.MODE16 = 0;                // 8-bit transfer mode
    SPI2CON1bits.SMP = 0;                   // sample in middle
    SPI2CON1bits.CKE = 1;                   // Output on falling edge
    SPI2CON1bits.CKP = 0;                   // CLK idle state low
    SPI2STATbits.SPIEN = 1;                 // enable SPI1
#endif
    
    LCD_RESET = LOW;
    LCD_RESET = HIGH;    
        
    LCDWrite(LCD_COMMAND, 0x21);            // LCD Extended Commands.
    LCDWrite(LCD_COMMAND, 0xB0 );           // Set LCD Vop (Contrast). 
    LCDWrite(LCD_COMMAND, 0x04 );           // Set Temp coefficent. //0x04
    LCDWrite(LCD_COMMAND, 0x14 );           // LCD bias mode 1:48. //0x13
    LCDWrite(LCD_COMMAND, 0x20 );           // LCD Basic Commands
    LCDWrite(LCD_COMMAND, 0x0C );           // LCD in normal mode.
  
 //While statement 
    while (1) {
        getTemp();   
        switch (State) {
            
            case 1:   //When the LCD/UART displays the current temp
                if (tempF<tempMin && tempF !=oldValue) {                      
           //When the temperature in the air is less than the lower bound temp value
            intToChar(tempF);    //see function description in definition        
            LED_BLUE=OFF; LED_GREEN=OFF;LED_RED=ON; //sets RGB output to correspond to temp.
            //Display on the LCD that the temp needs to get higher
            LCDCharacter('H'); LCDCharacter('e'); LCDCharacter('a'); LCDCharacter('t');
            LCDCharacter(' '); LCDCharacter('U'); LCDCharacter('p'); LCDCharacter('!');   
            LCDCharacter(' '); LCDCharacter(' '); LCDCharacter(' '); LCDCharacter(' ');
            for (i=0; i<12; i++) {
                LCDCharacter(' '); //use provide spacing between the temp and message
            }
            printNumber(); // used display the temperature on the lcd. for more info see definition
            for (i=0; i<43; i++) { //clears screen
                LCDCharacter(' ');
            } //
            sprintf(stringa, "Temperature is below minimum threshold: %f \n\r", tempF);
            puts(stringa); //display temperature to the UART                              
            oldValue=tempF; //creates a new Old temp to see if future temp changes.
            
        } else if (tempF>tempMax && tempF != oldValue) {  //For when the temp is higher the the higher bound        
            intToChar(tempF);    
            LED_BLUE=ON;LED_GREEN=OFF; LED_RED=OFF; //indicates the temp needs to cool down with RGB
            //display on LCD that temp needs to cool down
            LCDCharacter('C'); LCDCharacter('o'); LCDCharacter('o'); LCDCharacter('l');
            LCDCharacter('D'); LCDCharacter('o'); LCDCharacter('w'); LCDCharacter('n');
            LCDCharacter('!'); LCDCharacter(' '); LCDCharacter(' '); LCDCharacter(' ');
            for (i=0; i<12; i++) {LCDCharacter(' ');}
            printNumber(); //Provide spacing between message and temperature
            for (i=0; i<43; i++) {LCDCharacter(' ');}
            sprintf(stringa, "Temperature exceeds maximum threshold: %5.2f \n\r", tempF); //displays 
            puts(stringa);                               
            oldValue=tempF;
        } else if (tempF != oldValue) {                         
            intToChar(tempF);   
            
            //Displays Green on RGB, so the temperature is within bounds set.
            LED_BLUE=OFF; LED_GREEN=ON; LED_RED=OFF; 
            //Displays that temp is "Just Right"        
            LCDCharacter('J'); LCDCharacter('u'); LCDCharacter('s'); LCDCharacter('t');
            LCDCharacter(' '); LCDCharacter('R'); LCDCharacter('i'); LCDCharacter('g');
            LCDCharacter('h'); LCDCharacter('t'); LCDCharacter('.'); LCDCharacter(' ');
            for (i=0; i<12; i++) 
            {LCDCharacter(' '); //Proviced proper spacing between number and message
            }
            printNumber(); 
            for (i=0; i<43; i++) {LCDCharacter(' ');}
            oldValue=tempF;
            sprintf(stringa, "The temperature is within the specified bounds: %5.2f\n\r", tempF);
            puts(stringa);                               
        }
            break;
            
            case 2: //Set the High temperature bound   
            //General insructions for the RGB for temp read vs high and low bounds
            if (tempF>tempMax) {
                //Too hight; it needs to cool down
            LED_BLUE=ON; LED_GREEN=OFF; LED_RED=OFF;
            } 
            else if (tempF<tempMin) {
                //Too low; needs to heat up
            LED_BLUE=OFF; LED_GREEN=OFF; LED_RED=ON;
            } 
            else {// just right
                LED_BLUE=OFF; LED_GREEN=ON; LED_RED=OFF;
            }
                
            if (tempMax != oldValue) {// comparison value to see if UARt and LCD need to generate new message
            intToChar(tempMax);
            //Shows LCD message for high temperature bound
            LCDCharacter('M'); LCDCharacter('a'); LCDCharacter('x'); LCDCharacter('i');
            LCDCharacter('m'); LCDCharacter('u'); LCDCharacter('m'); LCDCharacter(':');
            LCDCharacter(' '); LCDCharacter(' '); LCDCharacter(' '); LCDCharacter(' ');
            for (i=0; i<12; i++) {
                LCDCharacter(' '); //Provide spacing between message and temperature
            }
            printNumber(); //shows High bound
            for (i=0; i<43; i++) {
                //clears screen
                LCDCharacter(' ');
            }
            //Display message to UART
            sprintf(stringa, "The high temperature value is %5.2f \n\r", tempMax);
            puts(stringa);
            oldValue=tempMax; //Create new comparison value
            }
            break;
            
            case 3:  // Displays Low bound value on LCD  
                //Instructions for RGB for temp relative to bounds
            if (tempF>tempMax) {
            LED_BLUE=ON; LED_GREEN=OFF; LED_RED=OFF;
            } else if (tempF<tempMin) {
            LED_BLUE=OFF; LED_GREEN=OFF; LED_RED=ON;
            } else { LED_BLUE=OFF; LED_GREEN=ON; LED_RED=OFF;
            }
            if (tempMin != oldValue) { //sees if temperature changes
            intToChar(tempMin);
            //Displays LCD message for low bound temperature 
            LCDCharacter('M'); LCDCharacter('i'); LCDCharacter('n'); LCDCharacter('i');
            LCDCharacter('m'); LCDCharacter('u'); LCDCharacter('m'); LCDCharacter(':');
            LCDCharacter(' '); LCDCharacter(' '); LCDCharacter(' '); LCDCharacter(' ');
            for (i=0; i<12; i++) {
                LCDCharacter(' '); //creates spacing between message and temperature
            }
            printNumber(); //displays temperature
            for (i=0; i<43; i++) {
                LCDCharacter(' '); //Clears screen
            }
            //Displays meassage and bound to UART
            sprintf(stringa, "The low temperature value is %5.2f \n\r", tempMin);
            puts(stringa);
            oldValue=tempMin; //New comparions value
            }
            break;
        }
    }
    return 0;
}

//Configuration code that Professor Stevenson gave us
void configPins(void) {    
    RPINR18bits.U1RXR=20;               
    RPOR1bits.RP36R=1;
    ANSELBbits.ANSB1=0;                 
    ANSELBbits.ANSB0=0;                    
    TRISBbits.TRISB1=1;                 
    TRISBbits.TRISB0=1;                    
    CNPUBbits.CNPUB1=1;                 
    CNPUBbits.CNPUB0=1;                 
    CNENBbits.CNIEB1=1;                 
    CNENBbits.CNIEB0=1;                                      
}

//Initiliaziong interrupts. Followed 
void initInterrupts() {
    INTCON2bits.GIE=1; //Enable level 1-7 interrupts   
    IFS1bits.CNIF=0; //Clear change of notification
    IPC4bits.CNIP=2; //Set change notification interrupt priority to 4
    IEC1bits.CNIE=1; //Enable change of notification interrupt
}    
//Initiliazes the timer in T1 interrupt. Based off Professor Stevenson's example code
void initTimer() {
    T1CONbits.TON = 0;//Disable timer
    T1CONbits.TCS = 0;// enables internal clock
    T1CONbits.TGATE = 0;// disallows clock gating
    T1CONbits.TCKPS = 0b01; // set the presclar to 8:1
    TMR1 = 0;//timer reset
    PR1 = 45000;//period value
    IPC0bits.T1IP = 3;//Sets priority
    IFS0bits.T1IF = 0; //Clear interrupt flag
    IEC0bits.T1IE = 1;//Enables interrupt
    T1CONbits.TON = 0;//Turns off Timer 
}
//Based off Professor Stevenson's example code. Is used to configure the UART communication
void configUART1() {

    int brg = (FCY/BAUDRATE/4)-1; //calculates the bbaudreate
    U1MODEbits.BRGH = 1;// High speed mode
    U1BRG = brg;//stores value in register
    
    U1MODEbits.PDSEL = 0; // 8 bit data with 0 parity bits
    U1MODEbits.STSEL = 0; // 1 stop bit
    
    U1MODEbits.UEN = 0b00; 
    U1MODEbits.UARTEN = 1;// enable UART RX/TX      
    U1STAbits.UTXEN = 1;      
}
//Based off Professor Stevenson's example code. Is used to configure the DS1631
void configDS1631() { 
    startI2C2(); //start transfer
    putI2C2(ADDRESS & 0xFE);//sends address
    putI2C2(0xac);//sends access
    putI2C2(0x0c);//Send configuration
    stopI2C2();//finish the transgfer
}
////Based off Professor Stevenson's example code. Is used to start the conversion of DS1631
void startConvertDS1631() { 

    startI2C2(); //start transfer
    putI2C2(ADDRESS & 0xfe);//send addres
    putI2C2(0x51);
    stopI2C2();//finsishes transger
}
//
unsigned int readTempDS1631(void) { 
    int temp_hi, temp_lo;
    startI2C2(); // Start transfer
    putI2C2(ADDRESS & 0xFE);// Send address
    putI2C2(0xAA); //sends read temperature
    rstartI2C2(); //restarts the transfer
    putI2C2(ADDRESS | 0x01); //send address
    temp_hi = getI2C2(I2C_ACK); // gets the  ack (MSBO) of the temperature
    temp_lo = getI2C2(I2C_NACK); // gets the nack of (LSB) temperature
    stopI2C2(); // stops the transger
    return ((temp_hi<<8) | temp_lo);
}
 //Based off Professor Stevenson's example code. Is used to configure the I2C2 communication
void configI2C(void) {
// Determine the Baud Rate Generator reload value
// brg = FCY/(1000*FSCL) - FCY/1000000 - 2 = 31    
    I2C2BRG = 0x1F; //Hardcoded 31
    I2C2CONbits.I2CEN = 1; //Enalbe the i2c communication
}

void startI2C2() { 
    I2C2CONbits.SEN = 1; //initilize start 
    while(I2C2CONbits.SEN); //continue until finished
}

//Exact same as startI2C2
void rstartI2C2() { 
    I2C2CONbits.RSEN = 1;
    while(I2C2CONbits.RSEN); 
}

void stopI2C2() { 
    I2C2CONbits.PEN = 1; //This sets the stop
    while(I2C2CONbits.PEN);//This waits until stop occurs.
}

int putI2C2(char data) { 
    I2C2TRN = data; //put data in transmit buffer
    while(I2C2STATbits.TRSTAT); //Wait until transfer finished
    
    return(I2C2STATbits.ACKSTAT);/
}

int getI2C2(char ack2send) {
    int inByte;
    
    while(I2C2CON & 0x1F);//Wait for idle condition
    I2C2CONbits.RCEN = 1;//Enable receive
    while(!I2C2STATbits.RBF);//Wait for receive byte
    inByte = I2C2RCV; //read byte
    
    while(I2C2CON & 0x1F); //wait for idle condition
    I2C2CONbits.ACKDT = ack2send;
    I2C2CONbits.ACKEN = 1; //enable 0 bit transismission (ACK)
    while(I2C2CONbits.ACKEN);
    
    return(inByte);
    
}
//Provided by Professor Stebvenson
void LCDWrite(LCDBYTE dc, LCDBYTE data) {
    LCD_SCE = LOW; // Selection LCD panel
    LCD_DC = dc; // Indicate Data/Command signal
#ifdef USESPI1
    SPI1BUF = data;                     // Put data in output buffer for SPI1
    while(!SPI1STATbits.SPIRBF){}       // wait for it to send
#else
    SPI2BUF = data;                     // Put data in output buffer for SPI2
    while(!SPI2STATbits.SPIRBF){}       // wait for it to send
#endif
    Nop();                              // A little extra wait time
    LCD_SCE = HIGH;                     // Deselect LCD panel
       
}
//Given by Professor Stevenson
void LCDCharacter(char character)
{
    int index;
    LCDWrite(LCD_DATA, 0x00);
    for (index = 0; index < 5; index++) {
        LCDWrite(LCD_DATA, ASCII[character - 0x20][index]);
    }
    LCDWrite(LCD_DATA, 0x00);
}
  
void intToChar(float ints) {
    
    //This function takes a float integer and converts it to a char, so it can get 
    // used to be able to be used in printNumber which is used to display the temperature on the 
    // LCD screen
    tensPlace= (int) (ints/10)%10; //divides by 10 and modulus to see tens place
    onesPlace= (int) (ints)%10;// similar way for tens place
    tenthsPlace= (int) (10*ints)%10;// shifts number to get .1 places
    hundredthsPlace= (int) (100*ints)%10; //shifts number to get .01 places
    // this part takes the integers calculated in the first part of the function
    // then converts the integers to strings to it can be used in the LCD character function
    tensString= '0'+tensPlace;                  
    onesString= '0'+onesPlace;
    tenthsString= '0'+tenthsPlace;
    hundredthsString= '0'+hundredthsPlace;
}

//Function that uses brings the method of printing the temp on the LCD
 // using LCD character function provided by Professor Stevenson
void printNumber() {
    LCDCharacter(tensString);
    LCDCharacter(onesString);
    LCDCharacter('.');
    LCDCharacter(tenthsString);
    LCDCharacter(hundredthsString);
}
//followed from HW8 Problem 3 code; retreives a temperature in C f
// readTemp1631 then converts to a temp in fahrenheit in the necessary steps
void getTemp() {
    //followed from HW8 Problem 3 code; retreives a temperature in C f
    // readTemp1631 then converts to a temp in fahrenheit in the necessary steps
    tempC= (int) readTempDS1631();          
    tempC = tempC/256;                           
    tempF= tempC*9;      
    tempF=tempF/5;
    tempF = tempF+32;
    
}
// initializing CN interrupt; followed example code from Professor Stevenson
void __attribute__((__interrupt__,no_auto_psv)) _CNInterrupt() {
    T1CONbits.TON=1; 
    IFS1bits.CNIF=0; 
}
 

//The use of this timer interrupt is to properly allow the buttons to 
// dictate which message to appear on the LCD and the UART
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt() {
    T1CONbits.TON=0; 
    IEC1bits.CNIE=1;                     
    TMR1=0;
    IFS0bits.T1IF=0;                     
    
    //The use of this timer interrupt is to properly allow the buttons to 
    // dictate which message to appear on the LCD and the UART
    switch(State) { //switch
        case 1:
            if(UpButton==0 && DownButton==0) { //Both Pressed
                State++;                
            }
            break;
        case 2:
            if(UpButton==0 && DownButton==0) { //Both Pressed
                State++; //Change messages on uart and LCD to display low bound
            } else if (UpButton==1 && DownButton==0) { //Down Button Pressed
                tempMax--; //changes the high bound by -1
            } else if (UpButton==0 && DownButton==1) { //Up Button pressed
                    tempMax++;   //increments the high vound                 
            }
            break;
        case 3:
            if (UpButton==0 && DownButton==0) { //Both PRessed, switch states
                State=1; //Change messages on uart and LCD to display temperature
            } else if (UpButton==1 && DownButton==0) { //Down Button pressed
                tempMin--; //Decrements low bound               
            } else if (UpButton==0 && DownButton==1) { //Up Button pressed 
                tempMin++; //increments low bound
            }
    }
}

