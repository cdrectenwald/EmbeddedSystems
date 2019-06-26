/*
 * File:   main.c
 * Author: Chris Rectenwald
 *
 * 
 */

unsigned int current_temp = 0;      // catch current temp here
unsigned int previous_temp = 0;     // used to compare current temp to last one
unsigned int high_temp = 24;        // user-defined max temp, init to 24
unsigned int low_temp = 20;         // user-defined low temp, init to 20

unsigned char mode = 0;             // mode 0 = update, mode 1 = user input
unsigned char condition = 0;        // conditions are heating, cooling, none

void UpdateTemp(void);          // called to check temp + update display
void Initialize(void);          // start up all header file init functions

#include "p24EP128GP202.h"
#include "myconfig.h"
#include "UART.h"
#include "LCD_Control.h"
#include "DS1631.h"
#include "RGB_LED.h"
#include "Buttons.h"
#include "ISRs.h"

#define NONE 0
#define HEATING 1
#define COOLING 2

int main(void) {

    Initialize();

    while (1) {
        if (!mode) {
            UpdateTemp();      // only update the temp if we're not in user mode
        }
    }

    return 0;
}

void UpdateTemp(void) {
    current_temp = readTempDS1631();        // read current temp
    char temp_buff[6];                      // used to hold char equiv. of temp
    itoa(current_temp, temp_buff);          // convert the int temp to char
                                                // (allows us to print to LCD)
    if (current_temp > high_temp) {
        condition = 2;                      // check if we entered cooling state
        RGB_Color(BLUE);
    }

    else if (current_temp < low_temp) {
        condition = HEATING;                // check if we entered heating state
        RGB_Color(RED);
    }
    else {
        condition = NONE;                   // otherwise, normal state
        RGB_Color(GREEN);
    }

    if (current_temp != previous_temp) {    // only refresh screen on change
        my_puts(state_update[12]);          // print status change to UART
        outChar1(temp_buff[0]);
        outChar1(temp_buff[1]);
        LCDTemp(current_temp, high_temp, low_temp, condition); // update screen
        my_puts(state_update[0]);
    }
    
    previous_temp = current_temp;
}

void Initialize(void) {
    pin_setup();                // setup pins for buttons and PWM output
    timer_setup();              // config timer module for button interrupts   
    configPins();               // setup inputs / outpus, remapped pins, etc.
    configUART1();              // config UART module
    configI2C1();               // config I2C module
    configDS1631();             // send startup info to temp sensor via I2C
    startConvertDS1631();       // tell temp sensor to begin conversion
    LCDInit();                  // initialize LCD screen over SPI and clear
    RGB_LED_init();             // setup pins for RGB LED output
}