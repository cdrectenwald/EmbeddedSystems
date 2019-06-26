/*
 * File:   RGB_LED.h
 * Author: Chris Rectenwald
 * Purpose:  Header file to setup and manipulate an RGB LED simply
 */

// Pins for LED Control Signals
#define LED_RED LATBbits.LATB13             // red pin on RB13
#define LED_RED_PIN TRISBbits.TRISB13
#define LED_RED_PULLUP CNPUBbits.CNPUB13    // enable pull-up resistor

#define LED_GREEN LATBbits.LATB14           // green pin on RB14
#define LED_GREEN_PIN TRISBbits.TRISB14
#define LED_GREEN_PULLUP CNPUBbits.CNPUB14  // enable pull-up resistor

#define LED_BLUE LATBbits.LATB15            // blue pin on RB15
#define LED_BLUE_PIN TRISBbits.TRISB15
#define LED_BLUE_PULLUP CNPUBbits.CNPUB15   // enable pull-up resistor
  
#define IN 1
#define OUT 0

#define ON 1
#define OFF 0

#define RED 0
#define GREEN 1
#define BLUE 2


void RGB_LED_init(void) {       // initialize all pins as outputs w/ pull-ups on
    LED_RED_PIN = OUT;
    LED_RED_PULLUP = ON;
    LED_RED = OFF;
    
    LED_GREEN_PIN = OUT;
    LED_GREEN_PULLUP = ON;
    LED_GREEN = OFF;
    
    LED_BLUE_PIN = OUT;
    LED_BLUE_PULLUP = ON;
    LED_BLUE = OFF;
}

void RGB_Color(unsigned char color) {       // choose a color and light 'em uP!
    
    if (color == RED) {
        LED_RED = ON;
        LED_GREEN = OFF;
        LED_BLUE = OFF;
    }
    if (color == GREEN) {
        LED_RED = OFF;
        LED_GREEN = ON;
        LED_BLUE = OFF;
    }
    if (color == BLUE) {
        LED_RED = OFF;
        LED_GREEN = OFF;
        LED_BLUE = ON;
    }
    
}