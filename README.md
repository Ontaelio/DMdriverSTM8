# DMdriverSTM8
Use DM631, DM632, DM633, DM634 with STM8 microcontrollers

Tested with ST Visual Develop.

Unlike the two libraries for Atmega328 and STM32, this one comes in the form of a single .c file. Use it as your main.c, writing your own code inside it, or create a .h header file out of it. 

If you do a library out of this please inform me.

Usage: mostly same as the libraries (check their repositaries please), except for no objects here nor default values due to C. Thus, all the functions don't have any object prefix (like 'Test.setPoint(), just setPoint() ).

Initializing is done with a single function DMdriverInit(uint8_t Driver, uint8_t Number, uint16_t LatchPort, uint8_t LatchPin), where

Driver is the DM chip(s) used; possible values DM631, DM632, DM633, DM634.

Number is the number of DM chips chained.

LatchPort is the port of the pin connected to LAT. Possible values __PORTA, __PORTB .. __PORTF.

LatchPin is the # of the pin connected to LAT. Thus D2 is __PORTD, 2; C4 is __PORTC, 4; etc.

This was done just for the sake of it. I do not intend to use STM8 chips in my projects, nor do I have any wish to work with the horrid STVD again in any foreseeable future. However, if you have any problems/questions I'll try to address them; please e-mail me (ontaelio@gmail)
