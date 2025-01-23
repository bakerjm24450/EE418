# _EE 418 Homework 3_

The goal of this assignment is to measure the interrupt latency on the ESP32 -- the 
time from when an interrupt occurs until the ISR executes.

The program should include an ISR triggered by a rising edge on one of the GPIO pins.
In the ISR, output a pulse to a different GPIO pin. 

Using an external function generator, apply a 100 kHz square wave to the input pin and observe
both the square wave input and the pulse on the output GPIO pin. The interrupt latency
is the time between the rising edge on the input and the rising edge on the output.