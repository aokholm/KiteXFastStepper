# KiteXFastStepper
Fast stepper motor driver that listens to i2c

# Motivation/What is This?
The AccelStepper Library by Mike McCauley allows up to 2000 steps per second on a Arduino Mini Pro (8 MHz, 3.3V). 
I wanted to see how many steps per second it would be possible to achive on this microcontroller, while maintaining an optimal acceleration curve.
This library (FastStepper) precomputes the speed profile. So trades memory use for speed. It should be able to achive around 10000 steps per second - an 5x improvement.
Note you have to implement the (forward, backwards) and backwards step calls yourselves. 
This is due the Arduino digitalWrite function is quite slow (up towards 100ms), 
while interfacing directly with the hardware you can achieve speeds around 30 ms. See the stepperController.ino for an example of how it can be done for the Mini Pro.

# Notes
The stepperController is a program that listens on i2c for new input on where to place the stepper motor, and executes this as fast as possible.
