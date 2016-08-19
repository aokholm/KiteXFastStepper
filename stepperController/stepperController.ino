// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain. 


#include <Wire.h>
#include <FastStepper.h>
//#include <AccelStepper.h>

#define STEP_PIN 6
#define DIR_PIN 5


void forward() {
  //Serial.println("f");
  PORTD = B00100000;
  PORTD = B01100000;
  delayMicroseconds(8);
  PORTD = B00100000;
}

void backward() {
  //Serial.println("b");
  PORTD = B00000000;
  PORTD = B01000000;
  delayMicroseconds(8);
  PORTD = B00000000;
}


FastStepper stepper(forward, backward, 4000, 17000);

int x = 100; // move a bit on start
int lastX = 0;

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(115200);           // start serial for output
  
  // set ports 
  short outputPorts = B01100000; // port 5 & 6 on arduino 
  DDRD = outputPorts & B11111100; // safety measure setting 0 and 1 is serial communication
}

void loop() {
  stepper.run();

  if (x != lastX) {
    lastX = x;
    stepper.moveTo(x);
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  x = (Wire.read() | Wire.read() << 8);
          // print the integer
}

