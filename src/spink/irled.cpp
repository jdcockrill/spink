/*
  irled.cpp - Library for flashing an IR LED with 
  a given signal.
  Created by jdcockrill, September 11, 2011.
*/
#include "WProgram.h"
#include "irled.h"

IRLED::IRLED(int pin)
{
  pinMode(pin, OUTPUT);
  _pin = pin;
}

// This procedure sends a 38KHz pulse to the IRledPin 
// for a certain # of microseconds. We'll use this whenever we need to send codes
void IRLED::_pulseIR(long microsecs) {
  // we'll count down from the number of microseconds we are told to wait

  cli();  // this turns off any background interrupts

  while (microsecs > 0) {
    // 38 kHz is about 13 microseconds high and 13 microseconds low
    digitalWrite(_pin, HIGH);  // this takes about 3 microseconds to happen
    delayMicroseconds(10);         // hang out for 10 microseconds
    digitalWrite(_pin, LOW);   // this also takes about 3 microseconds
    delayMicroseconds(10);         // hang out for 10 microseconds

    // so 26 microseconds altogether
    microsecs -= 26;
  }

  sei();  // this turns them back on
}

// Send an IR signal using the given array
// as the on/off timings.
void IRLED::sendSignal(int arr[], int len) {
  for (int i = 0; i < len; i++) {
    if (i%2 == 0) {
      _pulseIR(arr[i]);
    }
    else {
      delayMicroseconds(arr[i]);
    }
  }
}
