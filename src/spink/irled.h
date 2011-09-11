/*
  irled.h - Library for flashing an IR LED with 
  a given signal.
  Created by jdcockrill, September 11, 2011.
*/
#ifndef irled_h
#define irled_h

#include "WProgram.h"

class IRLED
{
  public:
    IRLED(int pin);
    void sendSignal(int arr[], int len);
  private:
    void _pulseIR(long microsecs);
    int _pin;
};

#endif
