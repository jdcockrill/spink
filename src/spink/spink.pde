 /*
  spink.pde - An adjustable shutter-timer for
  my Canon EOS 550D. A rotary encoder sets the
  time, it is displayed on a 4-digit 7-segment
  LED display and an IR LED emits the signal to
  trigger the camera.
  Created by jdcockrill, September 11, 2011.
*/
// Define rotary encoder pins
#define ENC_A 14
#define ENC_B 15
#define ENC_PORT PINC

// Define LED pin
#define LED_A 16

#include "ircodes.h"
#include "irled.h"
#include <SimpleTimer.h>

// Setup the IR LED pin as an output.
IRLED irled(LED_A);

// Define Screen pins
int screenSegmentPins[] = {2,3,4,5,6,7,8};
int screenSegmentPinCount = 7; // this is fixed for a 7-segment display
int screenControlPins[] = {9,10,11,12};
int screenControlPinCount = 4; // this is fixed for a display with 4 digits

// Set up the pins for the rotary encoder as inputs
void _setupRotaryPins()
{
  /* Setup encoder pins as inputs */
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
}

// Set up the pins for the LED screen as outputs
void _setupScreenPins() 
{
  // Set all segment and control pins to output
  for (int ii = 0; ii < screenSegmentPinCount; ii++) {
    pinMode(screenSegmentPins[ii], OUTPUT);
  }
  for (int ii = 0; ii < screenControlPinCount; ii++) {
    pinMode(screenControlPins[ii], OUTPUT);
  }

  // Set all segment and control pins to LOW
  for (int ii = 0; ii < screenSegmentPinCount; ii++) {
    digitalWrite(screenSegmentPins[ii], LOW);
  }
  for (int ii = 0; ii < screenControlPinCount; ii++) {
    digitalWrite(screenControlPins[ii], LOW);
  }
}

// Turn on the given LED segment (1 is ones, 2 is tens, etc)
void turnOnSquare(int num)
{
  switch(num)
  {
  case 1:
    digitalWrite(screenControlPins[1],LOW);
    digitalWrite(screenControlPins[2],LOW);
    digitalWrite(screenControlPins[3],LOW);
    digitalWrite(screenControlPins[0],HIGH);
    break;
  case 2:
    digitalWrite(screenControlPins[0],LOW);
    digitalWrite(screenControlPins[2],LOW);
    digitalWrite(screenControlPins[3],LOW);
    digitalWrite(screenControlPins[1],HIGH);
    break;
  case 3:
    digitalWrite(screenControlPins[0],LOW);
    digitalWrite(screenControlPins[1],LOW);
    digitalWrite(screenControlPins[3],LOW);
    digitalWrite(screenControlPins[2],HIGH);
    break;
  case 4:
    digitalWrite(screenControlPins[0],LOW);
    digitalWrite(screenControlPins[1],LOW);
    digitalWrite(screenControlPins[2],LOW);
    digitalWrite(screenControlPins[3],HIGH);
    break;
  default:
    // this should never occur, but do what you want here
    break;
  }
}

// Display the given number on the LED screen
// Only works for 0-9, will ignore anything else.
void displayDigit(int num)
{
  //  Serial.println("DisplayDigit");
  switch(num)
  {
  case 0:
    //PORTD=B00000011; // pins 2-7 on
    digitalWrite(screenSegmentPins[0], LOW);
    digitalWrite(screenSegmentPins[1], LOW);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], LOW);
    digitalWrite(screenSegmentPins[4], LOW);
    digitalWrite(screenSegmentPins[5], LOW);      
    digitalWrite(screenSegmentPins[6], HIGH); // turn off pin 8
    break;
  case 1:
    //PORTD=B11100111; // only pins 3 and 4 are on
    digitalWrite(screenSegmentPins[0], HIGH);
    digitalWrite(screenSegmentPins[1], LOW);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], HIGH);
    digitalWrite(screenSegmentPins[4], HIGH);
    digitalWrite(screenSegmentPins[5], HIGH);
    digitalWrite(screenSegmentPins[6], HIGH); // turn off pin 8
    break;
  case 2:
    //PORTD=B10010011; // only pins 2,3,5, 6 and 8 on
    digitalWrite(screenSegmentPins[0], LOW);
    digitalWrite(screenSegmentPins[1], LOW);
    digitalWrite(screenSegmentPins[2], HIGH);
    digitalWrite(screenSegmentPins[3], LOW);
    digitalWrite(screenSegmentPins[4], LOW);
    digitalWrite(screenSegmentPins[5], HIGH);
    digitalWrite(screenSegmentPins[6], LOW); // segment g on
    break;
  case 3:
    //PORTD=B11000011; // only pins 2,3,4 and 5 on
    digitalWrite(screenSegmentPins[0], LOW);
    digitalWrite(screenSegmentPins[1], LOW);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], LOW);
    digitalWrite(screenSegmentPins[4], HIGH);
    digitalWrite(screenSegmentPins[5], HIGH);
    digitalWrite(screenSegmentPins[6], LOW); // segment g on
    break;
  case 4:
    //PORTD=B01100111; // only pins 3,4 and 7 on
    digitalWrite(screenSegmentPins[0], HIGH);
    digitalWrite(screenSegmentPins[1], LOW);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], HIGH);
    digitalWrite(screenSegmentPins[4], HIGH);
    digitalWrite(screenSegmentPins[5], LOW);
    digitalWrite(screenSegmentPins[6], LOW); // segment g on
    break;
  case 5:
    //PORTD=B01001011; //B10110100; // only pins 2,4,5 and 7 on
    digitalWrite(screenSegmentPins[0], LOW);
    digitalWrite(screenSegmentPins[1], HIGH);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], LOW);
    digitalWrite(screenSegmentPins[4], HIGH);
    digitalWrite(screenSegmentPins[5], LOW);
    digitalWrite(screenSegmentPins[6], LOW); // segment g on
    break;
  case 6:
    //PORTD=B00001011; //B11110100; // only pins 2,4,5,6 and 7 on
    digitalWrite(screenSegmentPins[0], LOW);
    digitalWrite(screenSegmentPins[1], HIGH);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], LOW);
    digitalWrite(screenSegmentPins[4], LOW);
    digitalWrite(screenSegmentPins[5], LOW);
    digitalWrite(screenSegmentPins[6], LOW); // segment g on
    break;
  case 7:
    //PORTD=B11100011; // only pins 2,3 and 4 on
    digitalWrite(screenSegmentPins[0], LOW);
    digitalWrite(screenSegmentPins[1], LOW);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], HIGH);
    digitalWrite(screenSegmentPins[4], HIGH);
    digitalWrite(screenSegmentPins[5], HIGH);
    digitalWrite(screenSegmentPins[6], HIGH); // segment g off
    break;
  case 8:
    //PORTD=B00000011; // pins 2-7 on
    digitalWrite(screenSegmentPins[0], LOW);
    digitalWrite(screenSegmentPins[1], LOW);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], LOW);
    digitalWrite(screenSegmentPins[4], LOW);
    digitalWrite(screenSegmentPins[5], LOW);
    digitalWrite(screenSegmentPins[6], LOW); // turn on pin 8
    break;
  case 9:
    //PORTD=B01000011; // only pins 2,3, 4 and 5 on
    digitalWrite(screenSegmentPins[0], LOW);
    digitalWrite(screenSegmentPins[1], LOW);
    digitalWrite(screenSegmentPins[2], LOW);
    digitalWrite(screenSegmentPins[3], LOW);
    digitalWrite(screenSegmentPins[4], HIGH);
    digitalWrite(screenSegmentPins[5], LOW);
    digitalWrite(screenSegmentPins[6], LOW); // segment g on
    break;
  }
}

// take the given time in seconds and print to the LED display
void printTime(int v) {

  int ones;
  int tens;
  int minsOnes;
  int minsTens;

  if(v > 9999) {
    Serial.println("Skipped");
    return;
  }
  int s    = v%60;
  ones     = s%10;
  tens     = s/10;
  int m    = v/60;
  minsOnes = m%10;			
  minsTens = m/10;

  //Now print the number digit by digit
  delay(1);
  turnOnSquare(1);
  displayDigit(ones);
  delay(1);
  turnOnSquare(2);
  displayDigit(tens);
  delay(1);
  turnOnSquare(3);
  displayDigit(minsOnes);
  delay(1);
  turnOnSquare(4);
  displayDigit(minsTens);
}

/* returns change in encoder state (-1,0,1) */
int8_t read_encoder()
{
  static int8_t enc_states[] = {
    0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0      };
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

// Send the Canon IR signal. Used as a callback for SimpleTimer
void doSend(){
  Serial.println("Shutter trigger");
  int arrayLen = sizeof(CanonIRsignal) / sizeof(int);
  irled.sendSignal(CanonIRsignal, arrayLen);
  delay(65);
  irled.sendSignal(CanonIRsignal, arrayLen);
}

SimpleTimer timer;

void setup()
{
  // Rotary screen
  _setupRotaryPins();
  // LED screen
  _setupScreenPins();
  // IR LED
  //_setupLEDPin();

  // init serial communication
  Serial.begin(115200); 
  Serial.println("Ready to begin");
}

void loop()
{
  //this variable will be changed by encoder input
  //start @ 3 mins
  static long counter = 180L * 4L;
  //last known timer value
  static long lastval = 0;
  //last known timer ID
  static int timerId = -1;

  int8_t tmpdata;
  /* read in the data from the encoder */
  tmpdata = read_encoder();
  // if we got -1 or 1 (i.e. there was a change)
  if( tmpdata ) {
    // invert it (rotations seem to go the wrong way)
    tmpdata = tmpdata * -1;
    Serial.print("Counter value: ");
    Serial.println(counter, DEC);
    if ((counter + tmpdata) >= 0) {
      counter += tmpdata;
    }
  }

  // Each click seems to be 4-changes
  long val = counter / 4L;
  // If the value has changed a whole click
  if (lastval != val)
  {
    // Calculate the value in ms for the timer library
    long timerms = val * 1000L;
    Serial.print("Reset timer to: ");
    Serial.println(timerms);

    // Delete the old timer (if there was one)
    if (timerId != -1)
      timer.deleteTimer(timerId);
    // Set the new one.
    timerId = timer.setInterval(timerms, doSend);
    lastval = val;
  }
  // Poll the timer (signal is sent if timer has elapsed)
  timer.run();
  // Print the current value to the LED display.
  printTime(val);
}



