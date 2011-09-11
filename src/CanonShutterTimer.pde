// Define rotary encoder pins
#define ENC_A 14
#define ENC_B 15
#define ENC_PORT PINC

// Define LED pin
#define LED_A 16

#include "ircodes.h"
#include <SimpleTimer.h>

// Setup the IR LED pin as an output.
void _setupLEDPin()
{
  pinMode(LED_A, OUTPUT);
}

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
  for (int ii = 2; ii < 10; ii++) {
    // all pins 2-9 OUTPUTs
    pinMode(ii, OUTPUT);
  }
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT); 

  for (int ii = 2; ii < 10; ii++) {
    // all pins 2-9 OUTPUTs
    digitalWrite(ii, LOW);
  }
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW); 
}

// This procedure sends a 38KHz pulse to the IRledPin 
// for a certain # of microseconds. We'll use this whenever we need to send codes
void pulseIR(long microsecs) {
  // we'll count down from the number of microseconds we are told to wait

  cli();  // this turns off any background interrupts

  while (microsecs > 0) {
    // 38 kHz is about 13 microseconds high and 13 microseconds low
    digitalWrite(LED_A, HIGH);  // this takes about 3 microseconds to happen
    delayMicroseconds(10);         // hang out for 10 microseconds
    digitalWrite(LED_A, LOW);   // this also takes about 3 microseconds
    delayMicroseconds(10);         // hang out for 10 microseconds

    // so 26 microseconds altogether
    microsecs -= 26;
  }

  sei();  // this turns them back on
}

// Send an IR signal using the given array
// as the on/off timings.
void sendSignal(int arr[], int len) {
  for (int i = 0; i < len; i++) {
    if (i%2 == 0) {
      pulseIR(arr[i]);
    }
    else {
      delayMicroseconds(arr[i]);
    }
  }
}

// Turn on the given LED segment (1 is ones, 2 is tens, etc)
void turnOnSquare(int num)
{
  int d1=9,d2=17,d3=18,d4=19;
  switch(num)
  {
  case 1:
    digitalWrite(d2,LOW);
    digitalWrite(d3,LOW);
    digitalWrite(d4,LOW);
    digitalWrite(d1,HIGH);
    break;
  case 2:
    digitalWrite(d1,LOW);
    digitalWrite(d3,LOW);
    digitalWrite(d4,LOW);
    digitalWrite(d2,HIGH);
    break;
  case 3:
    digitalWrite(d1,LOW);
    digitalWrite(d2,LOW);
    digitalWrite(d4,LOW);
    digitalWrite(d3,HIGH);
    break;
  case 4:
    digitalWrite(d1,LOW);
    digitalWrite(d2,LOW);
    digitalWrite(d3,LOW);
    digitalWrite(d4,HIGH);
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
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);      
    digitalWrite(8, HIGH); // turn off pin 8
    break;
  case 1:
    //PORTD=B11100111; // only pins 3 and 4 are on
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);      
    digitalWrite(8, HIGH); // turn off pin 8
    break;
  case 2:
    //PORTD=B10010011; // only pins 2,3,5, 6 and 8 on
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);      
    digitalWrite(8,LOW); // segment g on
    break;
  case 3:
    //PORTD=B11000011; // only pins 2,3,4 and 5 on
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);      
    digitalWrite(8,LOW); // segment g on
    break;
  case 4:
    //PORTD=B01100111; // only pins 3,4 and 7 on
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);      
    digitalWrite(8,LOW); // segment g on
    break;
  case 5:
    //PORTD=B01001011; //B10110100; // only pins 2,4,5 and 7 on
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);      
    digitalWrite(8,LOW); // segment g on
    break;
  case 6:
    //PORTD=B00001011; //B11110100; // only pins 2,4,5,6 and 7 on
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);      
    digitalWrite(8,LOW); // segment g on
    break;
  case 7:
    //PORTD=B11100011; // only pins 2,3 and 4 on
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);      
    digitalWrite(8,HIGH); // segment g off
    break;
  case 8:
    //PORTD=B00000011; // pins 2-7 on
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);      
    digitalWrite(8,LOW); // turn on pin 8
    break;
  case 9:
    //PORTD=B01000011; // only pins 2,3, 4 and 5 on
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);      
    digitalWrite(8,LOW); // segment g on
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
  sendSignal(CanonIRsignal, arrayLen);
  delay(65);
  sendSignal(CanonIRsignal, arrayLen);
}

SimpleTimer timer;

void setup()
{
  // Rotary screen
  _setupRotaryPins();
  // LED screen
  _setupScreenPins();
  // IR LED
  _setupLEDPin();

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


