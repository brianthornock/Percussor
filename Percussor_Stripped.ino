#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins
// Digital/PB Pin Assignments
const unsigned int tapSwitch = 0; //PB0/Pin 5
const unsigned int trigger = 1; //PB1/Pin 6
const unsigned int ledOut = 3; //PB3/Pin 2


//Tap Tempo Parameters
unsigned int tapTime; //Time between pulses
unsigned int prevTapTime; //Used for debouncing tap tempo switch
unsigned int prevTapDelay; //Used for debouncing tap tempo switch
uint8_t maxTaps = 10; //Maximum number of taps to count for tap tempo
uint8_t useTap; //Indicator of whether we have sufficient data to do tap tempo
uint8_t prevTaps = 0;
unsigned int prevTimes [10];
uint8_t updateTapTempo = LOW;
unsigned long lastDebounceTime = 0; // initiate the debounce time counter
uint8_t buttonState;

unsigned int minTime; //Minimum time from tap pulse to pulse
unsigned int maxTime; //Maximum time from tap pulse to pulse
unsigned int tapTimeout; //Timeout for tap tempo

//Tap Tempo Switch Parameters
uint8_t tapState = LOW; //State of the tap switch
uint8_t lastTapState = LOW; //Used for debouncing tap switch
uint8_t tapDebounceDelay = 20; //Debounce time for tap switch

uint8_t pulseTime = 2; //Number of ms for a trigger pulse to be high
unsigned int pulseInterval; // Interval in ms between trigger pulses
unsigned long lastPulse; // Time of the last time we sent a pulse while in tap tempo mode

//Set up LED related items
int8_t currLEDState = LOW; // LED starts off off
unsigned int currLEDOffInterval; // How long the LED has been off
unsigned int currLEDOnInterval; // How long the LED has been on
int8_t updateLEDInterval = 1;
unsigned long prevMillis; // Used for keeping track of LED blinking
uint8_t maxLEDTime = 200;


void setup() {
  //Define what each pin is
  pinMode(tapSwitch, INPUT);
  pinMode(trigger, OUTPUT);
  pinMode(ledOut, OUTPUT);
  //pinMode(debugLED, OUTPUT);

  //Set up the initial state of the pins
  digitalWrite(tapSwitch, LOW);
  digitalWrite(trigger, LOW);
  digitalWrite(ledOut, LOW);
  //digitalWrite(debugLED, LOW);
  

  tapTime = 333; // Default time between pulses
  prevTapTime = tapTime;
  pulseInterval = tapTime;
  prevTapDelay = 0;

  minTime = 50; // shortest allowable period in ms between changes in voltage level
  maxTime = 2000; // longest allowable period in ms between changes in voltage level
  tapTimeout = round(1.5 * maxTime); // How long to keep waiting for taps
  lastPulse = millis();


  //Set up the pin change interrupts for the tap switch
  GIMSK = 0b00100000; //Enable pin change interrupts
  PCMSK = 0b00000001; //Enable PCIE on PCINT0 for tap switch
  sei(); //Start interrupt service
}



void loop() {

  //Check tap tempo and adjust the tempo if needed
  if (updateTapTempo){
    //Check to see if tap tempo is used
    checkTapTempo();
  }

  updateSampleTime();

  if ((millis() - lastPulse) > (pulseInterval - pulseTime)) {
    writeTrigger();
  }
  
  updateLED();
}//End loop




//Interrupt handling
ISR (PCINT0_vect) {

  if (digitalRead(tapSwitch)==HIGH){
    updateTapTempo=1;
  }

}



void checkTapTempo() {

  debounceTapTempo();

  if (tapState == HIGH) {

    tapState = LOW;
    //Check to see if we already have a tap tempo history. If so, add this to
    //the history. If not, start a new count.
    if (prevTaps > 0) {
      int currTime = millis();
      int currDelay = currTime - prevTapDelay;
      // Check to make sure we didn't time out
      if (currDelay < tapTimeout) {
        //Set the flag for using tap tempo
        useTap = 1;

        // Create the temp array for storing times in
        unsigned int newPrevTimes [maxTaps];

        if (prevTaps < maxTaps) {

          //Fill up the new array with all the old values first
          for (int k = 0; k < prevTaps - 1; k++) {
            newPrevTimes[k] = prevTimes[k];
          }

          //Then add in the new value at the end
          newPrevTimes[prevTaps - 1] = currDelay;
          ++prevTaps;

        } // End if prevTaps < maxTaps

        for (int nTime = 0; nTime < maxTaps; nTime++) {
          prevTimes[nTime] = newPrevTimes[nTime];
        }

      } // End if currDelay < tapTimeout
      else {
        //If we timeout, reset the counter and zero out the tempo array
        prevTaps = 1;

        for (int i = 0; i < maxTaps; i++) {
          prevTimes[i] = 0;
        }

        useTap = 0;
      } // End if tap has timed out
    } // End if prevTaps > 0
    // If we do not have any previous taps (first tap after timeout)
    else {
      
      prevTaps = 1;

      for (int i = 0; i < maxTaps; i++) {
        prevTimes[i] = 0;
      }

      useTap = 0;
    }

    if (useTap == 1 && prevTaps > 2) {
      //Calculate the average polling time, including the multiplier and the random switch
      int sum, loop, numVals;
      float avg;

      sum = avg = 0;
      numVals = 0;

      for (loop = 0; loop < prevTaps - 1; loop++) {
        if (prevTimes[loop] != 0) {
          sum += prevTimes[loop];
          numVals++;
        }
      }
      avg = (float)sum / numVals;
      tapTime = round(avg);

      if (tapTime > maxTime) {
        tapTime = maxTime;
      }
    }
    else {
      //If we don't have the information to produce a tap tempo, stick with what we have
    }
    prevTapDelay = millis();
  }

}



void updateSampleTime() {

  //If we are more than 5 ms off and not using pattern, update the sampleTime
  if ((useTap) && (abs(tapTime - prevTapTime) >= 5)) {
    pulseInterval = tapTime;
    updateLEDInterval = 1;
    //updateTapTempo = 0;
    //useTap = 0;
    //blinkLED(3,50);
  }
} // End updateSampleTime()



//Code for debouncing tap tempo switch
void debounceTapTempo() {
  int reading = digitalRead(tapSwitch);

  // Button is pressed down
  if (reading != lastTapState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > tapDebounceDelay) {

    if (reading != buttonState) {

      buttonState = reading;

      if (buttonState == HIGH) {
        tapState = HIGH;
      }
    }
  }
  lastTapState = reading;
}



// Update the tap tempo LED
void updateLED() {
  if (updateLEDInterval) {
    updateLEDInterval = 0;
    currLEDOnInterval = min(round(pulseInterval / 2),maxLEDTime);
    currLEDOffInterval = round(pulseInterval - currLEDOnInterval);
  }
 
  //Check to see if we have completed the LED on or off interval and change if we have
  if (currLEDState == LOW) {
    if (millis() - prevMillis >= currLEDOffInterval) {
      currLEDState = HIGH;
      prevMillis += currLEDOffInterval;
      digitalWrite(ledOut, HIGH);
    }
  }

  if (currLEDState == HIGH) {
    if (millis() - prevMillis >= currLEDOnInterval) {
      currLEDState = LOW;
      prevMillis += currLEDOnInterval;
      digitalWrite(ledOut, LOW);
    }
  }
}



// Write the trigger pulse
void writeTrigger() {

  digitalWrite(ledOut,HIGH);
  currLEDState = HIGH;
  prevMillis = millis();

  digitalWrite(trigger,HIGH);
  delay(pulseTime);
  digitalWrite(trigger,LOW);

  lastPulse = millis();
}



void blinkLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(ledOut, HIGH);
    delay(duration);
    digitalWrite(ledOut, LOW);
    delay(duration);
  }
}
