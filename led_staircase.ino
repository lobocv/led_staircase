
/*
LED Stair Case Project:

Up to 16 LED strips can be controlled using the Arduino and TLC5940NT PWM IC. THE TLC5940 allows for 16 PWM channel expansion.
A motion activated sensor at the top and bottom stair trigger a cascading LED effect which lights up each stair for a set
amount of time and then turns off.

See TLC tutorial here: http://tronixstuff.com/2013/10/21/tutorial-arduino-tlc5940-led-driver-ic/
                       http://www.learningaboutelectronics.com/Articles/TLC5940-PWM-driver-circuit-with-an-arduino.php
                       
Arduino pin 13 -> SCLK (TLC pin 25)
Arduino pin 11 -> SIN (TLC pin 26)
Arduino pin 10 -> Blank (TLC pin 23)
Arduino pin 9 -> XLAT (TLC pin 24)
Arduino pin 3 -> GSCLK (TLC pin 18)


*/

#include "Tlc5940.h"


/************************************************\
                    GLOBALS
\*************************************************/
const int TLC_MAX_PWM = 4095;
const int CUBIC = 0;
const int LINEAR = 1;


/************************************************\
                    TLC5940NT
\*************************************************/
// Mapping of TLC Outputs to stairs
const int STAIR_01 = 1;    // Top stair
const int STAIR_02 = 2;
const int STAIR_03 = 3;
const int STAIR_04 = 4;
const int STAIR_05 = 5;
const int STAIR_06 = 6;
const int STAIR_07 = 7;
const int STAIR_08 = 8;
const int STAIR_09 = 9;
const int STAIR_10 = 10;
const int STAIR_11 = 11;
const int STAIR_12 = 12;    // Bottom stair
const int TOP_STAIR = 1;
const int BOTTOM_STAIR = 1;

/************************************************\
                    Arduino
\*************************************************/
const int digitalInPin = 6;  // Digital input pin that the motion sensor or switch is attached to



/************************************************\
                    CONFIG
\*************************************************/
const int STAY_ON_TIME = 0;
const int STEP_DELAY = 0;
const int RAMP_DELAY = 25;
const int RAMP_FUNC = 0;            // 0 - cubic, 1 - linear


int cubic_func(float x, int scale) {
   // Convert animation progress to cubic smoothing function value
   // x:     0 <= x <= 1
   // scale: Maximum value of smoothing function, ie. smooth_func(1) 
   float q = float(x) / 100.0;
   return (3*q*q - 2 * q*q*q) * scale;
}

int linear_func(float x, int scale){
   // Convert animation progress to linear smoothing function value
   // x:     0 <= x <= 1
   // scale: Maximum value of smoothing function, ie. smooth_func(1) 
  int y;
  y = map(x, 0, 100, 0, scale);     
  return y;
}


void tlc_set(int channel, int progress) {
  int pwm_value;

  // If Vs is tied to Vcc and the TLC output is pulled high via a resistor to Vcc then 
  // the progress is reversed. ie TLC_MAX_PWM closes the MOSFET
  progress = 100 - progress;
  
  switch (RAMP_FUNC) {
    case CUBIC:
      pwm_value = cubic_func(progress, TLC_MAX_PWM);
    case LINEAR:
      pwm_value = linear_func(progress, TLC_MAX_PWM);      
  } 
  Tlc.set(channel, pwm_value);
  Tlc.update();
}

void setup() {
  Tlc.init(TLC_MAX_PWM);
  delay(5000);
  
  Serial.begin(9600);
  // Initialize the pushbutton pin as an input:
  pinMode(digitalInPin, INPUT);  
}





void loop() {
  int min_ii=0;
  int ii=min_ii;
  int jj=0;
  int trip=0;
  boolean running=false;
  int ramp_step=1;
  int pwm_value=0;
  
  // Check if the sensor was tripped
  trip = digitalRead(digitalInPin);
  Serial.println(trip);
//  // LED turn on / off sequence
  if (trip == 1) {
    Serial.println("start");    
    
    // For each stair
    for (jj=TOP_STAIR; jj <= BOTTOM_STAIR; jj+=1) {
          // Ramp UP      
          for (ii=0; ii <= 100; ii+=ramp_step) { 
            // change the analog out value:
            Serial.print(ii);
            Serial.print('\t');
//            Serial.print(pwm_value);
//            Serial.println();
            tlc_set(jj, ii);       
            delay(RAMP_DELAY);  
          };
      delay(STEP_DELAY);  
    };
    
    delay(STAY_ON_TIME);
    
    // For each stair
    for (int jj=TOP_STAIR; jj <= BOTTOM_STAIR; jj+=1) {
          // Ramp DOWN      
          for (ii=100; ii >= min_ii; ii-=ramp_step) { 
            // change the analog out value:
            Serial.print(ii);
            Serial.print('\t');
//            Serial.print(pwm_value);
//            Serial.println();
            tlc_set(jj, ii);       
            delay(RAMP_DELAY);  
          };
      delay(STEP_DELAY);  
    };
 
    // Reset the flag so the animation doesn't run again until it is tripped.    
    trip = 0;
    Serial.println("done");
  } // endif (trip == 1)
  else {
     Serial.println("No input detected");  
  };

   delay(100);                    
}
