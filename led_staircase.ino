
/*
LED Stair Case Project:

Up to 16 LED strips can be controlled using the Arduino and TLC5940NT PWM IC. THE TLC5940 allows for 16 PWM channel expansion.
A motion activated sensor at the top and bottom stair trigger a cascading LED effect which lights up each stair for a set
amount of time and then turns off.

See TLC tutorial here: http://tronixstuff.com/2013/10/21/tutorial-arduino-tlc5940-led-driver-ic/

Arduino pin 13 -> SCLK (TLC pin 25)
Arduino pin 11 -> SIN (TLC pin26)
Arduino pin 10 -> Blank (TLC pin 23)
Arduino pin 9 -> XLAT (TLC pin 24)
Arduino pin 3 -> GSCLK (TLC pin 18)


*/

#include "Tlc5940.h"

/************************************************\
                    TLC5940NT
\*************************************************/
const int TLC_MAX_PWM = 4095;

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
const int BOTTOM_STAIR = 10;

/************************************************\
                    TLC5940NT
\*************************************************/
const int digitalInPin = 7;  // Digital input pin that the motion sensor or switch is attached to



/************************************************\
                    CONFIG
\*************************************************/
const int STAY_ON_TIME = 1000;
const int STEP_DELAY = 500;
const int RAMP_DELAY = 5;

int smooth_func(float x, int scale) {
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


void setup() {
  Tlc.init(0);
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
  
//  Serial.println(trip);
//  if (trip) {
//      // Trigger the LED to run it's turn on sequence
//     running = true;
//  }
  

//  // LED turn on / off sequence
  if (trip == 1) {
    Serial.println('Sensor Tripped!');
//    // Ramp UP
//    for (ii; ii < 100; ii+=step) { 
//      // change the analog out value:
//      pwm_value = smooth_func(ii, TLC_MAX_PWM);
//      for (int jj=TOP_STAIR; jj <= BOTTOM_STAIR; jj++) {
//        Tlc.set(jj, pwm_value);       
//        Tlc.update();
//      };
//      delay(RAMP_DELAY);
//    }  
    
    // For each stair
    for (jj=TOP_STAIR; jj <= BOTTOM_STAIR; jj+=1) {
          // Ramp UP      
          for (ii=0; ii < 100; ii+=ramp_step) { 
            // change the analog out value:
            pwm_value = smooth_func(ii, TLC_MAX_PWM);
            Serial.print(jj);
            Serial.print('\t');
            Serial.print(pwm_value);
            Serial.println();
            Tlc.set(jj, pwm_value);       
            Tlc.update();
            delay(RAMP_DELAY);  
          };
      delay(STEP_DELAY);  
    };
    
    delay(STAY_ON_TIME);
    

    // For each stair
    for (int jj=TOP_STAIR; jj <= BOTTOM_STAIR; jj+=1) {
          // Ramp DOWN      
          for (ii=100; ii > min_ii; ii-=ramp_step) { 
            // change the analog out value:
            pwm_value = smooth_func(ii, TLC_MAX_PWM);
            Serial.print(jj);
            Serial.print('\t');
            Serial.print(pwm_value);
            Serial.println();
            Tlc.set(jj, pwm_value);       
            Tlc.update();
            delay(RAMP_DELAY);  
          };
      delay(STEP_DELAY);  
    };
 
    
    trip = 0;
  } // endif (trip == 1)

   delay(100);                    
}
