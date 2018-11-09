
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



/************************************************\
                    CONFIG
\*************************************************/
const int STAY_ON_TIME_SEC = 3;
const int STEP_DELAY_MILLISEC = 0;
const int RAMP_DELAY_MILLISEC = 5;
const int RAMP_FUNC = 0;                  // 0 - cubic, 1 - linear
const int MAX_BRIGHTNESS_PERCENT = 50;
const int PULSE_LENGTH_uS = 50;
const int TRIGGER_DISTANCE_CM = 80;

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
const int STAIR_01 = 2;    
const int STAIR_02 = 3;
const int STAIR_03 = 4;
const int STAIR_04 = 5;
const int STAIR_05 = 6;
const int STAIR_06 = 7;
const int STAIR_07 = 8;
const int STAIR_08 = 9;
const int STAIR_09 = 10;
const int STAIR_10 = 11;
const int STAIR_11 = 12;
const int STAIR_12 = 13;

const int TOP_STAIR = 2;
const int BOTTOM_STAIR = 13;

/************************************************\
                    Arduino
\*************************************************/
// top and bottom of stairs trigger (Out) and echo (in) pins for Ultra-sonic sensor
const int echoPinTop = 50;  
const int trigPinTop = 51;  

const int echoPinBottom = 48;  
const int trigPinBottom = 49;  


const int trigManualPin = 40;


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

void setBrightness(int stair, int progress) {
  int pwm_value;

  switch (RAMP_FUNC) {
    case CUBIC:
      pwm_value = cubic_func(progress, 255);
    case LINEAR:
      pwm_value = linear_func(progress, 255);      
  } 
  // Serial.print("Setting channel " + String(channel) + " to " + String(pwm_value));
  analogWrite(stair, pwm_value); 

}

void sendPulse(int pin) {
  int silence = 10;
  digitalWrite(pin, LOW);
  delayMicroseconds(silence);
  
  digitalWrite(pin, HIGH);
  delayMicroseconds(PULSE_LENGTH_uS);
  
  digitalWrite(pin, LOW);
  delayMicroseconds(silence);
}

long getDistance_cm(int pin) {
  long duration, cm;
  duration = pulseIn(pin, HIGH);
  // Serial.println("Duration = " + String(duration) + " us");
  // The speed of sound is: 343m/s = 0.0343 cm/uS = 1/29.1 cm/uS
  cm = (duration / 2) / 29.1;
  Serial.println("Distance = " + String(cm) + " cm");
  
  return cm;
}

void setup() {
  // Tlc.init(TLC_MAX_PWM);
  delay(1000);
  
  Serial.begin(9600);
  
  pinMode(trigManualPin, INPUT);  
  pinMode(echoPinTop, INPUT);  
  pinMode(echoPinBottom, INPUT);  
  pinMode(trigPinTop, OUTPUT);  
  pinMode(trigPinBottom, OUTPUT);  
  // tlc_set(2, 100);
}





void loop() {
  int min_brightness=0;
  int brightness=min_brightness;
  int trip=0;
  boolean running=false;
  int ramp_step=1;
  int pwm_value=0;
  long distancecmTop, distancecmBottom;

    // Check if the sensor was tripped
  sendPulse(trigPinTop);
  distancecmTop = getDistance_cm(echoPinTop);  
  
  trip = distancecmTop < TRIGGER_DISTANCE_CM;
//  // LED turn on / off sequence
  if (trip == 1) {
    Serial.println("Starting sequence");    
    
    // For each stair
    for (int stair=TOP_STAIR; stair <= BOTTOM_STAIR; stair+=1) {
          // Ramp UP      
          for (brightness=0; brightness <= MAX_BRIGHTNESS_PERCENT; brightness+=ramp_step) { 
            setBrightness(stair, brightness);       
            delay(RAMP_DELAY_MILLISEC);  
          };
      delay(STEP_DELAY_MILLISEC);  
    };
    
    delay(1000 * STAY_ON_TIME_SEC);
    
    // For each stair
    for (int stair=TOP_STAIR; stair <= BOTTOM_STAIR; stair+=1) {
          // Ramp DOWN      
          for (brightness=MAX_BRIGHTNESS_PERCENT; brightness >= min_brightness; brightness-=ramp_step) { 
            setBrightness(stair, brightness);       
            delay(RAMP_DELAY_MILLISEC);  
          };
      delay(STEP_DELAY_MILLISEC);  
    };
 
    // Reset the flag so the animation doesn't run again until it is tripped.    
    trip = 0;
    Serial.println("Finished sequence");
  } // endif (trip == 1)
  else {
     Serial.println("No input detected");  
  };

   delay(100);                    
}

