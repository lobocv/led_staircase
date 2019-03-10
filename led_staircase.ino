//#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
#endif



/************************************************\
                    DEFINITIONS
\*************************************************/
const int TAPERED = 0;
const int LINEAR = 1;
const int QUADRATIC = 2;

const int PIR_SENSOR = 0;
const int ULTRASONIC_SENSOR = 1;

/************************************************\
                    CONFIG
\*************************************************/
const int STAY_ON_TIME_SEC         = 8;        // How long to stay on before ramp down sequence
const int STEP_DELAY_MILLISEC      = 0;         // Delay between steps turning on
const int RAMP_DELAY_UP_MILLISEC   = 2;        // How long between LED brightness increases
const int RAMP_DELAY_DOWN_MILLISEC = 2;        // How long between LED brightness decreases
const int RAMP_FUNC                = QUADRATIC; // Ramping function
const int RAMP_UP_STEP             = 1;         // How much to increase brightness on each step 
const int RAMP_DOWN_STEP           = 1;         // How much to decrease brightness on each step
const int OVERLAP                  = 3;         // The amount of overlapping between stairs turning on/off
const int MAX_PROGRESS             = 255;       // The amount of iterations of brightness for each step

const float MAX_BRIGHTNESS_PERCENT = 1.0;       // Maximum brightness reached by the LEDs
const float MIN_BRIGHTNESS_PERCENT = 0;         // Minimum brightness reached by the LEDs

const int PULSE_LENGTH_uS          = 50;        // Length of the ultra-sonic pulse
const int TRIGGER_DISTANCE_BOT_CM  = 80;        // Trigger LED when less than this distance at the bottom sensor
const int TRIGGER_DISTANCE_TOP_CM  = 80;        // Trigger LED when less than this distance at the top sensor

const int SENSOR_TYPE = PIR_SENSOR;

/************************************************\
                    Arduino Mappings
\*************************************************/

// top and bottom of stairs trigger (in) pins for Passive IR sensor
const int PIRPinTop = 50;
const int PIRPinBot = 51;

// top and bottom of stairs trigger (Out) and echo (in) pins for Ultra-sonic sensor
const int echoPinTop = 50;  
const int trigPinTop = 51;  

const int echoPinBot = 48;  
const int trigPinBot = 49;  

// Mapping of output pins to LEDs
const int STAIR_01 = 44;    
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

// const int N_STAIRS = 3;
// int STAIRS[N_STAIRS] = {STAIR_01, STAIR_02, STAIR_03};
const int N_STAIRS = 12;
int STAIRS[N_STAIRS] = {STAIR_01, STAIR_02, STAIR_03, STAIR_04, STAIR_05, STAIR_06, STAIR_07, STAIR_08, STAIR_09, STAIR_10, STAIR_11, STAIR_12};
int TOP_STAIR = STAIRS[0];
int BOTTOM_STAIR = STAIRS[N_STAIRS-1];



int stairsForward[N_STAIRS], stairsReversed[N_STAIRS];

/************************************************\
                    SETUP
\*************************************************/
void setup() {
  
  for (int ii=0; ii < N_STAIRS; ii++) {
    stairsForward[ii] = STAIRS[ii];
    stairsReversed[ii] = STAIRS[N_STAIRS-1-ii];
  }
  
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  
  if (SENSOR_TYPE == ULTRASONIC_SENSOR) {
    // Set up pin modes
    pinMode(echoPinTop, INPUT);  
    pinMode(echoPinBot, INPUT);  
    pinMode(trigPinTop, OUTPUT);  
    pinMode(trigPinBot, OUTPUT);
  } else if (SENSOR_TYPE == PIR_SENSOR) {
    pinMode(PIRPinTop, INPUT);
    pinMode(PIRPinBot, INPUT);
  }
  
  for (int ii=0; ii < N_STAIRS; ii++) {
    pinMode(STAIRS[ii], OUTPUT);
  }
}


/************************************************\
                    Main
\*************************************************/

void loop() {
  int trip;
  int stairs;  
  int sleep, sleepCheck=10;

  trip = is_tripped();
  
  if (trip != 0) {
    
    // Set the direction in which the LEDS turn on
    if (trip > 0) {
      DEBUG_PRINT ("Starting sequence from Top Stair");    
      stairs = stairsForward;

    } else {
      DEBUG_PRINT ("Starting sequence from Bottom Stair");    
      stairs = stairsReversed;
    }
 
    rampUpFlight(stairs);
    sleep = 0;
    while (sleep < STAY_ON_TIME_SEC*1000) {
      delay(sleepCheck);
      if (is_tripped() != 0) {
        sleep=0;
      }
      sleep += sleepCheck;
    }

    rampDownFlight(stairs);
 
    DEBUG_PRINT ("Finished sequence");
  } else {
    DEBUG_PRINT ("No input detected");
  };
  
  delay(20);
}


/************************************************\
                    LIBRARY
\*************************************************/

bool _rampUpDone(int ii, int extent) {
  return ii < extent;
}

bool _rampDownDone(int ii, int extent) {
  return ii > extent;
}


// Iterate over a group of stairs like a box-car filter, incrementing or decrementing
// their brightness
void _rampFlight(int direction, int* stairs) {
  int progress_overlap = MAX_PROGRESS / OVERLAP;
  int extent = (N_STAIRS+OVERLAP)*MAX_PROGRESS;
  int ii, progress, upto, gofrom, sleep;
  
    if (direction > 0) {
    ii = 0;
    sleep= RAMP_DELAY_UP_MILLISEC;
  } else if (direction < 0) {
    sleep = RAMP_DELAY_DOWN_MILLISEC;
  }
  
  while (ii < extent) {
    
    upto = (ii / progress_overlap);
    gofrom = upto - OVERLAP;
    if (gofrom < 0) {
      gofrom = 0;
    }
    if (upto > N_STAIRS-1) {
      upto = N_STAIRS-1;
    }
    
    for (int step=gofrom; step <= upto; step++) {
      progress = ii - step * progress_overlap;
      if (direction < 0) {
        progress = MAX_PROGRESS - progress;
      }

      if ((progress < 0) || (progress > MAX_PROGRESS)) {
        continue;
      }

      setBrightness(stairs[step], progress);  
      delay(sleep);    
      
    }
    
    ii++;
  }

  
}


// Iterate over a group of stairs like a box-car filter, incrementing or decrementing
// their brightness
void _rampFlight2(int direction, int* stairs) {
  bool (*iterating)(int, int) ;
  int brightness_overlap = MAX_PROGRESS / OVERLAP;
  int extent = (N_STAIRS+OVERLAP)*MAX_PROGRESS;
  int ii, end, brightness, upto, gofrom, sleep;
  
  
  if (direction > 0) {
    ii = 0;
    end = extent;
    iterating = _rampUpDone;
    sleep= RAMP_DELAY_UP_MILLISEC;
    direction = RAMP_UP_STEP;
  } else if (direction < 0) {
    ii = extent;
    end = N_STAIRS * MAX_PROGRESS -  extent;
    iterating = _rampDownDone;
    sleep = RAMP_DELAY_DOWN_MILLISEC;
    direction = -RAMP_DOWN_STEP;
  }
  
  while (iterating(ii, end)) {
    
    upto = (ii / brightness_overlap);
    gofrom = upto - OVERLAP;
    if (gofrom < 0) {
      gofrom = 0;
    }
    if (upto > N_STAIRS-1) {
      upto = N_STAIRS-1;
    }
    
    for (int step=gofrom; step <= upto; step++) {
      brightness = ii - step * brightness_overlap;

      if ((brightness < 0) || (brightness > MAX_PROGRESS)) {
        continue;
      }

      setBrightness(stairs[step], brightness);  
      delay(sleep);    
      
    }
    
    ii += direction;
  }

  
}


void rampUpFlight(int* stairs) {
  _rampFlight(1, stairs);
}

void rampDownFlight(int* stairs) {
  _rampFlight(-1, stairs);
}


void setBrightness(int pin, int progress) {
  int pwm_value, _max;
  if (progress < 0) {
    progress = 0;
  } else if (progress > MAX_PROGRESS) {
    progress = MAX_PROGRESS;
  }
  
  _max = MAX_BRIGHTNESS_PERCENT * 255;
  switch (RAMP_FUNC) {
    case QUADRATIC:
      pwm_value = quadratic_func(progress, _max);
    case TAPERED:
      pwm_value = tapered_func(progress, _max);
    case LINEAR:
      pwm_value = linear_func(progress, _max);      
  } 
  
  DEBUG_PRINT ("Progress = " + String(progress) + ": Setting channel " + String(pin) + " to " + String(pwm_value) + "\n");
  analogWrite(pin, pwm_value); 

}

int quadratic_func(int progress, int scale) {
   float q = float(progress) / MAX_PROGRESS;
   return q*q * scale;
}

int tapered_func(int progress, int scale) {
   float q = float(progress) / MAX_PROGRESS;
   return (3*q*q - 2 * q*q*q) * scale;
}

int linear_func(int progress, int scale) {
  int y;
  y = map(progress, 0, MAX_PROGRESS, 0, scale);     
  return y;
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
  // The speed of sound is: 343m/s = 0.0343 cm/uS = 1/29.1 cm/uS
  cm = (duration / 2) / 29.1;
  return cm;
}

// Check if the sensor was tripped at either the top of bottom sensor.
// Returns:
//   +1 if top sensor was tripped
//   -1 if bottom sensor was tripped
//    0 if neither were tripped
int is_tripped() {
  if (SENSOR_TYPE == ULTRASONIC_SENSOR) {
    long distancecmTop, distancecmBot;
    sendPulse(trigPinTop);
    distancecmTop = getDistance_cm(echoPinTop);  
    DEBUG_PRINT ("Top Distance = " + String(distancecmTop) + " cm");
    
    sendPulse(trigPinBot);
    distancecmBot = getDistance_cm(echoPinBot);  
    DEBUG_PRINT ("Bottom Distance = " + String(distancecmBot) + " cm");
      
    if (distancecmTop < TRIGGER_DISTANCE_TOP_CM) {
      return 1;
    } else if (distancecmBot < TRIGGER_DISTANCE_BOT_CM) {
      return -1;
    } else {
      return 0;
    };     
  } else if (SENSOR_TYPE == PIR_SENSOR) {
    int top, bot;
    top = digitalRead(PIRPinTop);
    DEBUG_PRINT ("Top PIR reading = " + String(top));
    bot = digitalRead(PIRPinBot);
    DEBUG_PRINT ("Bot PIR reading = " + String(bot));
    if (top > 0) {
      return 1;
    } else if (bot > 0) {
      return -1;
    } else {
      return 0;
    }
  }

}
