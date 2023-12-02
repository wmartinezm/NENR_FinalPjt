#include <Keyboard.h>

#define FLX_STATE_DETECT          0
#define FLX_STATE_CHECK           1
#define FLX_STATE_PRESS_LEFT_KEY  2
#define FLX_STATE_PRESS_DOWN_KEY  3
#define EXT_STATE_DETECT          0
#define EXT_STATE_CHECK           1
#define EXT_STATE_PRESS_RIGHT_KEY 2
#define EXT_STATE_PRESS_UP_KEY    3

#define FLX                     A1
#define EXT                     A2
#define FLX_THRESHOLD           700 // 1.75 V
#define EXT_THRESHOLD           700 // 1.75 V
#define shortPulseThreshold     180  // Adjust these thresholds based on your timing requirements
#define longPulseThreshold      400

uint8_t flx_state = FLX_STATE_DETECT;
uint8_t ext_state = EXT_STATE_DETECT;

uint16_t flx_elapsed_time = 0;
uint16_t ext_elapsed_time = 0;

int musclePin[2] = {   // define Arduino input locations (A2 and A1 are the inputs for the EMG shield)
  FLX,
  EXT
};
char printBuff[10];    // allocate space for reading voltages

typedef struct { 
  int16_t buffer[8];
  //Only sized to store 12-bit inputs 
  int32_t sum; 
  uint8_t position; 
  uint8_t isFilled; 
}moving_average8_t; 

moving_average8_t vFlexorAvg = {0}; 
moving_average8_t vExtensorAvg = {0};

uint16_t flexor = 0;
uint16_t extensor = 0;

int16_t MovingAvg8(moving_average8_t *avg_data, int16_t latestData);

int flx_pulse_count = 0;
int ext_pulse_count = 0;

void setup() {
  Serial.begin(9600);
  Keyboard.begin();
  delay(10);
}

void loop() {
  long startTime = micros();  // Start timer

  flexor = MovingAvg8(&vFlexorAvg, abs(analogRead(musclePin[0]))); 
  extensor = MovingAvg8(&vExtensorAvg, abs(analogRead(musclePin[1])));

  switch(flx_state)
  {
    case FLX_STATE_DETECT:
      if ((flexor > FLX_THRESHOLD) && (flx_elapsed_time > shortPulseThreshold))
      {
        flx_state = FLX_STATE_CHECK;
        flx_elapsed_time = 0;
      }
      break;
    case FLX_STATE_CHECK:
      if (flx_elapsed_time > shortPulseThreshold)
      {
        if ((flexor < FLX_THRESHOLD/2) 
        && (flx_elapsed_time < (shortPulseThreshold + shortPulseThreshold/2)))
        {
          flx_state = FLX_STATE_PRESS_LEFT_KEY;
        }
        else if ((flexor >FLX_THRESHOLD/2) 
        && (flx_elapsed_time > (shortPulseThreshold + shortPulseThreshold/2)))
        {
          flx_state = FLX_STATE_PRESS_DOWN_KEY;
        }
        else if (flx_elapsed_time > (longPulseThreshold + shortPulseThreshold))
        {
          // Serial.println("N");
          flx_elapsed_time = 0;
          flx_state = FLX_STATE_DETECT;
        }
      }	
      break;
    case FLX_STATE_PRESS_LEFT_KEY:
      Keyboard.write(KEY_LEFT_ARROW);
      Serial.println("L");
      // Serial.println(flx_elapsed_time);
      flx_elapsed_time = 0;
      flx_state = FLX_STATE_DETECT;
      break;
    case FLX_STATE_PRESS_DOWN_KEY:
      Keyboard.write(KEY_DOWN_ARROW);
      Serial.println("D");
      // Serial.println(flx_elapsed_time);
      flx_elapsed_time = 0;
      flx_state = FLX_STATE_DETECT;
      break;
  }

  switch(ext_state)
  {
    case EXT_STATE_DETECT:
      if ((extensor > EXT_THRESHOLD) && (ext_elapsed_time > shortPulseThreshold))
      {
        ext_state = EXT_STATE_CHECK;
        ext_elapsed_time = 0;
      }
      break;
    case EXT_STATE_CHECK:
      if (ext_elapsed_time > shortPulseThreshold)
      {
        if ((extensor < EXT_THRESHOLD/2) 
        && (ext_elapsed_time < shortPulseThreshold + shortPulseThreshold/2))
        {
          ext_state = EXT_STATE_PRESS_RIGHT_KEY;
        }
        else if ((extensor > EXT_THRESHOLD/2) 
        && (ext_elapsed_time > shortPulseThreshold + shortPulseThreshold/2))
        {
          ext_state = EXT_STATE_PRESS_UP_KEY;
        }
        else if (ext_elapsed_time > (longPulseThreshold + shortPulseThreshold))
        {
          Serial.println("N");
          ext_elapsed_time = 0;
          ext_state = EXT_STATE_DETECT;
        }
      }	
      break;
    case EXT_STATE_PRESS_RIGHT_KEY:
      Keyboard.write(KEY_RIGHT_ARROW);
      Serial.println("R");
      ext_elapsed_time = 0;
      ext_state = EXT_STATE_DETECT;
      break;
    case EXT_STATE_PRESS_UP_KEY:
      Keyboard.write(KEY_UP_ARROW);
      Serial.println("U");
      ext_elapsed_time = 0;
      ext_state = EXT_STATE_DETECT;
      break;
  }

  Keyboard.releaseAll();

  flx_elapsed_time++;
  ext_elapsed_time++;

  long stopTime = micros() - startTime; // Determine how long it took to process
  if (stopTime < 1000) {  // Force a maximum sampling rate of 1 kHz
    delayMicroseconds(1000 - stopTime);
  }
}

/**** @brief  MovingAvg8 
* @details        Moving average of 8 12-bit data points 
* @pre 
* @post 
* @param avg_data    moving_average8_t data structure 
* @param latestData  12-bit data input 
* @return  int16_t    Final averaged value 
*/ 
int16_t MovingAvg8(moving_average8_t *avg_data, int16_t latestData) 
{ 
  //Subtract the oldest number from the prev sum, add the new number   
  avg_data->sum = (avg_data->sum - avg_data->buffer[avg_data->position]) + latestData; 
  //Assign the latest data to the oldest position in the array   
  avg_data->buffer[avg_data->position] = latestData; 
  //Increment position pointer   
  avg_data->position++; 
  if (avg_data->position >= 8)
  //Always 8, this function is only for 8   
  { 
    avg_data->position = 0; 
    avg_data->isFilled = true; 
  } 
  //return the average, and don't consider unfilled spots in the math   
  return abs(avg_data->isFilled ? (avg_data->sum >> 3) : (avg_data->sum / avg_data->position));
  //Shift >> 3 for 8, 2 for 4 
} 

float EMGFilter(float input)
{
  static float z1, z2; // filter section state

  // Filter section 1
  {
    float x = input - 0.05159732*z1 - 0.36347401*z2;
    input = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }

  // Filter section 2
  {
    float x = input - -0.53945795*z1 - 0.39764934*z2;
    input = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }

  // Filter section 3
  {
    float x = input - 0.47319594*z1 - 0.70744137*z2;
    input = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }

  // Filter section 4
  {
    float x = input - -1.00211112*z1 - 0.74520226*z2;
    input = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }

  return input;
}