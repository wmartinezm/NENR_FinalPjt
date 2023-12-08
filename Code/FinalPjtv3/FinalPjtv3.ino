#include <Keyboard.h>

#define BAUD_RATE 115200

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

#define BUFFER_SIZE 64
#define GAIN        7

int circular_bufferFlx[BUFFER_SIZE];
int circular_bufferExt[BUFFER_SIZE];
int data_indexFlx, sumFlx;
int data_indexExt, sumExt;

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

int16_t flexor = 0;
int16_t extensor = 0;
int16_t flx_raw_adc = 0;
int16_t ext_raw_adc = 0;
int16_t flx_filtered_signal = 0;
int16_t ext_filtered_signal = 0;
int16_t flx_envelop = 0;
int16_t ext_envelop = 0;

int16_t MovingAvg8(moving_average8_t *avg_data, int16_t latestData);

int flx_pulse_count = 0;
int ext_pulse_count = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  Keyboard.begin();
  delay(10);
}

void loop() {
  long startTime = micros();  // Start timer
  // Gets the raw ADC values and center it to 0V.
  flx_raw_adc = analogRead(A1);
  ext_raw_adc = analogRead(A2);
  // Filtering EMG signal
  flx_filtered_signal = EMG_FilterFlx(flx_raw_adc);
  ext_filtered_signal = EMG_FilterExt(ext_raw_adc);
  // Find the envelop from each signal.
  flx_envelop = EMG_Get_EnvelopFlx(abs(flx_filtered_signal));
  ext_envelop = EMG_Get_EnvelopExt(abs(ext_filtered_signal));
  // Additional filtering using moving average.
  flexor = MovingAvg8(&vFlexorAvg, (flx_envelop*GAIN)); 
  extensor = MovingAvg8(&vExtensorAvg, (ext_envelop*GAIN));
  // TODO: Following print lines are only for debug
  // Serial.print(flx_raw_adc);
  // Serial.print(",");
  // Serial.print(flx_filtered_signal);
  // Serial.print(",");
  // Serial.print(flx_envelop);
  // Serial.print(",");
  // Serial.println(flexor);
  // Serial.print(",");
  // Serial.print(ext_filtered_signal);
  // Serial.print(",");
  // Serial.print(ext_envelop);
  // Serial.print(",");
  // Serial.println(extensor);

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
          // Serial.println("N");
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

/***
* @brief  MovingAvg8 
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

/***
* @brief  EMG Envelope Detection
* @details        Extracts the enveloped amplitude of an electromyography (EMG) signal.
* @pre         Circular buffer and variables (circular_buffer, sum, data_index, BUFFER_SIZE)
* @post        Circular buffer and variables are updated.
* @param abs_emg   Absolute value of the EMG signal.
* @return      Enveloped value, representing the slowly varying amplitude of the EMG signal.
*               The result is the average of recent absolute EMG values in a circular buffer,
*               scaled by 2 to provide a smoothed representation of the signal's amplitude,
*               filtering out rapid fluctuations.
*/ 
inline int EMG_Get_Envelop(int abs_emg, int *data_index, int *sum, int *circular_buffer) {
    *sum -= circular_buffer[*data_index];
    *sum += abs_emg;
    circular_buffer[*data_index] = abs_emg;
    *data_index = (*data_index + 1) % BUFFER_SIZE;
    return (*sum/BUFFER_SIZE) * 2;
}

inline int EMG_Get_EnvelopFlx(int abs_emg) {
    return EMG_Get_Envelop(abs_emg, &data_indexFlx, &sumFlx, circular_bufferFlx);
}

inline int EMG_Get_EnvelopExt(int abs_emg) {
    return EMG_Get_Envelop(abs_emg, &data_indexExt, &sumExt, circular_bufferExt);
}

/**** @brief  EMG Filtering
* @details        Applies a cascaded second-order Infinite Impulse Response (IIR) filter 
*               to electromyography (EMG) signals for noise reduction and signal smoothing.
* @pre         Filter section state variables (z1, z2) are initialized.
* @post        Filter section state variables are updated for future calls.
* @param input   Input raw EMG signal value.
* @return      Filtered EMG signal value after passing through cascaded second-order IIR filters.
*               The filters are designed to suppress noise and enhance relevant signal components.
*               The specific filter characteristics are defined in four sections, each with its 
*               own state variables, determining the filter's behavior.
*/

float EMG_FilterFlx(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.03685183*z1 - 0.05446245*z2;
    output = 0.09398085*x + 0.18796170*z1 + 0.09398085*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.21112944*z1 - 0.49731521*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.69935958*z1 - 0.72648459*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.87363719*z1 - 0.89774590*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

float EMG_FilterExt(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.03685183*z1 - 0.05446245*z2;
    output = 0.09398085*x + 0.18796170*z1 + 0.09398085*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.21112944*z1 - 0.49731521*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.69935958*z1 - 0.72648459*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.87363719*z1 - 0.89774590*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}