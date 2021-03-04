/*
  Library for Teensy 3.6/Teensy 3.2 6.302 board
  Created by teaching staff of 6.302, a control class at MIT.
*/

#include "lib6302.h"

// Set Dac and Adc resolution, and PWM frequency for library
// Set PWM frequency, Teensy 8bit PWM: 3.6-234375, 3.2-187500!!!
// Divide by 2,4,8 or 16 for 9,10,11, or 12 bits.
// Warning 12 bit pwm in audio range for young listeners.

lib6302::lib6302(int pwmF, int dacRes, int adcRes, int T2TBaud) {
  _PWMF = pwmF;
  _dacRes = dacRes;
  _adcRes = adcRes;
  _T2TBaud = T2TBaud;
  _dacMax = float(pow(2,dacRes) - 1);
  _adcMax = float(pow(2,adcRes) - 1);
  _firstTime = true;
  _sinceToggle = 0;
  _inputString = "";
  _flipSign = 1.0;
}

extern ADC *_adc;
extern volatile uint16_t _bufMax;
extern volatile uint16_t _adcArray[];
extern volatile uint16_t _numS;

void adc0_isr(void) {
  if(_numS < _bufMax) {
    _adcArray[_numS] =
      (uint16_t) _adc->adc0->analogReadContinuous();
    _numS++; 
  } 
  else _adc->adc0->analogReadContinuous();
};

void lib6302::setupADC(int numAvg, boolean goFast)
{
  _adc->adc0->setAveraging(numAvg); 
  _adc->adc0->setResolution(_adcRes);
  if(goFast) {  // Go fast as possible
    _adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); 
    _adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); 
  } else { // A little slower but less noisy
    _adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); 
    _adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); 
  }
  _adc->adc0->stopContinuous();
  _adc->adc0->enableInterrupts(adc0_isr);
}


// In: ADC pin, num samples in a burst.
// Out: Avg val of burst, individual vals in _adcSampleArray.
int lib6302::_adcReadIn(int pin, uint16_t numSamples) {
  //  Don't overrun sampleArray.
  if(_numS > _bufMax) {
    _numS = _bufMax; 
  }
  
  // Wait until the samples collected
  _adc->adc0->wait_for_cal();
  _numS = 0;
  _adc->adc0->startContinuous(pin);
  while(_numS < numSamples) {}  // Wait to get numSamples.
  _adc->adc0->stopContinuous();

  // Average Samples
  uint32_t sumSample = 0;
  for(int i = 0; i < numSamples; i++) sumSample += _adcArray[i];
  return((int) (sumSample/numSamples));
}

// Scales 0->adcMax to 0.0->1.0.
float lib6302::adcGetAvg(int pin, uint16_t numSamples) {
  numSamples = min(_bufMax,numSamples);
  float avgADC = _adcReadIn(pin,numSamples)/_adcMax;
  return(avgADC);
}

// Gets numSamples, scales 0->adcMax to 0.0->1.0.
float lib6302::adcGetBuf(int pin, uint16_t numSamples, float sampBuf[]) {
  numSamples = min(_bufMax,numSamples);
  float avgADC = _adcReadIn(pin,numSamples)/_adcMax;
  for(int i = 0; i < numSamples; i++) sampBuf[i] = (_adcArray[i])/_adcMax;
  return(float(avgADC));
}

// Setup the 6302 library.
void lib6302::setup(String config_message, float togglePeriod) {
  
  T2Tserial.begin(_T2TBaud);
  _inputString = "";
  _firstTime = true;
  _config_message = config_message;
  _togglePeriod = int(1.0e6*togglePeriod);

  // Set up loop monitor pin
  pinMode(loopMonitor, OUTPUT);

  // Set up inputs
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(DAC0,OUTPUT);
  pinMode(DAC1,OUTPUT);

  // Set up outputs.
  pinMode(loopMonitor, OUTPUT);
  pinMode(nSleepPWM, OUTPUT);
  digitalWrite(nSleepPWM, LOW);
  analogWriteResolution(_dacRes); 
  analogWriteFrequency(pwmA1,_PWMF); 
  analogWriteFrequency(pwmA2,_PWMF); 
  analogWriteFrequency(pwmB1,_PWMF); 
  analogWriteFrequency(pwmB2,_PWMF); 
  analogWriteFrequency(pwmA1_T,_PWMF); 
  analogWriteFrequency(pwmA2_T,_PWMF); 
  analogWriteFrequency(pwmB1_T,_PWMF); 
  analogWriteFrequency(pwmB2_T,_PWMF);
  analogWrite(pwmA1,_dacMax); 
  analogWrite(pwmA2,_dacMax); 
  analogWrite(pwmB1,_dacMax); 
  analogWrite(pwmB2,_dacMax); 
  analogWrite(pwmA1_T,_dacMax); 
  analogWrite(pwmA2_T,_dacMax); 
  analogWrite(pwmB1_T,_dacMax); 
  analogWrite(pwmB2_T,_dacMax);

  analogWrite(DAC0, _dacMax/2);  // half way.
  analogWrite(DAC1, _dacMax/2);  // half way.
  
  // Turn on and off loop monitor,diagnostic
  digitalWrite(loopMonitor,HIGH);
  delay(10);
  digitalWrite(loopMonitor,LOW);
  delay(10);
  digitalWrite(loopMonitor,HIGH);

}

void lib6302::_processString(String inStr, float rcvBuf[]) {
  char St = inStr.charAt(0);
  inStr.remove(0,1);
  inStr.remove(0,1);
  float val = inStr.toFloat();
  switch (St) {
    case '0':
      rcvBuf[0] = val;
      //Serial.println(10.0);
      //Serial.println(val);
      break;
    case '1':
      rcvBuf[1] = val;
      break;
    case '2': 
      rcvBuf[2] = val;
      break;
    case '3':
      rcvBuf[3] = val;      
      break;  
    case '4':
      rcvBuf[4] = val;      
      break;  
    case '5':
      rcvBuf[5] = val;      
      break;
    case '6':
      rcvBuf[6] = val;      
      break;
    case '~':
      _firstTime = true;
      break;
    default:
    break;  
  }
}

// Load the serial output buffer.
void lib6302::sendStatus(int numStats, float stats[])
{
  int n = 0;
  // Start Byte.
  _sendBuf[n++] = '\f';
  _sendBuf[n++] = 'R';

  // status data
  int dataBytes = numStats*sizeof(stats[0]);
  memcpy(&(_sendBuf[n]),&(stats[0]),dataBytes);
  n += dataBytes;

  // CR to end status buffer.
  _sendBuf[n++] = '\n';
  
  // write to the serial line.
  T2Tserial.write(_sendBuf,n);
}

// Function to access the first time flag
boolean lib6302::isFirst()
{
  return(_firstTime);
}

float lib6302::getFlip()
{
  return(_flipSign);
}

float lib6302::getSlider(int whichVal)
{
  return(_rcvBuf[whichVal]);
}

float lib6302::getDacMax()
{
  return(_dacMax);
}

float lib6302::getAdcMax()
{
  return(_adcMax);
}

// Initializes the loop if this is the first time, or if reconnect sent
// from GUI.  Otherwise, just look for serial event.
void lib6302::startloop() {
  if (_firstTime) { // If first time, send config string
    _inputString = "";
    _firstTime = false;
    // Flush Transmitter
    T2Tserial.flush();
    // Clear out rcvr.
    while(T2Tserial.available()) {
      T2Tserial.read(); 
    }
    // Send the configuration info.
    T2Tserial.println(_config_message);    
  
    for(float timeOut = 0; !T2Tserial.available() && timeOut < 1e6; timeOut++);
    while(T2Tserial.available() > 0) T2Tserial.read(); // Clr Rcv
  } 
  else { // Check for data from GUI
    while (T2Tserial.available()) {
      char inChar = (char)T2Tserial.read();
      _inputString += inChar;
      if (inChar == '\n') { // newline terminates line.
        _processString(_inputString,_rcvBuf);
        _inputString = "";
        break;
      } else if(_inputString.length() > 50) { // Comm failure.
        _inputString = "";
        break;
      }
    }

    // Flip the monitor pin every loop
    _monitorSwitch = !_monitorSwitch;
    digitalWrite(loopMonitor, _monitorSwitch);

    // Flip the sign if its been a toggle period.
    if(int(_sinceToggle) > _togglePeriod) {
      _flipSign *= -1.0; 
      _sinceToggle = 0;
    }

  }
}

// Write motor commands to hbridge.
// normalized (0->1) to sign-flipped hbridge, clipped (0, dMax).
float lib6302::hbridgeWrite(float normV, int pin) {
  float normVclip = min(max(normV,0.0),1.0);
  int unclip = int(normV*float(_dacMax));
  analogWrite(pin, _dacMax - unclip);
  return(normVclip);
}

// normV from -1->1, for maximum negative voltage to maximum positive.
// On pinP: normV from -oLap->1-oLap maps to dMax->0, 
// On pinN: normV from -1+oLap->oLap maps to dMax->0
#define oLapF 0.00  //overlap forward and backward by 5%
float lib6302::hbridgeBipolar(float normV, int pinP, int pinN) {
  float normVp = hbridgeWrite(normV + oLapF, pinP);
  float normVm = hbridgeWrite(oLapF - normV, pinN);
  float retval = normVp;
  if (normVm > normVp) retval = -normVm;
  return(retval);
}

