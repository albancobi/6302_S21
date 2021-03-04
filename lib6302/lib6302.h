/*
  Library for Teensy 3.6/Teensy 3.2 6.302 board
  Created by teaching staff of 6.302, a control class at MIT.
*/

#ifndef lib6302_h
#define lib6302_h

#include <ADC.h>

// Pin definitions.
#define T2Tserial Serial1  // use Pins 2,3 on Teensy 3.6 and 3.2.

#define nSleepPWM 2  // Turns Both PWM chips on and off.
// Dual Hbridge (A and B) with Crnt and Volt Sense
#define pwmA1 22
#define pwmA2 23
#define pwmB1 9
#define pwmB2 10

// Second Dual Hbridge (A_T and B_T) (no crnt sensing)
#define pwmA1_T 3
#define pwmA2_T 4
#define pwmB1_T 5
#define pwmB2_T 6

// Hbridge Crnt and Volt Sense Inputs 
#define IsenseB A7
#define IsenseA A6
#define VsenseA A5
#define VsenseB A4

// DACs
#define DAC1 A22  // A21,A22 for Teensy 3.6.
#define DAC0 A21

// Loop monitor Pin (for testing)
#define loopMonitor 24

// End Pin definitions.

// Definitions for ADC
#define LIB6302_ADC(BUF)\
ADC *_adc = new ADC(); \
volatile uint16_t _bufMax = BUF;\
volatile uint16_t _adcArray[BUF];\
volatile uint16_t _numS = 0;\

class lib6302
{
 public:
  lib6302(int pwmF, int dacRes, int adcRes, int T2TBaud);
  void setup(String config_message, float togglePeriod);
  float hbridgeWrite(float normV, int pin);
  float hbridgeBipolar(float normV, int pinP, int pinN);
  void sendStatus(int numStats, float stats[]);
  void startloop();
  bool isFirst();
  float getFlip();
  float getSlider(int whichVal);
  float getAdcMax();
  float getDacMax();
  void setupADC(int numAvg, boolean goFast);
  float adcGetAvg(int pin, uint16_t numSamples);
  float adcGetBuf(int pin, uint16_t numSamples, float sampBuf[]);
 private:
  void _processString(String inStr, float rcvBuf[]);
  int _adcReadIn(int pin, uint16_t numSamples);
  float _rcvBuf[10];
  char _sendBuf[100];
  int _dacRes;
  int _adcRes;
  float _dacMax;
  float _adcMax;
  int _PWMF;
  int _T2TBaud;
  float _flipSign = 1.0;
  boolean _firstTime;
  int _togglePeriod;
  elapsedMicros _sinceToggle = 0;
  boolean _monitorSwitch = false;
  String _config_message = "";
  String _inputString = "";
};

#endif
