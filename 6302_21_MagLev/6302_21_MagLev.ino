// ****************** MUST INCLUDE LINES BELOW*****************************
#include <lib6302.h>
lib6302 my6302(234375/4,12,16,1000000);
LIB6302_ADC(500); 
// ****************** MUST INCLUDE LINES ABOVE*****************************


// Constants for Users to Modify *************************************
// *******************************************************************
// Are we calibrating (just drive percentage of max) or feedback?
#define CALIBRATE 0 // 1 for calibration, 0 for Feedback Control.
#define togglePeriod 2.1 // toggles +/- every period, use 0.1 when Calibrating.

// End of Constants for Users to Modify ******************************
// *******************************************************************

#if CALIBRATE == 1
#define displayUpdatePeriod 1000 // No smaller than 1ms
#define controllerUpdatePeriod 1000 // Greater than 100ns
#else
#define displayUpdatePeriod 100 // No smaller than 1ms
#define controllerUpdatePeriod 100 // Greater than 50ns
#endif

//*********SETTING UP THE MONITOR SLIDERS AND PLOTS
//***************************************************
// Slider
String sliderDummy = "S~Space Holder~-0.5~0.0~";
String slider1 = "\fB~S~Toggle Amplitude~0~0.1~";
#if CALIBRATE == 1
String slider1 = "\fB~S~Toggle Amplitude~0~1.0~";
#endif
String sliders = slider1 + sliderDummy + sliderDummy;
#define TOGGLE 0

// Plots
String meas1_2_1p2 = "P~S1(blue) S2(red) S2+S1-Avg(green)~-0.1~1.1~500~3~";
String magErr = "P~Error~-1.0~1.0~500~1~";
String measCMD = "P~CMD~-1.1~1.1~500~1~";

#if CALIBRATE ==  1
String config_message = sliders + meas1_2_1p2 + measCMD + "\r";
#define MS1 0
#define MS2 1
#define MS1p2 2
#define CMD 3
#else
String config_message = sliders + magErr + measCMD + "\r";
#define MAGERR 0
#define CMD 1
#endif
#define useBrowser true // false=Arduino Plotter, true=Browser Monitor 
//*******************************************************************
// **************** DONE WITH MONITOR SETUP!!!***********************


// **************** Maglev Specific Defs *****************************
//***************************************************
#define magS1 A0
#define magS2 A1
#define magCmdSense A3
#define magPWM pwmA1_T
#define magNPWM pwmA2_T

#define baudRate 1000000
elapsedMicros loopTimer; // loopTime used to force precise deltaT between starts
// Set up function.
void setup() {
  Serial.begin(baudRate);
  my6302.setup(config_message, togglePeriod);
  my6302.setupADC(64,true);  // 64 averages, fast sampling.
  digitalWrite(nSleepPWM, HIGH);
}

  
//******************************** Main Control Loop *********************************
//************************************************************************************
void loop() {  
  // Initializes, gets GUI updates, and ensures exact deltaT between loops.
  my6302.startloop();
  
  // Start exactly one period since last start.
  while (int(loopTimer) < controllerUpdatePeriod) {};
  loopTimer = 0;

  // Send desired to analog out, and sign flip if desired negative.
  float desired = my6302.getSlider(TOGGLE);
  float dacMax = my6302.getDacMax();
  desired *= my6302.getFlip();
  
  // Reads angle and compute error
  float S1 = my6302.adcGetAvg(magS1,1);
  float S2 = my6302.adcGetAvg(magS2,1);
  float avgS1S2 = 0.5*(S1+S2);
  analogWrite(DAC0, int(avgS1S2*dacMax));
  #if CALIBRATE == 1
  analogWrite(DAC1, int(0.5*dacMax));
  float Cmd = desired;
  #else
  analogWrite(DAC1, int((desired+0.5)*dacMax));
  float Cmd = 2.0*(my6302.adcGetAvg(magCmdSense,1) -0.5);
  #endif
  
  // Write motor command to H-bridge, returns clipped value.
  float CmdSent = my6302.hbridgeBipolar(Cmd,magPWM,magNPWM); 

  // Send data to monitor.
  if (useBrowser && !my6302.isFirst()) {
#if CALIBRATE == 1
    float loopStatus[4];
    loopStatus[MS1] = S1;
    loopStatus[MS2] = S2;
    loopStatus[MS1p2] = avgS1S2;   
    loopStatus[CMD] = Cmd;
    my6302.sendStatus(4,loopStatus);
#else
    float loopStatus[2];
    loopStatus[MAGERR] = Cmd;  
    loopStatus[CMD] = CmdSent;
    my6302.sendStatus(2,loopStatus);
#endif
  } 
  else Serial.println(Cmd); 

  
}
  
