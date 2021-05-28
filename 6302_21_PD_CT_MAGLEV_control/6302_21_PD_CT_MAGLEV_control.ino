// ****************** MUST INCLUDE LINES BELOW*****************************
#include <lib6302.h>
//234375 8 bits,/2, 9bits, 14648.437 12 bits, 29296.875 11 bits, 58593.75 10 bits
lib6302 my6302(58593.75,12,16,1000000); //PWM Freq, DAC bits, ADC bits, 3.6<->3.2 rate
LIB6302_ADC(64);  // ADC sample buffer
// ****************** 6302 MUST INCLUDE LINES ABOVE*************************

// Constants for Users to Modify ******************************
// ***********************************************************


// Are we calibrating (just drive percentage of max) or feedback?
#define CALIBRATE 1 // 1 for calibration, 0 for Feedback Control. 
 
#define magPWM pwmA1_T  // Swap PWM and NPWM to change Feedback Sign
#define magNPWM pwmA2_T                   

#define filtAlpha 0.004 // Step input filter coefficient.


// End of Constants for Users to Modify ******************************
// *******************************************************************

float Offset1; 
float Offset2; 
#define twopi 3.1415926


#if CALIBRATE == 1
#define PlotUpdateRatio 10
#define controllerUpdatePeriod 100 // Greater than 100ns
#else
#define PlotUpdateRatio 10
#define controllerUpdatePeriod 100 // Greater than 50ns
#endif

#if CALIBRATE == 1
#define filtAlpha 0.5
#endif


// *******SETTING UP THE MONITOR DISPLAY (SLIDERS AND PLOTS). *************
String measCMD = "P~CMD~-1.2~1.2~500~1~";
String measS1mS2pS1 = "P~S1mS2(blue) ... S1(red)~-0.12~0.12~500~2~";
String measErr = "P~Err~-1.0~1.5~500~1~";
String sliderAlg = "S~Toggle Amp (Neg=Step,Pos=Sine)~-1.0~1.0~";
String sliderAsm = "S~Desired (Neg=Toggle)~-0.2~0.2~";
String sliderFpK = "S~Freq(Hz)~0.2~5~S~K0~0.1~10~";

#define SLIDER_AMP 0
#define SLIDER_HZ 1
#define SLIDER_K0 2

#if CALIBRATE ==  1
String config_message = "\fB~"+sliderAlg+sliderFpK+measCMD+measS1mS2pS1+"\r";
#else
String config_message = "\fB~"+sliderAsm+sliderFpK+measCMD+measErr+"\r";
#endif
#define PLOT_CMD 0
#define PLOT_ERR 1
#define PLOT_S1 2

#define useBrowser true // false=Arduino Plotter, true=Browser Monitor 
// ********************FINISHED MONITOR SETUP*************************


// Maglev pin definitions
#define S1Pin A2
#define desiredPin A0
#define errPin A1
#define CmdPin A3


float dacMax = 0;
float deltaTSecs = 0;

// Setup for PD control.
#define baudRate 1000000
void setup() {
  Serial.begin(baudRate);
  my6302.setup(config_message, 1.0);
  my6302.setupADC(4,true);  // 2 averages, fast sampling. 
  digitalWrite(nSleepPWM, HIGH);  // Turn on PWM driver
  deltaTSecs = float(controllerUpdatePeriod)*1.0e-6;
  dacMax = my6302.getDacMax();  
  analogWrite(DAC0, int(dacMax*0.5)); 
}

//********************** Main USER ALTERABLE Control Loop **********************
//**************************************************************************

elapsedMicros loopTimer; // loopTime used to force precise deltaT between starts
float oldDesired = 0.0;
int plotCycle = 0;
int loopCntr = 0;
int headRoom = controllerUpdatePeriod;

void loop() {  
  // Initializes, gets GUI updates.
  my6302.startloop();

  // Headroom is the faction of the period still remaining after the loop completes.
  int headRoomNow = controllerUpdatePeriod - int(loopTimer);
  headRoom = min(headRoom,headRoomNow);
  
  // Start exactly one period since last start.
  while (int(loopTimer) < controllerUpdatePeriod) {};
  loopTimer = 0;
  
  // Get the slider values.
  float desiredAmp = my6302.getSlider(SLIDER_AMP);
  float K0 = my6302.getSlider(SLIDER_K0);
  float desiredFreq = my6302.getSlider(SLIDER_HZ); 

  // Determine the fraction of a period = 1/desiredFreq.
  float periodFrac = float(loopCntr)*deltaTSecs*desiredFreq;
  if(periodFrac >= 1.0) loopCntr = 0;
  else loopCntr  += 1;
  
  // If desied<0 negative, flip sign for steps, with filter.
  if (desiredAmp < 0) {
    if (periodFrac > 0.5) desiredAmp *= -1.0;
    desiredAmp = (1-filtAlpha)*oldDesired + filtAlpha*desiredAmp;
    oldDesired = desiredAmp;
  }

  // Write the desired value to the DAC for analog desired value.
  // Note sign flip so that the analog summer sums (-desired)+(S1-S2).
#if CALIBRATE == 1
  float u = desiredAmp;
  analogWrite(DAC0, int(dacMax*(0.5))); 
#else
  // Read command
  float u = K0*(2*my6302.adcGetAvg(CmdPin,16)-1.0);  // Read analog command
  analogWrite(DAC0, int(dacMax*(0.5 - desiredAmp))); 
#endif
  // Write motor cmd to H-bridge.
  float uClip = my6302.hbridgeBipolar(u, magPWM,magNPWM); // For two props!

  // Send data to monitor.
  plotCycle += 1;
  if (plotCycle >= PlotUpdateRatio && useBrowser && !my6302.isFirst()) {
    plotCycle = 0;
    float loopStatus[3];
#if CALIBRATE == 0
    int numStats = 2;
#else  
    int numStats = 3;
    float S1 = (my6302.adcGetAvg(S1Pin,4)-0.5);  // Read angle.
    loopStatus[PLOT_S1] = S1;
#endif
    float err = (my6302.adcGetAvg(errPin,4)-0.5);  // Read angle.
    loopStatus[PLOT_CMD] = uClip;
    loopStatus[PLOT_ERR] = err;
    my6302.sendStatus(numStats,loopStatus); 
    headRoom = controllerUpdatePeriod;
  } 
}

 
