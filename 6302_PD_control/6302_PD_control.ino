// ****************** MUST INCLUDE LINES BELOW*****************************
#include <lib6302.h>
lib6302 my6302(234375/4,12,16,1000000); //PWM Freq, DAC bits, ADC bits, 3.6<->3.2 rate
LIB6302_ADC(500);  // ADC sample buffer of 500
// ****************** 6302 MUST INCLUDE LINES ABOVE*************************

// Constants for Users to Modify *************************************
#define togglePeriod 3.5 // toggles between +/- every period
#define doToggle true   // true = desired value toggle
#define cmdNom1 0.5   // Motor cmd for nominal speed.
#define cmdNom2 0.5
#define angleOffset 0.5  // So that a horizontal prop is zero
#define angleScale 6.28     // (NOTE SIGN) Scale to make rotation = 2*pi
#define mBack  1           // Number of sample points back for delta calc.
#define controllerUpdatePeriod 1000 // Update period in microseconds.
#define monitorUpdatePeriod controllerUpdatePeriod // monitor update period
// End Constants for Users to Modify **************************************

// *******SETTING UP THE MONITOR DISPLAY (SLIDERS AND PLOTS). *************
// Sliders
String sliders = "S~Desired~-0.75~0.75~S~Kp~0~5~S~Kd~0~1~";
#define DES 0
#define KP 1
#define KD 2

// Plots
String measDes = "P~AngDesired(blue) AngMeas(red)~-1.8~1.8~500~2~";
String deriv = "P~DeltaAng/deltaT~-100~100~500~1~";
String Cmd = "P~MotorCmd1(blue) MotorCmd2(red)~-0.1~1.1~500~2~";
String monPlots = measDes + deriv + Cmd;
#define PLOT_DES 0
#define PLOT_MEAS 1
#define PLOT_DERIV 2
#define PLOT_CMD1 3
#define PLOT_CMD2 4

String config_message = "\fB~" + sliders + monPlots + "\r";
#define useBrowser true // false=Arduino Plotter, true=Browser Monitor 
// ********************FINISHED MONITOR SETUP*************************

// Arm lab pin definitions
#define anglePin A3
#define motorPWM_T pwmA2
#define motorNPWM_T pwmA1
#define motorPWM pwmB2_T
#define motorNPWM pwmB1_T

// Setup for PD control.
float pastMeasured[mBack+1];  // Stores past errors for deriv approx
elapsedMicros loopTimer; // loopTime used to force precise deltaT between starts
void setup() {
  Serial.begin(115200);
  my6302.setup(config_message, togglePeriod);
  my6302.setupADC(64,true);  // 64 averages, fast sampling.
  digitalWrite(nSleepPWM, HIGH);  // Turn on PWM driver
  for (int i = mBack; i >= 0; i--) pastMeasured[i] = 0.0;
}


//********************** Main USER ALTERABLE Control Loop **********************
//**************************************************************************
void loop() {  
  
  // Initializes, gets GUI updates.
  my6302.startloop();
      
  // Start exactly one period since last start.
  while (int(loopTimer) < controllerUpdatePeriod) {};
  loopTimer = 0;

  // Scale the desired input and flip its sign if toggling.
  float desired = my6302.getSlider(DES);
  if(desired < 0) desired *= my6302.getFlip();

  // Reads angle and compute error
  float measured = angleScale*(my6302.adcGetAvg(anglePin,1)-angleOffset);
  float cntrlErr = desired - measured;
  
  // Save backward errors for delta calculation.
  for (int i = mBack; i > 0; i--) pastMeasured[i] = pastMeasured[i-1];
  pastMeasured[0] = measured;
  
  // Compute the delta error scaled by the delta time (note usecs to seconds).
  float dMeasDt = (measured - pastMeasured[0])/(controllerUpdatePeriod*1.0e-6);  // FIX THIS!!!
  float dCmd = my6302.getSlider(KP)*cntrlErr - my6302.getSlider(KD)*dMeasDt;
                
  float motorCmd = cmdNom1 + dCmd;
  float motorCmd_T = cmdNom2 - dCmd;
  
  // Write motor cmd to H-bridge.
  float motorCmdClip = my6302.hbridgeWrite(motorCmd, motorPWM);
  float motorCmd_TClip = my6302.hbridgeWrite(motorCmd_T, motorPWM_T);

  // Send data to monitor.
  if (useBrowser && !my6302.isFirst()) {
    float loopStatus[5];
    loopStatus[PLOT_DES] = desired;
    loopStatus[PLOT_MEAS] = measured;
    loopStatus[PLOT_DERIV] = dMeasDt;
    loopStatus[PLOT_CMD1] = motorCmdClip;
    loopStatus[PLOT_CMD2] = motorCmd_TClip;
    my6302.sendStatus(5,loopStatus);
  } 
  else Serial.println(measured); 
}


  


  
