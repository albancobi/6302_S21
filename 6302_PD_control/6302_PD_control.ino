// ****************** MUST INCLUDE LINES BELOW*****************************
#include <lib6302.h>
lib6302 my6302(234375,12,16,1000000); //PWM Freq, DAC bits, ADC bits, 3.6<->3.2 rate
LIB6302_ADC(64);  // ADC sample buffer
// ****************** 6302 MUST INCLUDE LINES ABOVE*************************

// Constants for Users to Modify *************************************
#define togglePeriod 2.5 // toggles between +/- every period
#define theta_0 0  // Angle linearized about theta_A = 0 (horizontal).
#define cmdNom1 0.45   // Motor cmd to hold propeller at theta_0. originally 0.65
#define cmdNom2 0.5   // Motor cmd to hold second prop 
#define angleScale -6.28     // (NOTE SIGN) Scale to make rotation = 2*pi
#define mBack   10     // Number of sample points back for delta calc., originally 2
#define controllerUpdatePeriod 1000 // Update period in microseconds.
#define monitorUpdatePeriod controllerUpdatePeriod // monitor update period
// End Constants for Users to Modify **************************************

// *******SETTING UP THE MONITOR DISPLAY (SLIDERS AND PLOTS). *************
// Sliders
String sliders = "S~Theta_d (toggles when neg)~-0.75~0.75~S~Kp~0~10~S~Kd~0~10~";
#define TD 0
#define KP 1
#define KD 2
// Plots
String measDes = "P~Theta_d(blue) Theta_a(red)~-1.8~1.8~500~2~";
String deriv = "P~DThetaErr/dT~-100~100~500~1~";
//String Cmd = "P~MotorCmd1(blue) MotorCmd2(red)~-0.1~1.1~500~2~";
String Cmd = "P~MotorCmd~-0.1~1.1~500~1~";
String monPlots = measDes + deriv + Cmd;
#define PLOT_TD 0
#define PLOT_TA 1
#define PLOT_DThDT 2
#define PLOT_CMD 3
String config_message = "\fB~" + sliders + monPlots + "\r";
#define useBrowser true // false=Arduino Plotter, true=Browser Monitor 
// ********************FINISHED MONITOR SETUP*************************

// Arm lab pin definitions
#define anglePin A2
#define motorPWM_T pwmB2_T
#define motorNPWM_T pwmB1_T
#define motorPWM pwmB2
#define motorNPWM pwmB1

// Setup for PD control.
float pastThetaErr[mBack+1];  // Stores past errors in an array of [x] elements for deriv approx
void setup() {
  Serial.begin(115200);
  my6302.setup(config_message, togglePeriod);
  my6302.setupADC(8,false);  // 8 averages, not fast sampling.
  digitalWrite(nSleepPWM, HIGH);  // Turn on PWM driver
  for (int i = mBack; i >= 0; i--) pastThetaErr[i] = 0.0;
}

//********************** Main USER ALTERABLE Control Loop **********************
//**************************************************************************

elapsedMicros loopTimer; // loopTime used to force precise deltaT between starts
#define PRINTHEADROOM 1

void loop() {  
  // Initializes, gets GUI updates.
  my6302.startloop();

  // Headroom is the faction of the period still remaining after the loop completes.
  float headRoom = float(controllerUpdatePeriod - loopTimer)/float(controllerUpdatePeriod);
  headRoom = max(headRoom,0.0);
  
  // Start exactly one period since last start.
  while (int(loopTimer) < controllerUpdatePeriod) {};
  loopTimer = 0;

  // Scale the desired angle and flip its sign if toggling.
  float theta_d= my6302.getSlider(TD);
  if(theta_d < 0) theta_d *= my6302.getFlip();

  // Read angle sensor and compute error
  float theta_A = angleScale*(my6302.adcGetAvg(anglePin,8)-0.5);  // 8 averages of averaged read.
  float theta_a = theta_A - theta_0;  // Compute difference from nomimal
  float thetaErr = theta_d - theta_a; // Compute desired minus measured angle
  
  // Save backward angles for delta calculation.
  for (int i = mBack; i > 0; i--) pastThetaErr[i] = pastThetaErr[i-1];
  pastThetaErr[0] = thetaErr;
  
  // Compute the delta error scaled by the delta time (note usecs to seconds).
  /* float dThetaErrDt = (thetaErr - pastThetaErr[0])/(controllerUpdatePeriod*1.0e-6);  // FIX THIS!!! error is zero. look at line 84. */
  float dThetaErrDt = (thetaErr - pastThetaErr[mBack])/(controllerUpdatePeriod*1.0e-6);  // A.COBI
  float dCmd = my6302.getSlider(KP)*thetaErr + my6302.getSlider(KD)*dThetaErrDt;
                
  //float motorCmd = cmdNom2 - dCmd;  % For two props!
  float motorCmd_T = cmdNom1 + dCmd;
  
  // Write motor cmd to H-bridge.
  //float motorCmdClip = my6302.hbridgeWrite(motorCmd, motorPWM);  % For two props!
  float motorCmd_TClip = my6302.hbridgeWrite(motorCmd_T, motorPWM_T);

  // Send data to monitor.
  if (useBrowser && !my6302.isFirst()) {
    float loopStatus[4];
    loopStatus[PLOT_TD] = theta_d;
    loopStatus[PLOT_TA] = theta_a;
    loopStatus[PLOT_DThDT] = dThetaErrDt;
    loopStatus[PLOT_CMD] = motorCmd_TClip;
    my6302.sendStatus(4,loopStatus);  
  } 
  else Serial.println(theta_a); 
#if PRINTHEADROOM == 1
  Serial.println(headRoom);
#endif
}
