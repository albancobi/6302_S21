// ****************** MUST INCLUDE LINES BELOW*****************************
#include <lib6302.h>
lib6302 my6302(234375/2,12,16,1000000); //PWM Freq, DAC bits, ADC bits, 3.6<->3.2 rate
LIB6302_ADC(64);  // ADC sample buffer
// ****************** 6302 MUST INCLUDE LINES ABOVE*************************

// Constants for Users to Modify *************************************
// macros that replace variable in code with value before compiling
#define togglePeriod 4.0 // toggles between +/- every period
#define theta_0 0.0  // Angle linearized about horizontal, adjust.
#define cmdNom1 0.62  // Motor cmd to hold propeller at theta_0.
#define cmdNom2 0.5 // Motor cmd to hold second prop 
#define angleScale -6.28     // (NOTE SIGN) Scale to make rotation = 2*pi
#define mBack 5 // Number of sample points back for delta calc.
#define controllerUpdatePeriod 1000 // Update period in microseconds.
#define NumProps 1 // Number of props being used
#define InputFilter 0 // Set to 1 to filter input

// End Constants for Users to Modify **************************************

// *******SETTING UP THE MONITOR DISPLAY (SLIDERS AND PLOTS). *************
// Sliders
String sliders1 = "S~Theta_d (toggles when neg)~-0.75~0.75~S~Kff~0~10~S~Kp~0~10~";
String sliders2 = "S~Kd~0~10~S~Ki~0~5~S~Dummy~0~1~";
String sliders = sliders1 + sliders2;
#define TD 0
#define KFF 1
#define KP 2
#define KD 3
#define KI 4
// Plots
String measDes = "P~Theta_d(blue) Theta_a(red)~-0.5~0.5~1000~2~";
String deriv = "P~DThetaErr/dT(blue) SumErr(red)~-10~10~250~2~";
#if NumProps == 2
String Cmd = "P~MotorCmd1(blue) MotorCmd2(red)~-0.1~1.1~250~2~";
#else
String Cmd = "P~MotorCmd~-0.1~1.1~250~1~";
#endif
String monPlots = measDes + deriv + Cmd;
#define PLOT_TD 0
#define PLOT_TA 1
#define PLOT_DThDT 2
#define PLOT_SUM 3
#define PLOT_CMD 4
#define PLOT_CMD2 5
String config_message = "\fB~" + sliders + monPlots + "\r";
#define useBrowser true // false=Arduino Plotter, true=Browser Monitor 
// ********************FINISHED MONITOR SETUP*************************

// Arm lab pin definitions
#define anglePin A2
#define motorPWM_T pwmB2_T
#define motorNPWM_T pwmB1_T
#define motorPWM pwmB1
#define motorNPWM pwmB2


// Setup for PD control.
float pastThetaErr[mBack+1];  // Stores past errors for deriv approx in an array of size mBack+1
#define baudRate 1000000
void setup() {
  Serial.begin(baudRate);
  my6302.setup(config_message, togglePeriod);
  my6302.setupADC(8,true);  // 8 averages, not fast sampling.
  digitalWrite(nSleepPWM, HIGH);  // Turn on PWM driver
  for (int i = mBack; i >= 0; i--) pastThetaErr[i] = 0.0;
}

//********************** Main USER ALTERABLE Control Loop **********************
//**************************************************************************

elapsedMicros loopTimer; // loopTime used to force precise deltaT between starts
float oldTheta_d = 0.0;
float oldOldTheta_d  = 0.0;

#define PRINTHEADROOM 1
float errSum = 0.0; // declare variable to keep track of error sum

// Main program loop
void loop() {  
  // Initializes, gets GUI updates.
  my6302.startloop();

  // Headroom is the faction of the period still remaining after the loop completes.
  float headRoom = float(controllerUpdatePeriod - loopTimer)/float(controllerUpdatePeriod);
  headRoom = max(headRoom,0.0);
  
  // Start exactly one period since last start.
  while (int(loopTimer) < controllerUpdatePeriod) {};
  loopTimer = 0;

  // Get the slider values.
  float theta_d = my6302.getSlider(TD);
  float kp = my6302.getSlider(KP);
  float kd = my6302.getSlider(KD);
  float ki = my6302.getSlider(KI);
  float kff = my6302.getSlider(KFF);

  // Scale the desired angle and flip its sign if toggling.
  if(theta_d < 0) theta_d *= my6302.getFlip();
  #if InputFilter == 1
  float p = 0.997;
  theta_d = p*oldTheta_d + (1-p)*theta_d; // discrete time low pass filter
  #endif
  oldTheta_d = theta_d;
  
  // Read angle sensor and compute error
  float theta_A = angleScale*(my6302.adcGetAvg(anglePin,1)-0.5);  // 8 averages of averaged read.
  float theta_a = theta_A - theta_0;  // Compute difference from nomimal
  float thetaErr = theta_d - theta_a; // Compute desired minus measured angle
  
  // Save backward errs for delta calculation.
  float dt = controllerUpdatePeriod*1.0e-6;
  for (int i = mBack; i > 0; i--) pastThetaErr[i] = pastThetaErr[i-1];
  pastThetaErr[0] = thetaErr;
  float dThetaErrDt = (thetaErr - pastThetaErr[mBack])/(mBack*dt);  
  
  // Sum the backward errors for sum calculation.
  float SumMax = 1.0;// FIX THIS, is this anti-windup?
  errSum = errSum + dt*thetaErr;  // FIX THIS, I think i fixed
  errSum = max(min(SumMax,errSum), -SumMax);

  // Compute the delta error scaled by the delta time (note usecs to seconds).
  float dCmd = kp*thetaErr + kd*dThetaErrDt + ki*errSum + kff*theta_d;

  // Write motor cmd to H-bridge.
#if NumProps == 2
  float motorCmd = cmdNom2 - dCmd;  // For two props!
  float motorCmdClip = my6302.hbridgeWrite(motorCmd, motorPWM); // For two props!
#endif
  float motorCmd_T = cmdNom1 + dCmd;
  float motorCmd_TClip = my6302.hbridgeWrite(motorCmd_T, motorPWM_T);

  // Send data to monitor.
  if (useBrowser && !my6302.isFirst()) {
    float loopStatus[4+NumProps];
    loopStatus[PLOT_TD] = theta_d;
    loopStatus[PLOT_TA] = theta_a;
    loopStatus[PLOT_DThDT] = dThetaErrDt;
    loopStatus[PLOT_SUM] = (float) errSum;
    loopStatus[PLOT_CMD] = motorCmd_TClip;
#if NumProps == 2
    loopStatus[PLOT_CMD2] = motorCmdClip;
#endif
    my6302.sendStatus(4+NumProps,loopStatus);  
  } 
  else Serial.println(theta_a); 
#if PRINTHEADROOM == 1
  Serial.println(headRoom);
#endif
}
