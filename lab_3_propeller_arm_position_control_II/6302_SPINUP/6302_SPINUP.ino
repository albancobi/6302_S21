// ****************** MUST INCLUDE LINES BELOW*****************************
#include <lib6302.h>
lib6302 my6302(234375/4,12,16,1000000);
LIB6302_ADC(500);  // ADC sample buffer of 500'
// ****************** MUST INCLUDE LINES ABOVE*****************************

// Constants for Users to Modify 
// ************************************************************
#define togglePeriod 1.0 // seconds between +/- toggling of desired value.
#define nominalCmd 0.5 // Motor cmd for nominal speed.
#define nominalRPS 150.0 // Nominal Rotations per second.
#define ScaleCMD 250.0 // The speed/cmd linearization estimate.
#define controllerUpdatePeriod 1000 // Update period in microseconds.
// ************************************************************************
// End of Constants and code for Users to Modify 
// SEE END OF FILE FOR USER-ALTERABLE CONTROL LOOP!!!

// SETTING UP THE MONITOR DISPLAY (SLIDERS AND PLOTS)
// ************************************************************************
String sliders = "S~Desired DeltaRPS (Neg to Toggle)~-20.0~20.0~S~Kff~0~10~S~Dummy~0~1~";
#define DESIRED 0
#define KFF 1

String BothErr = "P~Des delRPS(blue) Meas delRPS(red)~-20.0~20.0~500~2~";
String Mcmd = "P~MotorCmd~-1.5~1.5~500~1~";
String monPlots = BothErr + Mcmd;
#define DESIREDPLOT 0
#define MEAS 1
#define CMD 2

// Now assemble the various configuration strings, and set monitor on.
String config_message = "\fB~" + sliders + monPlots + "\r";
#define useBrowser true // false=Arduino Plotter, true=Browser Monitor 
// ************************************************************************
//  Done with Monitor Display Setup*********************


// Calculate the backEMF = V_motor - R_motor*I_motor.
// Uses generic analog read function to avoid interrupt conflicts between 
// tach and ADC.
#define offsetMotorV  0.5 
#define scaleMotorV 2.0   // vMotor 
#define scaleMotorC 0.66  // iMotor = (V(iSense)/6)/0.25ohms = 0.66
#define Rm 0.37

#define Bemf2RPS 1425
float getBackEMF(int pinV, int pinC,int nAvg) {
  // Read motor current and motor voltage.
  float sumBemf = 0.0;
  for(int i = 0; i < nAvg; i++) {
    float adcMax = my6302.getAdcMax(); 
    float absMotorC = scaleMotorC*my6302.adcGetAvg(pinC,1); // Sensor reads abs(crnt).
    float motorV = scaleMotorV*(my6302.adcGetAvg(pinV,1) - offsetMotorV);
    
    // BackEMF is motorV minus Rmotor*absMotorC, but signs flip if motorV < 0.
    float backEMF = motorV;
    float RmDrop = absMotorC*Rm;
    backEMF -= ((backEMF > 0.0) ? RmDrop : -RmDrop); 
    sumBemf += backEMF;
  }
  return(Bemf2RPS*sumBemf/float(nAvg));
}

// Control loop setup
// ************************************************************************
// Motor PWM and Sensor defines.
#define motorPWMpin pwmB1
#define motorNPWMpin pwmB2

void setup() {
  digitalWrite(nSleepPWM, LOW);
  Serial.begin(115200);
  my6302.setup(config_message, togglePeriod);
  my6302.setupADC(16,true);  // 64 averages, fast sampling.
  digitalWrite(nSleepPWM, HIGH);
}


//********************** Main USER ALTERABLE Control Loop **********************
//**************************************************************************
elapsedMicros loopTimer = 0;

void loop() {  
  
  // Initializes, gets GUI updates.
  my6302.startloop();
  float headroom = controllerUpdatePeriod - int(loopTimer);
  while (int(loopTimer) < controllerUpdatePeriod) {};
  loopTimer = 0;

  // Get slider desired value and possibly toggle.
  float desired = my6302.getSlider(DESIRED);
  float desiredDeltaRPS = desired;
  if(desired < 0) desiredDeltaRPS *= my6302.getFlip();
  float fdForward = my6302.getSlider(KFF)*desiredDeltaRPS;
    
  // Compute the error and the motor cmd = nominal+Kff*desired+Kp*Err.
  float RPSbemf = getBackEMF(VsenseB,IsenseB,2);
  float RPSbemfDelta = RPSbemf - nominalRPS;
  float RPSbemfDeltaBackwards = RPSbemf + nominalRPS;
  if(abs(RPSbemfDeltaBackwards) < abs(RPSbemfDelta)) {
    RPSbemfDelta = RPSbemfDeltaBackwards;
  }

  // Note scaling by the sensitivity of speed to cmd, adjusts operating point
  float motorCmd = nominalCmd + fdForward/ScaleCMD;
  
  // Write motor command to H-bridge, returns clipped value.
  motorCmd = my6302.hbridgeBipolar(motorCmd, motorPWMpin,motorNPWMpin); 

  // Send data to monitor.
  if (useBrowser && !my6302.isFirst()) {
    float loopStatus[3];
    loopStatus[DESIREDPLOT] = desiredDeltaRPS;
    loopStatus[MEAS] = RPSbemfDelta;  
    loopStatus[CMD] = motorCmd;
    my6302.sendStatus(3,loopStatus);
  } 
  else Serial.println(RPSbemfDelta); 
}


  


  
  


  
