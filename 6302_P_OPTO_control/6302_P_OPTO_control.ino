// ****************** MUST INCLUDE LINES BELOW*****************************
#include <lib6302.h>
// Init library and set pwm freq, DAC resolution, ADC resolution, and teensy 3.6-3.2 bit rate.
// PWM frequency affects DAC resolution, higher frequency, lower resolution.
// For 8-bit PWM: Teensy 3.6-234375, Teensy 3.2-187500!!!
// Divide by 2,4,8 or 16 for 9,10,11, or 12 bits.
// WARNING 12 bit pwm is in audio range for young listeners.
lib6302 my6302(234375/4,12,16,1000000);
// Set up the analog-to-digital library, with a buffersize
LIB6302_ADC(500);  // ADC sample buffer of 500
// ****************** MUST INCLUDE LINES ABOVE*****************************

// Constants for Users to Modify 
// ************************************************************
#define togglePeriod 1.0 // seconds between +/- toggling of desired value.
#define nominalCmd 0.2 // Motor cmd for nominal speed.
#define nominalRPS 50.0 // Nominal Rotations per second.
#define ScaleCMD 250.0   // Scale factor: Cmd change -> rate of change of propeller RPS 
#define ticksPerPeriod 2 // See lab
#define ticksPerUpdate 2

// ************************************************************************
// End of Constants and code for Users to Modify 
// SEE END OF FILE FOR USER-ALTERABLE CONTROL LOOP!!!



// SETTING UP THE MONITOR DISPLAY (SLIDERS AND PLOTS)
// ************************************************************************
// Lab Specific monitor configurations are specified with a configuration string, 
// which specifies sliders and plots.
//
//  Configuration strings start with "\fB~" and ends with "\r"  
//  The character "~" is used as a delimiter between entries in the config string.
//
//  SLIDER format: "S~Slide_Title~minVal~maxVal~", if max-min <= 0 no display!!!
//  1st slider val:my6302.getSlider(0), 2nd slider val:my6302.getSlider(1), etc.
//  Use pound-defines to keep track of the order!!
String sliders = "S~Desired DeltaRPS (Neg to Toggle)~-20.0~20.0~S~Kff~0~10~S~Kp~0~100~";
#define DESIRED 0
#define KFF 1
#define KP 2
//
//  PLOT Format: "P~Title~yMin~yMax~xMax~#Traces~
//  NO MORE than two traces per plot (first is red, second blue)
//  Sending data to monitor in same the order as the plots, 
//  use pound-defines to keep track.
String BothErr = "P~Desired delRPS(blue) Measured delRPS(red)~-20.0~20.0~500~2~";
String Mcmd = "P~MotorCmd~-1.5~1.5~500~1~";
String monPlots = BothErr + Mcmd;
#define DESIREDPLOT 0
#define MEAS 1
#define CMD 2
//
// Now assemble the various configuration strings, and set monitor on.
String config_message = "\fB~" + sliders + monPlots + "\r";
#define useBrowser true // false=Arduino Plotter, true=Browser Monitor 
// ************************************************************************
//  Done with Monitor Display Setup*********************


// Set up interrupt handler for optical speed encoder
// ************************************************************************
#define failSafeCntMax 10
volatile int failSafeCnt = 0;
elapsedMicros senseMuSec = 0;  // microsecs since last interrupt.
volatile int lastDelta = 0;    // Previous interval length
volatile int deltaSum = 0;     // Accumulated intervals.
volatile int tickCount = 0;    // Interrupts since last speed calc.
volatile float measRPS = 0;    // Speed estimate from intervals.

volatile boolean setSpeedFlag = false;  // Flag set when spd calculated, 
                                        // cleared when used in loop.
void iservRPS() {
  int delta = int(senseMuSec);
  if( (failSafeCnt < failSafeCntMax) &&
       (delta < lastDelta/2 || delta > 2*lastDelta) ) {
    failSafeCnt += 1;  // throw away a few way-off deltas.
  } else {
    senseMuSec = 0;
    lastDelta = delta;
    deltaSum += delta;
    tickCount += 1;

    if(tickCount >= ticksPerUpdate) {
        setSpeedFlag = true;  
        failSafeCnt = 0;
        tickCount = 0;
        float scale = float(ticksPerUpdate)/float(ticksPerPeriod);
        measRPS = (scale*1.0e6)/float(deltaSum);
        deltaSum = 0;
        setSpeedFlag = true;    
    }
  }
}
// ************************************************************************
// Done with Set up of interrupt handler for optical speed encoder



// Control loop setup
// ************************************************************************

// Motor PWM and Sensor defines.
#define motorPWMpin pwmB1
#define motorNPWMpin pwmB2
#define nOptoSensor 7

void setup() {

  Serial.begin(115200);
  my6302.setup(config_message, togglePeriod);

  digitalWrite(nSleepPWM, HIGH);
  
  // Set up optical sensor interrupt routine.
  pinMode(nOptoSensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(nOptoSensor), iservRPS, FALLING);
}

//********************** Main USER ALTERABLE Control Loop **********************
//**************************************************************************
elapsedMicros loopTimer = 0;
#define MaxLoopWait 100000

void loop() {  
  
  // Initializes, gets GUI updates.
  my6302.startloop();
      
  // Wait until a Speed reading, or 
  while (!setSpeedFlag && (int(loopTimer) < MaxLoopWait)) {};
  setSpeedFlag = false; 
  loopTimer = 0;

  // Get slider desired value and possibly toggle.
  float desired = my6302.getSlider(DESIRED);
  float desiredDeltaRPS = desired;
  if(desired < 0) desiredDeltaRPS *= my6302.getFlip();
  
  // Compute the error and the motor cmd = nominal+Kff*desired+Kp*Err.
  float measuredDeltaRPS = measRPS - nominalRPS;
  float fdBack = my6302.getSlider(KP)*(desiredDeltaRPS - measuredDeltaRPS);
  float fdForward = my6302.getSlider(KFF)*desiredDeltaRPS;

  // Note scaling by the sensitivity of speed to cmd.
  float motorCmd = nominalCmd + (fdForward + fdBack)/ScaleCMD;
  
  // Write motor command to H-bridge, returns clipped value.
  motorCmd = my6302.hbridgeBipolar(motorCmd, motorPWMpin,motorNPWMpin); 

  // Send data to monitor.
  if (useBrowser && !my6302.isFirst()) {
    float loopStatus[3];
    loopStatus[DESIREDPLOT] = desiredDeltaRPS;
    loopStatus[MEAS] = measuredDeltaRPS;
    loopStatus[CMD] = motorCmd;
    my6302.sendStatus(3,loopStatus);
  } 
  else Serial.println(measuredDeltaRPS); 
}


  


  
