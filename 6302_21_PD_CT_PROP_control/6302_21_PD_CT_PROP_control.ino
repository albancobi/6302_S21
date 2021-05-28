// ****************** MUST INCLUDE LINES BELOW*****************************
#include <lib6302.h>
lib6302 my6302(234375,12,16,1000000); //PWM Freq, DAC bits, ADC bits, 3.6<->3.2 rate
LIB6302_ADC(64);  // ADC sample buffer
// ****************** 6302 MUST INCLUDE LINES ABOVE*************************

// Constants for Users to Modify *************************************
// MUST SET CALIBRATION COEFFIFICENTS!!!
#define angleOffset 0.0 // Offset for angle sense, -0.3->0.3
#define cmdOffset 0.0 // Offset for analog command -0.3->0.3
#define MODE 1 // 0=Control(Do not use), 1=Calibrate, 2=Sweep

#define cmdNom1 0.4  // Motor cmd to hold propeller at theta_0.1
#define cmdNom2 0.4 // Motor cmd to hold second prop 
#define angleScale 6.283   // Scale for two pi
#define analogCktCmdScale 10.0  // Divide by 10 in summer, divide input by 10.
#define analogCktReadScale 10.0  // Divide by 10 in summer, divide input by 10.
#define filtAlpha 0.005   //


#define default_Ampl 0.1 // Sweeper default amplitude
#define default_K0 4     // Sweeper Nominal Gain
// End Constants for Users to Modify **************************************


#if MODE == 0
#define controllerUpdatePeriod 100 // Update period in microseconds.
#define PlotUpdateRatio 10  // Number of updates before new plot
#else
#define controllerUpdatePeriod 200 // Update period in microseconds.
#define PlotUpdateRatio 5  // Number of updates before new plot
#endif

// *******SETTING UP THE MONITOR DISPLAY (SLIDERS AND PLOTS). *************
#if MODE == 2  // Sweeper
  // Frequency Sweep Parameters
  #define firstFreq 0.25
  #define lastFreq 4
  
  // Variables for Conversion
  #define deg2rad 0.0175
  #define twopi 6.2831853
  #define MAX_FREQ_PTS 30
  
  // Sweeper variables
  float freqs[MAX_FREQ_PTS];
  float Fmultiplier;
  float newFreq;
  boolean newFreqFlg;
  int freqIndex;
  int periodInDts;
  int numDts, numPeriods;
  float sa,ca, sc,cc, sd,cd;

  // break up in to substrings to avoid repetition
  String freqP = "P~Frequency~-0~8~90~1~";
  String magP = "P~Magnitude~-0.1~5.0~90~1~";
  String phaseP = "P~Phase~-300~10~90~1~";
  String freqCmdP = "P~FreqCmd~-0~10~90~1~";
  String magCmdP = "P~MagCmd~-0.1~20.0~90~1~";
  String phaseCmdP = "P~PhaseCmd~-200~200~90~1~";
  String monPlots = freqP + magP + phaseP + freqCmdP + magCmdP + phaseCmdP;
  #define PLOT_F 0
  #define PLOT_MAGA 1
  #define PLOT_PHASEA 2
  #define PLOT_FC 3
  #define PLOT_MAGC 4
  #define PLOT_PHASEC 5
  String config_message = "\fB~" + monPlots + "\r";

#else

  // Sliders
  String sliders = "S~Theta_d (neg for step, + sine)~-0.2~0.2~S~Freq(Hz)~0.1~4~S~K0~0~20~";
  #define SLIDE_TD 0
  #define SLIDE_HZ 1
  #define SLIDE_K0 2
  // Plots
  #define PLOT_TE 0
  #define PLOT_TD 1
  #define PLOT_SUM 2
  #define PLOT_TD2 4
  
  #if MODE == 0
  #define PLOT_CMD 1
  String monPlots = "P~Theta~-0.3~0.3~500~1~P~Motor Cmd~-0.6~0.6~500~1~";
  #else
  #define PLOT_CMD 3
  String monPlotsTheta = "P~Theta(Blue) Theta_d(R) Err+Off(G)~-0.5~0.5~500~3~";
  String monPlotsCmd = "P~MotorCmd(B) Theta_d(R)~-0.6~0.6~500~2~";
  String monPlots = monPlotsTheta + monPlotsCmd;
  #endif
  String config_message = "\fB~" + sliders + monPlots + "\r";
  
#endif

#define useBrowser true // false=Arduino Plotter, true=Browser Monitor 
// ********************FINISHED MONITOR SETUP*************************

#define twopi 6.2831853

// Arm lab pin definitions
#define anglePin A2
#define desiredPin A0
#define errPin A1
#define CmdPin A3
#define motorPWM_T pwmB1_T
#define motorNPWM_T pwmB2_T
#define motorPWM pwmB1
#define motorNPWM pwmB2

float dacMax = 0;
float deltaTSecs = 0;
float k0 = default_K0;
int tenSecInDt = 1000;

float headRoom = float(controllerUpdatePeriod);

// Setup for PD control.
#define baudRate 1000000
void setup() {
  Serial.begin(baudRate);
  my6302.setup(config_message, 1.0);
  my6302.setupADC(4,true);  // 8 averages, not fast sampling. 
  digitalWrite(nSleepPWM, HIGH);  // Turn on PWM driver
  deltaTSecs = float(controllerUpdatePeriod)*1.0e-6;
  tenSecInDt = int(10.0/deltaTSecs);
  dacMax = my6302.getDacMax();  
  headRoom = controllerUpdatePeriod;

  #if MODE == 2
  freqIndex = 0;
  newFreqFlg = true;
  newFreq = firstFreq;
  Fmultiplier = exp(log(lastFreq/firstFreq)/float(MAX_FREQ_PTS));
  #endif
}


//********************** Main USER ALTERABLE Control Loop **********************
//**************************************************************************

elapsedMicros loopTimer; // loopTime used to force precise deltaT between starts
float oldTheta_d = 0.0;
int plotCycle = 0;
long loopCntr = 0;


#if MODE == 2
float cosv = 0.0;
float sinv = 0.0;

float updateThetaDsweeper() {

    if(!newFreqFlg) numDts += 1; 
    else { // Initialize next freq pt, period MUST be integral multiple of deltaT
      newFreqFlg = false;
      periodInDts = 2*int(0.5/(deltaTSecs*newFreq));
      numPeriods = 0;
      if( (periodInDts < 10) || (freqIndex == MAX_FREQ_PTS) ) { // new Freq too high, restort
        newFreq = firstFreq;
        freqIndex = 0; 
        periodInDts = 2*int(0.5/(deltaTSecs*newFreq));
      }
      newFreq = float(1.0)/(deltaTSecs*float(periodInDts)); // Force period = N*deltaT 
      numDts = 0; sc = 0; cc = 0; sa = 0; ca = 0; sd = 0; cd = 0; 
    }
    float arg = (twopi*float(numDts))/float(periodInDts);
    cosv = cos(arg);
    sinv = sin(arg);
  
    // Compute next sine wave point, send to analog out
    return(default_Ampl*sinv);
}

float update_status_sweeper(float u, float theta_d) {

  float theta = angleScale*(my6302.adcGetAvg(anglePin,4)-0.5-angleOffset);  // Read angle.
  
  if(numDts < periodInDts) { // Not on period boundary, accumulate inintegral.
    sc += u*sinv; cc += u*cosv; 
    sa += theta*sinv; ca += theta*cosv;
    sd += theta_d*sinv; cd += theta_d*cosv;
  } 
  else { // On a period boundary.
    numPeriods += 1;
    if ( (numPeriods > 2)  &&  numPeriods*periodInDts > tenSecInDt) {
       newFreqFlg = true;  // more then 2 periods AND longer than 10 secs, new freq.
    } else {
       numDts = 0; sa = 0.0; ca = 0.0; sc = 0.0; cc = 0.0; sd = 0.0; cd = 0.0;
    }
  }

  if(newFreqFlg) {  //  Just finished a frequency, dump out results
    float phaseSys = atan2(ca,sa)/deg2rad;
    if(phaseSys > 0) phaseSys -= 360;
    float phaseCmd = atan2(cc,sc)/deg2rad;
    if(phaseCmd > 180) phaseCmd -= 360;
    float magD = sqrt(sd*sd + cd*cd);    
    float magC = sqrt(sc*sc + cc*cc)/magD;
    float magA = sqrt(sa*sa + ca*ca)/magD;

    if(!my6302.isFirst()) {
      float loopStatus[6];
      loopStatus[PLOT_F] = newFreq;
      loopStatus[PLOT_MAGA] = magA;
      loopStatus[PLOT_PHASEA] = phaseSys;
      loopStatus[PLOT_FC] = newFreq;
      loopStatus[PLOT_MAGC] = magC;
      loopStatus[PLOT_PHASEC] = phaseCmd;
      my6302.sendStatus(6,loopStatus);
    }
    newFreq *= Fmultiplier;
    freqIndex += 1;
    headRoom = controllerUpdatePeriod;
  } 
  return(theta);
}
#else // Not Sweep

float updateThetaD() {

    float theta_d = my6302.getSlider(SLIDE_TD);
    float desiredFreq = my6302.getSlider(SLIDE_HZ); 
  
    // Determine the fraction of a period = 1/desiredFreq.
    float periodFrac = float(loopCntr)*deltaTSecs*desiredFreq;
    if(periodFrac > 1.0) loopCntr = 0;
    else loopCntr  += 1;
    
    // Evaluate sinusoid, or flip sign for steps, with filter.
    if (theta_d > 0) theta_d *= sin(twopi*periodFrac); 
    else {
      if (periodFrac > 0.5) theta_d *= -1.0;
      theta_d = (1-filtAlpha)*oldTheta_d + filtAlpha*theta_d;
      oldTheta_d = theta_d;
    }
    return(theta_d);
}
#endif

void loop() {  
  
  // Initializes, gets GUI updates.
  my6302.startloop();

  // Headroom is the faction of the period still remaining after the loop completes.
  float headRoomNow = float(controllerUpdatePeriod - int(loopTimer));
  headRoom = min(headRoom,headRoomNow);
  
  // Start exactly one period since last start.
  while (int(loopTimer) < controllerUpdatePeriod) {};
  loopTimer = 0;

  // Get theta desired and send it to the analog circuit via DAC0
#if MODE == 2  // Frequency sweeper
  float theta_d = updateThetaDsweeper();  
  float k0 = default_K0;
#else  // Not the sweeper
  float theta_d = updateThetaD();  
  float k0 = my6302.getSlider(SLIDE_K0);
#endif
 
  // Send theta desired to the analog circuit via DAC0
  // Note sign flip and scale so that the analog summer sums (-theta_d)+theta.
  analogWrite(DAC0, int(dacMax*(0.5 - (analogCktCmdScale*(theta_d/angleScale)))));  

  // Read analog-circuit-generated motor command and scale by slider value K0

  float u = k0*(angleScale*analogCktCmdScale*(my6302.adcGetAvg(CmdPin,16)- 0.5) + cmdOffset);  // Read analog command

  // Write motor cmd to H-bridge.
  float mCmdClipCenter = my6302.hbridgeWrite(cmdNom2+u, motorPWM) - 0.5; // For two props!
  my6302.hbridgeWrite(cmdNom1-u, motorPWM_T);

#if MODE == 2  // The sweeper status update
  float theta = update_status_sweeper(u, theta_d);
#else  // Not the sweeper.

  // Send data to monitor.
  plotCycle += 1;
  if (plotCycle >= PlotUpdateRatio && useBrowser && !my6302.isFirst()) {
    float theta = angleScale*(my6302.adcGetAvg(anglePin,16)- 0.5) + angleOffset;
    float loopStatus[5];
    loopStatus[PLOT_TE] = theta;
    loopStatus[PLOT_CMD] = mCmdClipCenter;
  #if MODE == 0
    my6302.sendStatus(2,loopStatus);  
  #else
    float thetaErr = angleScale*analogCktReadScale*(my6302.adcGetAvg(errPin,5)-0.5);  
    //float theta_dr = (angleScale/analogCktCmdScale)*(my6302.adcGetAvg(desiredPin,5)-0.5);    
    loopStatus[PLOT_TD] = theta_d;
    loopStatus[PLOT_SUM] = thetaErr;
    loopStatus[PLOT_TD2] = theta_d;
    my6302.sendStatus(5,loopStatus);  
  #endif
     plotCycle = 0;
     headRoom = float(controllerUpdatePeriod);
  } 
#endif 
}
