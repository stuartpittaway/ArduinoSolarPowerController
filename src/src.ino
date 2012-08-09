/* A self-contained algorithm for routing surplus PV power to a dump load.  Uses an 
 * 'energy bucket' to model the operation of a digital supply meter.  Energy measurements 
 * at the supply point are accumulated over each individual mains cycle and added 
 * directly to the bucket.  A safety margin is included in the form of a steady leak in 
 * the bucket.  
 *
 * The energy level is checked every time that the bucket has been updated.  If the 
 * level is above the threshold value, an output pin is set; otherwise it is cleared. 
 * By use of an external trigger device that detects zero-crossings, this signal can be used to 
 * control a triac which will 'fire' or 'clear' at the next zero-crossing point.  The trigger 
 * pin is active low; it operates in parallel with the on-board LED (pin 13) which is 
 * active high.
 * 
 * Although measurements of power are accumulated between +ve going crossings, the triac 
 * operates between consecutive -ve going crossings.  This interleaved behaviour ensures the 
 * fastest possible response time for whole-cycle operation of the triac.
 *
 * The standard high-pass filter is retained only for determining the start of each new
 * cycle of the mains voltage.  DC is removed from the raw V & I samples by 
 * subtracting the known offset as determined by a single low-pass filter.  This LPF is 
 * updated once every mains cycle with the net dc offset of all raw voltage samples taken
 * during the previous cycle.  This approach requires the use of a single (buffered) reference 
 * which is used for both the voltage and current sensors.
 *
 * Each iteration of loop() corresponds to a single pair of V & I measurements, of which 
 * there are several dozen per mains cycle.  This code contains no periodicity greater than 
 * one mains cycle (20mS) and is intended for continuous operation, preferably without use of
 * Serial statements which could disrupt the flow.
 *
 * As it is *energy* which is of prime importance to this application, no independent calibration 
 * of voltage and current measurements has been included.  Having removed the DC-offset, 
 * 'raw' V & I samples are multipled together to give 'raw' power, these instantaneous values 
 * being accumulated during each mains cycle to a measure of'raw' energy.  A single calibration 
 * factor, POWERCAL, is then applied to convert this value into Joules before being added to the 
 * energy bucket.  Its value is not critical because any absolute inaccuracy will cancel
 * out when import and export are constrained to be equal, as this algorithm does. 
 * 
 * A secondary calibration factor, VOLTAGECAL, is used purely for determining the correct point
 * in the voltage waveform when the external trigger device may be safely armed. Each of these
 * calibration factors is described in more detail when their values are assigned.
 *
 *                  Robin Emley (calypso_rae on Open Energy Monitor Forum)
 *                  July 2012
 */

/*
  Modified by Stuart Pittaway, 17 July 2012/9 Aug 2012
 Original version compiles to 6858 bytes with debugging on, new version is 6984, but reduced RAM footprint.
 changed data types to be ANSI C/portable, included digitalwritefast library.
 Changed various variables into hardcoded define statements to save program space
 
 PINS changed to run on Arduino UNI compat board (I'm using breadboard!)
 
 */

// for normal operation, the next line should be commented out
//#define DEBUG


//#define POWERCAL 0.085F
// Joules per ADC-unit squared.  Used for converting the product of 
// voltage and current samples into Joules.
// To determine this value, note the rate that the energy bucket's
// level increases when a known load is being measured at a convenient
// test location (e.g  using a mains extention with the outer cover removed so that 
// the current-clamp can fit around just one core.  Adjust POWERCAL so that
// 'measured value' = 'expected value' for various loads.  The value of
// POWERCAL is not critical as any absolute error will cancel out when 
// import and export flows are balanced.  

// Volts per ADC-unit. (1.44)
//#define VOLTAGECAL 1.44F
// This value is used to determine when the voltage level is suitable for 
// arming the external trigger device.  To set this value, note the min and max
// numbers that are seen when measuring 240Vac via the voltage sensor, which 
// is 678.8V p-t-p.  The range on my setup is 471 meaning that I'm under-reading 
// voltage by 471/679.  VOLTAGECAL therefore need to be the inverse of this, i.e.
// 679/471 or 1.44

//Calibrated by Stuart Aug 8th 2012, on breadboard circuit!
#define VCAL 86.45
#define ICAL 105.38
#define PHASECAL 1.7

/*
These values work on emonTX
#define VCAL 236.16
#define ICAL 105.38
#define PHASECAL 1.7
*/

//Added in digitalWriteFast from http://code.google.com/p/digitalwritefast
#include "digitalWriteFast.h"

#include "TimerOne.h"

// define the input and output pins
#define outputPinForLed       13
#define outputPinForTrigger   7

//eMonTX uses A2 and A3 for voltage and CT1 sockets
#define voltageSensorPin      A2
#define currentSensorPin      A3  

#define POSITIVE 1
#define NEGATIVE 0
#define ON 0        // the external trigger device is active low
#define OFF 1

// use float to ensure accurate maths, mains frequency (Hz)
#define cyclesPerSecond   50.0F 

#define SUPPLYVOLTAGE 5313

// 0.001 kWh = 3600 Joules
#define capacityOfEnergyBucket 3600

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


volatile double 
       apparentPower,
       powerFactor,
       Vrms,
       Irms;

float safetyMargin_watts = 0;  // <<<-------  Safety Margin in Watts (increase for more export)
uint32_t cycleCount = 0;
uint16_t samplesDuringThisMainsCycle = 0;
uint8_t nextStateOfTriac = OFF;

volatile uint8_t polarityNow = NEGATIVE; // probably not important, but better than being indeterminate
volatile bool beyondStartUpPhase=false;
volatile unsigned long msperzerocrossing=0;
volatile unsigned long previousmsperzerocrossing=0;

volatile int16_t voltageAtZeroCross=0;

volatile uint8_t bouncecounter=0;
boolean triggerNeedsToBeArmed = false;

// the 'energy bucket' mimics the operation of a digital supply meter at the grid connection point.
float energyInBucket = 0;                                                 

// Local declarations of items to support code that I've lifted from calcVI(), in EmonLib. 

int16_t sampleV,sampleI;   // voltage & current samples are integers in the ADC's input range 0 - 1023 
//int16_t lastSampleV;     // stored value from the previous loop (HP filter is for voltage samples only)         

double filteredV;  //  voltage values after filtering to remove the DC offset, and the stored values from the previous loop             
double prevDCoffset;          // <<--- for LPF to quantify the DC offset
double DCoffset;              // <<--- for LPF 
double sumP;                  //  cumulative sum of power calculations within this mains cycles

//uint8_t dutyCycle;
//volatile uint8_t safe_dutyCycle;

unsigned long interrupt_timing=0;

//volatile bool positive=true;
//volatile uint16_t testsample=0;
volatile double cumVdeltasThisCycle;   // <<--- for LPF 
volatile double realPower=0;

volatile uint16_t xxxxsamplesDuringThisMainsCycle=0;
//sq = squared, sum = Sum, inst = instantaneous
volatile double sqV,sumV,sqI,sumI;

void setup()
{  
  Serial.begin(115200);  //Faster baud rate to reduce timing of serial.print commands

  pinModeFast(outputPinForTrigger, OUTPUT);  
  pinModeFast(outputPinForLed, OUTPUT);  

  pinModeFast(10, OUTPUT);  
  pinModeFast(9, OUTPUT);  

  pinModeFast(voltageSensorPin, INPUT);
  pinModeFast(currentSensorPin, INPUT);

  analogReference(DEFAULT);  //5v

  Timer1.initialize();

  //STUART ADDED.... EXPERIMENAL FASTER ANALOGUE READ ON ARDUINO
  //NEEDS TO BE TESTED COMMENT OUT FOR NORMAL OPERATION! 
  //set prescale as per forum on http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  //table of prescale values... http://i187.photobucket.com/albums/x269/jmknapp/adc_prescale.png
  //Prescale 32
  sbi(ADCSRA,ADPS2);  //ON
  cbi(ADCSRA,ADPS1);  //OFF
  sbi(ADCSRA,ADPS0);  //ON

  //Start the interrupt and wait for the first zero crossing...
  attachInterrupt(1, positivezerocrossing, RISING );
}

//int loopcount=0;
void loop() // each loop is for one pair of V & I measurements
{  
        Serial.print("cyc# ");
        Serial.print(cycleCount);
        Serial.print(",time ");
        Serial.print(msperzerocrossing/1000.0);
        Serial.print(",Hz ");
        Serial.print(1000000.0/msperzerocrossing);
        Serial.print(",samp ");
        Serial.print(xxxxsamplesDuringThisMainsCycle);
        
        Serial.print(",intT ");
        Serial.print(interrupt_timing);       
        
        //Serial.print(", Vzc ");
        //Serial.print(voltageAtZeroCross);
        
        Serial.print(",sampleV ");
        Serial.print(sampleV);
        Serial.print(",fltdV ");
        Serial.print(filteredV);
        //Serial.print(", sampleV-dc ");
        //Serial.print((int)(sampleV - DCoffset));
        //Serial.print(", cumVdeltas ");
        //Serial.print(cumVdeltasThisCycle);
        //Serial.print(", prevDCoffset ");
        //Serial.print(prevDCoffset);
        Serial.print(",refFltdV ");
        Serial.print(DCoffset);
        
        Serial.print(",Vrms ");
        Serial.print(Vrms);
        Serial.print(",Irms ");
        Serial.print(Irms);
        Serial.print(",realPwr ");
        Serial.print(realPower);
        Serial.print(",app.Pwr ");
        Serial.print(apparentPower);
        Serial.print(",pwrF ");
        Serial.print(powerFactor);
        
        Serial.print(",P=");
        Serial.print(realPower);
        
        Serial.print(",enInBkt ");
        Serial.println(energyInBucket);

  //Serial.print(SUPPLYVOLTAGE);
  
/*  
  Serial.print(" V=");
  Serial.print(sampleV);
  Serial.print(" I=");
  Serial.print(sampleI);
  
  Serial.print(" F=");
  Serial.print(msperzerocrossing);  
  Serial.print(" Hz=");
  Serial.print(1000000.0/msperzerocrossing); 
  Serial.print(" P=");
  Serial.print(realPower);
  Serial.print(" S=");
  Serial.print(xxxxsamplesDuringThisMainsCycle);
  Serial.print(" E=");
  Serial.print(energyInBucket);

  //This is a timing of the interrupt routine in microseconds
  Serial.print("  int time=");
  Serial.print(interrupt_timing);

  Serial.println();
  */

  digitalWrite(outputPinForLed, HIGH);  // active high
  delay(5);  //small delay
  digitalWrite(outputPinForLed, LOW);  
  delay(250);  //small delay
} // end of loop()

/*
long readVcc() {
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;
  return result;
}
*/

void positivezerocrossing()
{
  digitalWriteFast(10, HIGH);
  
  //Read in raw voltage signal - just for use whilst calibrating!
  //voltageAtZeroCross = analogRead(voltageSensorPin);                 
   
  Timer1.detachInterrupt();

  msperzerocrossing=micros()-previousmsperzerocrossing;
  previousmsperzerocrossing=micros();

  xxxxsamplesDuringThisMainsCycle=samplesDuringThisMainsCycle;

  cycleCount++; // for stats only

  //-------------------------------------------------------------------------------------------------------------------------
  // Post loop calculations
  //------------------------------------------------------------------------------------------------------------------------- 
  //Calculation of the root of the mean of the voltage and current squared (rms) and Calibration coeficients applied. 
  
  //SUPPLYVOLTAGE = readVcc();
  
  double V_RATIO = VCAL *((SUPPLYVOLTAGE/1000.0) / 1023.0);
  Vrms = V_RATIO * sqrt(sumV / (float)samplesDuringThisMainsCycle); 
  
  double I_RATIO = ICAL *((SUPPLYVOLTAGE/1000.0) / 1023.0);
  Irms = I_RATIO * sqrt(sumI / (float)samplesDuringThisMainsCycle); 

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / (float)samplesDuringThisMainsCycle;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;



  /* update the Low Pass Filter for DC-offset removal */
  prevDCoffset = DCoffset;
  DCoffset = prevDCoffset + (0.01 * cumVdeltasThisCycle); 

  //Calculate the real power of all instantaneous measurements taken during the 
  //previous mains cycle, and determine the gain (or loss) in energy.
  //realPower = POWERCAL * sumP / (float)samplesDuringThisMainsCycle;
  float realEnergy = realPower / cyclesPerSecond;

  if (beyondStartUpPhase == true)
  {  
    // Providing that the DC-blocking filters have had sufficient time to settle,    
    // add this power contribution to the energy bucket
    energyInBucket += realEnergy;   

    // Reduce the level in the energy bucket by the specified safety margin.
    // This allows the system to be positively biassed towards export rather than import
    energyInBucket -= safetyMargin_watts / cyclesPerSecond; 

    // Apply max and min limits to bucket's level
    if (energyInBucket > capacityOfEnergyBucket)
      energyInBucket = capacityOfEnergyBucket;  
    if (energyInBucket < 0)
      energyInBucket = 0;  
  }
  else
  {  
    // wait until the DC-blocking filters have had time to settle, 5 seconds...
    if(cycleCount > 500)
      beyondStartUpPhase = true;
  }

  // first check the level in the energy bucket to determine whether the 
  // triac should be fired or not at the next opportunity
  if (energyInBucket > (capacityOfEnergyBucket / 2))        
  {
    //nextStateOfTriac = ON;  // the external trigger device is active low
    digitalWriteFast(outputPinForTrigger, ON);
  } 
  else
  {
    //nextStateOfTriac = OFF; 
    digitalWriteFast(outputPinForTrigger, OFF);
  } 




  // clear the per-cycle accumulators for use in this new mains cycle.
  sumV = 0;
  sumI = 0;
  sumP = 0;
  samplesDuringThisMainsCycle = 0;
  cumVdeltasThisCycle = 0;

  // attaches callback() as a timer overflow interrupt, called every 200microseconds
  //delay: 200=100 samples per wave, 150=133 samples, 
  Timer1.attachInterrupt(db,250);  

  digitalWriteFast(10, LOW);
  //digitalWriteFast(9, HIGH);
}





double filteredI;

void db() {
  unsigned long start=micros();

  static int16_t lastSampleV=sampleV;            // save previous voltage values for digital high-pass filter
  static double lastFilteredV = filteredV;  
  
  sampleV = analogRead(voltageSensorPin);                 //Read in raw voltage signal
  sampleI = analogRead(currentSensorPin);                 //Read in raw current signal

  double sampleVminusDC = sampleV - DCoffset;

  //-----------------------------------------------------------------------------
  // B1) Apply digital high pass filter to remove 2.5V DC offset from 
  //     voltage sample in both normal and debug modes.
  //     Further processing of the current sample is deferred.
  //-----------------------------------------------------------------------------
  filteredV = 0.996093F*(lastFilteredV+(sampleV-lastSampleV));

  double  sampleIminusDC = sampleI - DCoffset;

  //-----------------------------------------------------------------------------
  // C) Root-mean-square method voltage
  //-----------------------------------------------------------------------------  
  sqV= filteredV * filteredV;                 //1) square voltage values
  sumV += sqV;                                //2) sum
  
  //-----------------------------------------------------------------------------
  // D) Root-mean-square method current
  //-----------------------------------------------------------------------------   
  sqI = sampleIminusDC * sampleIminusDC;                //1) square current values
  sumI += sqI;                                //2) sum 
  
  //-----------------------------------------------------------------------------
  // E) Phase calibration
  //-----------------------------------------------------------------------------
  double phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV); 
   
  // power contribution for this pair of V&I samples 
  // cumulative power values for this mains cycle 
  double instP = phaseShiftedV * sampleIminusDC;          //Instantaneous Power
  sumP +=instP;     
  
  cumVdeltasThisCycle += (sampleV - DCoffset); // for use with LP filter

  samplesDuringThisMainsCycle++;  // for power calculation at the start of each mains cycle
  interrupt_timing=micros()-start;
}

