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


// -R2 / R1
// R2=470,000
// R1=  4,700 = -100.0 gain
// R1= 20,000 = - 23.5 gain 
// R1= 24,600 = - 19.1 gain
#define OPAMPGAIN = -470000.0/24600.0


#define VCAL 248.8F
#define ICAL 7.366F
#define PHASECAL 0.85F

//Added in digitalWriteFast from http://code.google.com/p/digitalwritefast
#include "digitalWriteFast.h"

#include <LiquidCrystal.h>

#include "TimerOne.h"

// define the input and output pins
#define outputPinForLed       13
#define outputPinForTrigger   9

//eMonTX uses A2 and A3 for voltage and CT1 sockets
#define voltageSensorPin      A2
#define currentSensorPin      A3

// use float to ensure accurate maths, mains frequency (Hz)
#define cyclesPerSecond   50.0F 

//How many times the interrupt will fire during a full mains cycle (microseconds)
//In this case 55 samples per full AC wave, if you notice the LCD display going REAL slow reduce this value...
#define NUMBER_OF_SAMPLES_PER_FULLWAVE 50


//Average the power readings over this number of AC samples (25=half second)
#define NUMBER_OF_FULLWAVES_TO_SAMPLE 25

//Time in microseconds between each interrupt
#define INTERRUPTDELAY (1000000/cyclesPerSecond)/NUMBER_OF_SAMPLES_PER_FULLWAVE

#define POSITIVE 1
#define NEGATIVE 0
#define ON 0        // the external trigger device is active low
#define OFF 1

// 0.001 kWh = 3600 Joules
#define capacityOfEnergyBucket 3600

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

static double safetyMargin_watts = 0;  // <<<-------  Safety Margin in Watts (increase for more export)

static volatile int32_t SUPPLYVOLTAGE;
static volatile int16_t lastSampleV,lastSampleI;

static volatile double apparentPower,powerFactor,Vrms,Irms;
static volatile double sumP,filteredI,lastFilteredI,filteredV;
static volatile double lastFilteredV,sumV,sumI,phaseShiftedV;

static volatile uint32_t cycleCount = 0;
static volatile uint8_t nextStateOfTriac = OFF;
static volatile int16_t voltageAtZeroCross=0;

static volatile int16_t sampleV,sampleI;   // voltage & current samples are integers in the ADC's input range 0 - 1023 
static volatile double realPower=0;

//Just for statistics
static volatile uint16_t samplesDuringThisMainsCycle = 0;
static volatile uint16_t xxxxsamplesDuringThisMainsCycle=0;

//Total number of samples taken over several AC wave forms
static volatile uint16_t numberOfSamples=0;

static volatile uint8_t polarityNow = NEGATIVE; // probably not important, but better than being indeterminate
static volatile bool beyondStartUpPhase=false;
static volatile unsigned long millisecondsPerZeroCross=0;
static volatile unsigned long previousmillisecondsPerZeroCross=0;

// the 'energy bucket' mimics the operation of a digital supply meter at the grid connection point.
static volatile double energyInBucket = 0;                                                 
//int16_t lastSampleV;     // stored value from the previous loop (HP filter is for voltage samples only)         

static volatile uint8_t bouncecounter=0;
static volatile boolean triggerNeedsToBeArmed = false;
//  voltage values after filtering to remove the DC offset, and the stored values from the previous loop             
//static  volatile float filteredV;  
//static  volatile float prevDCoffset;          // <<--- for LPF to quantify the DC offset
//static  volatile float DCoffset;              // <<--- for LPF 
//static volatile float cumVdeltasThisCycle;   // <<--- for LPF 

#define   CONTRAST_PIN   9
#define   CONTRAST       20

//LiquidCrystal(rs, enable, d4, d5, d6, d7) 
LiquidCrystal lcd(10, 11, 8,7,6,5);

void setup()
{  
  Serial.begin(115200);  //Faster baud rate to reduce timing of serial.print commands

  //Use PWM to control the LCD contrast
  pinModeFast(CONTRAST_PIN, OUTPUT);
  analogWrite(CONTRAST_PIN, CONTRAST);

  pinModeFast(outputPinForTrigger, OUTPUT);  
  pinModeFast(outputPinForLed, OUTPUT);  

  //pinModeFast(voltageSensorPin, INPUT);
  //pinModeFast(currentSensorPin, INPUT);

  analogReference(DEFAULT);  //5v
  SUPPLYVOLTAGE = readVcc();

  lcd.begin(16,2);               // initialize the lcd 

  lcd.home ();
  //         AAAABBBBCCCCDDDD
  lcd.print("OEM eMonTX Solar"); 
  delay(1000); 
  /*  
   lcd.setCursor ( 0, 1 );
   lcd.print("     Code by    "); 
   delay(1000); 
   lcd.setCursor ( 0, 1 );
   lcd.print("Stuart Pittaway "); 
   delay(1000); 
   lcd.setCursor ( 0, 1 );
   lcd.print("Solar PV divert "); 
   delay(1000); 
   lcd.setCursor ( 0, 1 );
   lcd.print("by calypso_rae  "); 
   delay(1000);
   */
  lcd.clear();

  Timer1.initialize();

  //STUART ADDED.... EXPERIMENAL FASTER ANALOGUE READ ON ARDUINO 
  //set prescale as per forum on http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  //table of prescale values... http://i187.photobucket.com/albums/x269/jmknapp/adc_prescale.png
  //Prescale 32
  sbi(ADCSRA,ADPS2);  //ON
  cbi(ADCSRA,ADPS1);  //OFF
  cbi(ADCSRA,ADPS0);  //OFF

  //Start the interrupt and wait for the first zero crossing...
  attachInterrupt(1, positivezerocrossing, RISING);
}


int page=0;
int counter=8;

void loop() // each loop is for one pair of V & I measurements
{  

  //cli(); // disable global interrupts  

  lcd.home ();
  lcd.clear ();

  if (page==0) {
    lcd.print(Vrms,1); 
    lcd.print("V  AP:");
    lcd.print(apparentPower,1);

    lcd.setCursor ( 0, 1 );

    lcd.print(Irms,1); 
    lcd.print("A  PF:");
    lcd.print(powerFactor,2); 
  }

  if (page==1) {
    lcd.print("RealPwr:");
    lcd.print(realPower,1);
    lcd.print("W");

    lcd.setCursor ( 0, 1 );

    lcd.print("enInBkt:");
    lcd.print(energyInBucket);
  }

  if (page==2) {
    lcd.print("filterV:");
    lcd.print(filteredV,1);
    lcd.setCursor ( 0, 1 );

    lcd.print("cyc#:");
    lcd.print(cycleCount);

  }
  
  counter--;
  if (counter==0) {
    page++; 
    counter=8;
  }
  if (page>1) page=0;

  Serial.print(" cyc# ");
  Serial.print(cycleCount);
  //Serial.print(",time ");
  //Serial.print(millisecondsPerZeroCross);
  //Serial.print(",Hz ");
  //Serial.print(1000000.0/millisecondsPerZeroCross);
  Serial.print(",samp ");
  Serial.print(xxxxsamplesDuringThisMainsCycle);


  //Serial.print(",intT ");
  //Serial.print(millisecondsPerZeroCross/xxxxsamplesDuringThisMainsCycle);       

  Serial.print(", Vzero ");
  Serial.print(voltageAtZeroCross);
/*
  Serial.print(",fltdV ");
  Serial.print(filteredV);
  Serial.print(",fltdI ");
  Serial.print(filteredI);
  Serial.print(",sampV ");
  Serial.print(sampleV);
  Serial.print(",sampI ");
  Serial.print(sampleI);
*/
  
  //Serial.print(", sampleV-dc ");
  //Serial.print((int)(sampleV - DCoffset));
  //Serial.print(", cumVdeltas ");
  //Serial.print(cumVdeltasThisCycle);
  //Serial.print(", prevDCoffset ");
  //Serial.print(prevDCoffset);
  //Serial.print(",refFltdV ");
  //Serial.print(DCoffset);
  Serial.print(",SupVol ");
  Serial.print(SUPPLYVOLTAGE);

  Serial.print(",Vrms=");
  Serial.print(Vrms);
  Serial.print("V,Irms=");
  Serial.print(Irms);
  //Serial.print(",realPwr ");
  //Serial.print(realPower);
  Serial.print("A,app.Pwr=");
  Serial.print(apparentPower);
  Serial.print("VA,pwrF=");
  Serial.print(powerFactor);

  Serial.print(",P=");
  Serial.print(realPower);

  Serial.print("W,enInBkt=");
  Serial.println(energyInBucket);

  //Serial.print(SUPPLYVOLTAGE);

  /*  
   Serial.print(" V=");
   Serial.print(sampleV);
   Serial.print(" I=");
   Serial.print(sampleI);
   
   Serial.print(" F=");
   Serial.print(millisecondsPerZeroCross);  
   Serial.print(" Hz=");
   Serial.print(1000000.0/millisecondsPerZeroCross); 
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

  digitalWriteFast(outputPinForLed, HIGH);  // active high
  delay(15);  //small delay
  digitalWriteFast(outputPinForLed, LOW);  
  delay(1000-15);  //small delay
} // end of loop()


static long readVcc() {
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

/*
static unsigned long myMicros() {
 extern volatile unsigned long timer0_overflow_count;
 uint8_t oldSREG = SREG;
 cli();
 uint32_t t = TCNT0;
 if ((TIFR0 & _BV(TOV0)) && (t == 0))
 t = 256;
 uint32_t m = timer0_overflow_count;
 SREG = oldSREG;
 return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
 }
 */

static void positivezerocrossing()
{ 
  Timer1.detachInterrupt();

    xxxxsamplesDuringThisMainsCycle=samplesDuringThisMainsCycle;
    samplesDuringThisMainsCycle = 0;

  cycleCount++; // for stats only

  if (cycleCount==NUMBER_OF_FULLWAVES_TO_SAMPLE) {
    cycleCount=0;
    //-------------------------------------------------------------------------------------------------------------------------
    // Post loop calculations
    //------------------------------------------------------------------------------------------------------------------------- 
    //Calculation of the root of the mean of the voltage and current squared (rms) and Calibration coeficients applied. 

    //-------------------------------------------------------------------------------------------------------------------------
    // 3) Post loop calculations
    //------------------------------------------------------------------------------------------------------------------------- 
    //Calculation of the root of the mean of the voltage and current squared (rms)
    //Calibration coeficients applied. 

    //DONT BOTHER DOING THIS EACH LOOP
    //SUPPLYVOLTAGE = readVcc();

    double V_RATIO = VCAL *((SUPPLYVOLTAGE/1000.0) / 1023.0);
    Vrms = V_RATIO * sqrt(sumV / (double)numberOfSamples); 
    double I_RATIO = ICAL *((SUPPLYVOLTAGE/1000.0) / 1023.0);
    Irms = I_RATIO * sqrt(sumI / (double)numberOfSamples); 

    //Calculation power values
    realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
    apparentPower = Vrms * Irms;
    powerFactor=realPower / apparentPower;

    /*
  // update the Low Pass Filter for DC-offset removal
     prevDCoffset = DCoffset;
     DCoffset = prevDCoffset + (0.01 * cumVdeltasThisCycle); 
     
     //Calculate the real power of all instantaneous measurements taken during the 
     //previous mains cycle, and determine the gain (or loss) in energy.
     //realPower = POWERCAL * sumP / (float)samplesDuringThisMainsCycle;
     double realEnergy = realPower / cyclesPerSecond;
     
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
     // wait until the DC-blocking filters have had time to settle for a few seconds...
     if(cycleCount > 200)
     beyondStartUpPhase = true;
     }
     
     // first check the level in the energy bucket to determine whether the 
     // triac should be fired or not at the next opportunity
     if (energyInBucket > (capacityOfEnergyBucket / 2))        
     {
     //nextStateOfTriac = ON;  // the external trigger device is active low
     digitalWriteFast(outputPinForTrigger, HIGH);
     } 
     else
     {
     //nextStateOfTriac = OFF; 
     digitalWriteFast(outputPinForTrigger, LOW);
     } 
     */
    numberOfSamples=0;

    // clear the per-cycle accumulators for use in this new mains cycle.
    sumV = 0;
    sumI = 0;
    sumP = 0;
    //cumVdeltasThisCycle = 0;
  }


  //Get the first reading near the zero cross...
  db();

  voltageAtZeroCross=sampleV;

  // attaches callback() as a timer overflow interrupt, called every X microseconds
  Timer1.attachInterrupt(db,INTERRUPTDELAY );  
}


void db() {
  lastSampleV=sampleV;
  lastSampleI=sampleI;

  lastFilteredV = filteredV;  
  lastFilteredI = filteredI;  

  sampleI = analogRead(currentSensorPin);
  sampleV = analogRead(voltageSensorPin);

  //static double sampleVminusDC = sampleV - DCoffset;

  //-----------------------------------------------------------------------------
  // B) Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
  //-----------------------------------------------------------------------------
  filteredV = 0.996*(lastFilteredV+sampleV-lastSampleV);
  filteredI = 0.996*(lastFilteredI+sampleI-lastSampleI);

  //static double  sampleIminusDC = sampleI - DCoffset;

  //-----------------------------------------------------------------------------
  // C) Root-mean-square method voltage
  //-----------------------------------------------------------------------------  
  sumV += filteredV * filteredV;                 //1) square voltage values

  //-----------------------------------------------------------------------------
  // D) Root-mean-square method current
  //-----------------------------------------------------------------------------   
  sumI += filteredI * filteredI;                //1) square current values

  //-----------------------------------------------------------------------------
  // E) Phase calibration
  //-----------------------------------------------------------------------------
  phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV); 

  //-----------------------------------------------------------------------------
  // F) Instantaneous power calc
  //-----------------------------------------------------------------------------   
  sumP +=phaseShiftedV * filteredI;          //Instantaneous Power

  samplesDuringThisMainsCycle++;  // for power calculation of a single AC wave
  numberOfSamples++;  // for power calculation over a number of AC wave samples
}


