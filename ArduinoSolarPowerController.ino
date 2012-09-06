
/*
ArduinoSolarPowerController.ino
 
Based on eMonTX and Mk2_PV_Router_mini by Robin Emley (calypso_rae on Open Energy Monitor Forum July 2012) 
Additional inspiration taken from http://read.pudn.com/downloads108/sourcecode/embed/444108/IAR/AVR465.c__.htm

 
PINS changed to run on Arduino UNI compat board (I'm using breadboard!)

I'm using a 16x2 LCD display with RGB lighting, model Winstar Display Co. WH1602B-CFH-JT

Connecting PINS: 4=RS, A5=enable, d4=8, d5=7,d6=6, d7=5
//LiquidCrystal(rs, enable, d4, d5, d6, d7) 
//LiquidCrystal lcd(4, A5, 8,7,6,5);


PINS FOR JEE BOARD RFM12B PINOUT
       BOARD= 5v gnd sck sdo sdi sel irq 3v3
ARDUINO PINS=n/a n/a d13 d12 d11 d10 d2  n/a
*/

/*
 Joules per ADC-unit squared.  Used for converting the product of voltage and current samples into Joules.
 To determine this value, note the rate that the energy bucket's level increases when a known load is being measured at a convenient
 test location (e.g  using a mains extention with the outer cover removed so that 
 the current-clamp can fit around just one core.  Adjust POWERCAL so that 'measured value' = 'expected value' for various loads.  The value of
 POWERCAL is not critical as any absolute error will cancel out when  import and export flows are balanced.  
 */
#define POWERCAL 0.085F

// -R2 / R1
// R2=470,000
// R1=  4,700 = -100.0 gain
// R1= 20,000 = - 23.5 gain 
// R1= 24,600 = - 19.1 gain
#define OPAMPGAIN = -470000.0/24600.0

#define VCAL 248.8F
#define ICAL 7.366F
#define PHASECAL 0.85F

//#define enable_serial_debug

// Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
// 433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
#define freq RF12_433MHZ                                                
// emonTx RFM12B node ID
const int nodeID = 10;                                                  
// emonTx RFM12B wireless network group - needs to be same as emonBase and emonGLCD needs to be same as emonBase and emonGLCD
const int networkGroup = 210;                                           

// Download JeeLib: http://github.com/jcw/jeelib
//#include <JeeLib.h>
//#include <Ports.h>
//#include <PortsBMP085.h>
#include <PortsLCD.h>
//#include <PortsSHT11.h>
#include <RF12.h>
//#include <RF12sio.h>

//use code from jeelib instead... #include <LiquidCrystal.h>

//Added in digitalWriteFast from http://code.google.com/p/digitalwritefast
//digitalWriteFast might not need this
#include "digitalWriteFast.h"
#include "TimerOne.h"

// define the input and output pins
//#define outputPinForLed       13
#define outputPinForTrigger   A6

//eMonTX uses A2 and A3 for voltage and CT1 sockets
#define voltageSensorPin      A2
#define currentSensorPin      A3

// the external trigger device is active low
#define TRIAC_ON 0        
#define TRIAC_OFF 1

#define LED_ON HIGH
#define LED_OFF LOW

#define   CONTRAST_PIN   9
#define   CONTRAST       1

// use float to ensure accurate maths, mains frequency (Hz)
#define cyclesPerSecond   50.0F 
#define safetyMargin_watts 0  // <<<-------  Safety Margin in Watts (increase for more export)

//How many times the interrupt will fire during a full mains cycle (microseconds)
//In this case 55 samples per full AC wave, if you notice the LCD display going REAL slow reduce this value...
#define NUMBER_OF_SAMPLES_PER_FULLWAVE 50

//Average the power readings over this number of AC samples (25=half second)
#define NUMBER_OF_FULLWAVES_TO_SAMPLE 50

//Time in microseconds between each interrupt
#define INTERRUPTDELAY (1000000/cyclesPerSecond)/NUMBER_OF_SAMPLES_PER_FULLWAVE

// 0.001 kWh = 3600 Joules
#define capacityOfEnergyBucket 3600

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//LiquidCrystal(rs, enable, d4, d5, d6, d7) 
//LiquidCrystal lcd(4, A5, 8,7,6,5);
LiquidCrystal lcd(A4, A5, 8,7,6,5);

static volatile int32_t SUPPLYVOLTAGE;
static volatile int16_t lastSampleV,lastSampleI;

static volatile double apparentPower,powerFactor,Vrms,Irms;
static volatile double sumP,filteredI,lastFilteredI,filteredV,singleCycleSumP;
static volatile double lastFilteredV,sumV,sumI,phaseShiftedV;
static volatile double divert_realPower,divert_realEnergy;
static volatile int readingsTaken=0;

static volatile uint32_t waveformSampledCount = 0;
static volatile int16_t voltageAtZeroCross=0;

static volatile int16_t sampleV,sampleI;   // voltage & current samples are integers in the ADC's input range 0 - 1023 
static volatile double realPower=0;

//Just for statistics
static volatile uint16_t samplesDuringThisMainsCycle = 0;
static volatile uint16_t previousSamplesDuringThisMainsCycle=0;

//Total number of samples taken over several AC wave forms
static volatile uint16_t numberOfSamples=0;
static volatile bool beyondStartUpPhase=false;
static volatile unsigned long millisecondsPerZeroCross=0;
static volatile unsigned long previousmillisecondsPerZeroCross=0;

// the 'energy bucket' mimics the operation of a digital supply meter at the grid connection point.
static volatile double divert_energyInBucket = 0;                                                 

static uint8_t page=0;
static uint8_t counter=8;


void setup()
{  
#ifdef enable_serial_debug
  Serial.begin(115200);  //Faster baud rate to reduce timing of serial.print commands
#endif

  //Use PWM to control the LCD contrast
  pinModeFast(CONTRAST_PIN, OUTPUT);
  analogWrite(CONTRAST_PIN, CONTRAST);

  pinModeFast(outputPinForTrigger, OUTPUT);  
  //pinModeFast(outputPinForLed, OUTPUT);  

  analogReference(DEFAULT);  //5v

  SUPPLYVOLTAGE = readVcc();

  rf12_initialize(nodeID, freq, networkGroup);                          // initialize RF
  rf12_sleep(RF12_SLEEP);

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

  //FASTER ANALOGUE READ ON ARDUINO 
  //set prescale as per forum on http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  //table of prescale values... http://i187.photobucket.com/albums/x269/jmknapp/adc_prescale.png
  //Prescale 32
  sbi(ADCSRA,ADPS2);  //ON
  cbi(ADCSRA,ADPS1);  //OFF
  sbi(ADCSRA,ADPS0);  //ON

  //Start the interrupt and wait for the first zero crossing...
  attachInterrupt(1, positivezerocrossing, RISING);
}


typedef struct { int power1, power2, power3, battery; } PayloadTX;      // create structure - a neat way of packaging data for RF comms
PayloadTX emontx;  

void send_rf_data()
{


  rf12_sleep(RF12_WAKEUP);
  // if ready to send + exit loop if it gets stuck as it seems too
  int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
  
  rf12_sendStart(0, &emontx, sizeof emontx);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  
  rf12_sendWait(2);
  
  rf12_sleep(RF12_SLEEP);
}


int transmitdelay=4;

void loop()
{  
  //Do what we want here, no time critical code to worry about
  //the interrupts do all the hard work!
  
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
    lcd.print(divert_energyInBucket,1);
  }

  counter--;
  if (counter==0) {
    page++; 
    counter=8;
  }
  if (page>1) page=0;

#ifdef enable_serial_debug

  Serial.print(" cyc# ");
  Serial.print(waveformSampledCount);

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
  Serial.print(divert_energyInBucket);
  Serial.print(",energy=");
  Serial.println(divert_realEnergy);


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
   Serial.print(previousSamplesDuringThisMainsCycle);
   Serial.print(" E=");
   Serial.print(energyInBucket);
   
   //This is a timing of the interrupt routine in microseconds
   Serial.print("  int time=");
   Serial.print(interrupt_timing);
   
   Serial.println();
   */

#endif

transmitdelay--;

if (transmitdelay==0) {

emontx.power1=apparentPower;
emontx.power2=divert_realEnergy;
emontx.power3=divert_energyInBucket;
emontx.battery=0;

  send_rf_data();                                                       // *SEND RF DATA* - see emontx_lib
  transmitdelay=4;
}
  delay(1000);
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

static void positivezerocrossing()
{ 
  //This is the start of a new mains cycle just before going zero cross point.  Timing of this 
  //depends upon the opamp comparitor circut.
  //To keep the timing constant as possible try to avoid putting anything in large if...then clauses
  //so that the path through the code is a similar as possible...
  Timer1.detachInterrupt();

  //Start of the solar power divert code...
  waveformSampledCount++;

  //This code sorts out the solar power diversion to power the triac circuit
  divert_realPower = POWERCAL * singleCycleSumP / (float)samplesDuringThisMainsCycle;
  divert_realEnergy = realPower / cyclesPerSecond;
  singleCycleSumP=0;

  if (beyondStartUpPhase == true)
  {  
    // Reduce the level in the energy bucket by the specified safety margin.
    // This allows the system to be positively biassed towards export or import
    divert_energyInBucket += divert_realEnergy-(safetyMargin_watts / cyclesPerSecond);   
  }

  // Apply max and min limits to bucket's level
  if (divert_energyInBucket > capacityOfEnergyBucket)
    divert_energyInBucket = capacityOfEnergyBucket;  

  if (divert_energyInBucket < 0)
    divert_energyInBucket = 0;    


  if (divert_energyInBucket > (capacityOfEnergyBucket / 2))        
  {
    digitalWriteFast(outputPinForTrigger, TRIAC_ON);
    //digitalWriteFast(outputPinForLed, LED_ON);
  } 
  else
  {
    digitalWriteFast(outputPinForTrigger, TRIAC_OFF);
    //digitalWriteFast(outputPinForLed, LED_OFF);
  } 

  //Back to the emonTx measurements
  if (waveformSampledCount == NUMBER_OF_FULLWAVES_TO_SAMPLE) {

    //Start the solar power divert after 5 samples (about 5 seconds)
    readingsTaken++;
    if(readingsTaken == 10) beyondStartUpPhase = true;    

    waveformSampledCount=0;
    //-------------------------------------------------------------------------------------------------------------------------
    // Post loop calculations
    //------------------------------------------------------------------------------------------------------------------------- 
    //Calculation of the root of the mean of the voltage and current squared (rms) and Calibration coeficients applied. 

    //-------------------------------------------------------------------------------------------------------------------------
    // 3) Post loop calculations
    //------------------------------------------------------------------------------------------------------------------------- 
    //Calculation of the root of the mean of the voltage and current squared (rms)
    //Calibration coeficients applied. 
    double V_RATIO = VCAL *((SUPPLYVOLTAGE/1000.0) / 1023.0);
    Vrms = V_RATIO * sqrt(sumV / (double)numberOfSamples); 
    double I_RATIO = ICAL *((SUPPLYVOLTAGE/1000.0) / 1023.0);
    Irms = I_RATIO * sqrt(sumI / (double)numberOfSamples); 

    //Calculation power values
    realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
    apparentPower = Vrms * Irms;
    powerFactor=realPower / apparentPower;

    numberOfSamples=0;

    // clear the per-cycle accumulators for use in this new mains cycle.
    sumV = 0;
    sumI = 0;
    sumP = 0;
    //cumVdeltasThisCycle = 0;
  }

  previousSamplesDuringThisMainsCycle=samplesDuringThisMainsCycle;
  samplesDuringThisMainsCycle = 0;

  //Get the first reading near the zero cross...
  takesinglereading();

  voltageAtZeroCross=sampleV;

  // attaches callback() as a timer overflow interrupt, called every X microseconds
  Timer1.attachInterrupt(takesinglereading,INTERRUPTDELAY );  
}


static void takesinglereading() {
  
  //Is it worth putting in the digital low pass filters here instead of high pass?
  //as per calypso_rae code?
  
  lastSampleV=sampleV;
  lastSampleI=sampleI;

  lastFilteredV = filteredV;  
  lastFilteredI = filteredI;  

  //As we are using an OpAmp the current reading is inverted - swap back.
  sampleI = 0x03FF-analogRead(currentSensorPin);
  sampleV = analogRead(voltageSensorPin);

  //-----------------------------------------------------------------------------
  // B) Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
  //-----------------------------------------------------------------------------
  filteredV = 0.996*(lastFilteredV+sampleV-lastSampleV);
  filteredI = 0.996*(lastFilteredI+sampleI-lastSampleI);

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
  singleCycleSumP +=phaseShiftedV * filteredI;          //Instantaneous Power for solar divert

  samplesDuringThisMainsCycle++;  // for power calculation of a single AC wave
  numberOfSamples++;  // for power calculation over a number of AC wave samples
}

