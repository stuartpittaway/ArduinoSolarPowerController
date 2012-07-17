ArduinoSolarPowerController
===========================

Hosted from original source from http://openenergymonitor.org/emon/node/841

Submitted by calypso_rae on Sun, 15/07/2012 - 17:03




Based on the standard V&I sketch which uses calcVI() in EmonLib, I’ve had a system in place for the last couple of months which diverts surplus PV power to our immersion heater.  This system has proved to be both effective and reliable, and I am most grateful for everyone who has helped me along the way.  The main downside to this arrangement is that it requires an expensive item of third-party kit to distribute the power.  (I know that others have found cheaper ways, but my Carlo Gavazzi unit cost me £70).

Other contributors to this forum have shown that a standard triac can be easily controlled by an Arduino.  By using a zero-crossing detector such as the Motorola MOC3041, it should be possible for the Arduino to allocate mains cycles directly to the load rather than delegating this task to a separate device.

A Mk2 system of this type has been working in our garage for the last few days.  The sketch and a schematic diagram are attached, its main features being:

- a comprehensive 'debug' mode which allows real world conditions to be simulated ;

- a continuous mode of operation, for both measurement and distribution of power;

- an "energy bucket" concept which gives precise control of surplus power;

- interleaved windows for measurement and generation, each only 20mS;

- a rapid response time to changing conditions (<50mS);

- suitable timing for 'arming' a zero-crossing trigger device;

- a single LPF which determines the dc-offset of raw V&I samples;

- a programmable safety margin for biassing import v. export;

- minimal calibration is required.

As supplied, the sketch is in ‘debug’ mode.  This allows the code to be put through its paces without requiring any additional hardware – just the Arduino.  When running in this mode, voltage and current samples are synthesized, as is the operation of the triac.   To convert it to ‘normal’ mode, just comment out the #ifdef DEBUG statement. 

The algorithms for measuring and distributing power are linked by an ‘energy bucket’ variable.  Surplus energy, as measured at the supply point, is recorded in this variable, and power is only allocated to the immersion heater when the available energy has reached a pre-determined level.  The energy bucket is updated after each complete cycle of the mains, and a decision is then taken as to whether the triac should be ‘on’ or ‘off’ during the next cycle. 

To minimise the response time, the single-cycle (20mS) windows for measuring and distributing power are interleaved.  If sufficient energy is not available, the triac will be ‘off’ from the next zero-crossing point, just 10mS after that information has been gained.

In the standard EmonLib code, there is a separate high-pass filter (HPF) on each of the voltage and current streams.  I’ve taken a different approach (thanks Robert for the suggestion), and have instead implemented a single LP filter which allows the DC bias to be accurately determined.  By updating the LPF only once per mains cycle (my idea!), its performance is nigh-on perfect, with no attenuation or phase shift.

Too good to believe?  Well, the LPF is not acting alone.  There is also a standard HP filter which acts just on the voltage stream.  The purpose of this secondary filter is to group the voltage samples into cycles so that the LPF can be accurately updated.   Unlike the LPF, the HPF can always be relied upon to start up correctly.  Together, they form a great combination.

A single (buffered) reference is used for both the voltage and current sensor as shown on the schematic diagram.  The LM358 runs from the Arduino’s 5V rail and provides a rock-steady reference point.  If separate reference circuits were to be used, the single LPF approach would then not be appropriate. 

The energy bucket has a nominal capacity of 3600J, or 0.001kWh, with power only being allocated when it is at least half-full.   To ensure that a small amount of export to the grid is maintained, a programmable safety margin has been included; this acts as a leak in the bucket thereby reducing the rate that ‘on’ cycles can be allocated to the immersion.  For anyone with a nervous disposition, just increase the value of safetyMargin_watts.  

So how accurate does this system need to be?  Not very, is the short answer.  Because there is only one place where current is measured (flowing into and out from the grid), any inaccuracy will be cancelled out.  Once the distribution algorithm has reached the operating point, ‘on’ cycles will be allocated at the appropriate rate for the prevailing conditions regardless of any absolute error in the measurement system (it just needs to be linear). 

In a place such as this, if I were to suggest that no calibration is required at all, I’d probably get shot down in flames.  So, here’s how it works: 

In ‘debug’ mode,  the synthesized values are directly equivalent to Volts and Amps.  By mutiplying pairs of V&I samples together, this gives the instantaneous power in Watts.  By adding a mains cycles’ worth of instP values together, and dividing by the number of samples,  this gives the average power during that mains cycle, also in Watts.  Dividing by cyclesPerSecond = 50 gives the energy gained or lost during that individual mains cycle, in Joules.  This energy contribution is then added to the existing contents of the bucket without need for further calibration.  In debug mode, POWERCAL = 1.

In ‘normal’ mode, the sensitivity of the measurement system is affected by a variety factors.  My simplistic approach, however, says that it is only “energy” that needs to be calibrated.  By experimenting with a few small loads, I soon found that a value of about 0.085 allows my system to track real-world energy flow.  With a 40W bulb connected, my bucket ‘fills’ at around 40J per second.   Moreover, when the light goes off, the bucket’s value remains constant to within a few tenths of a Joule per second – there's minimal unintended leakage.  So, before adding my measured power contributions into the energy bucket, they are multiplied by POWERCAL = 0.085.  The units of this parameter are Joules per ADC-unit squared, but what about its value? 

Using any of tools that have been posted recently for recording the range of raw sample values, the extent to which voltage and current are being over- or under-read in terms of ADC units can be determined.  From the voltage and current samples taken while powering my 3kW kettle, I appears that I am ‘under-reading’ voltage by 471/678.8 and ‘over-reading’ current by 535/35.36.  Taking the inverse of each of these ratios gives an overall value of 0.095, which is close enough to 0.085 for my purposes. 

One final bit of calibration is needed, this being to determine the correct moment for arming the external trigger device.  Because my system under-reads voltage by a factor of 471/678.8, my voltage samples (after removal of the DC-offset) need to be multiplied by the inverse of this ratio to obtain the correct value in Volts.  VOLTAGECAL = 1.441 is therefore applied before checking whether the voltage is at a suitable value for arming the trigger (for the MOC3041, this is a minimum of 20V beyond the +ve going z/c point.)

The hardware is entirely straightforward, photo attached.  The input circuitry is still on breadboard but will no doubt be transferred to strip-board in due course.  With the 40W triac bolted to an offcut of aluminium tubing, it gets pleasantly warm when the PV gets going, but is rarely too hot to touch.  A better heatsink, with vertical fins, would be a nice addition.

