//--------------------------------------------------------------------------------------
// Internal temperature measurement of Attiny84 based FunkySensor derivative
// based on work from harizanov.com and jeelabs.org
// Arpel @ 2012-2013
// GNU GPL V3
//--------------------------------------------------------------------------------------
#include <JeeLib.h> // https://github.com/jcw/jeelib
#include "pins_arduino.h"
#include <avr/sleep.h>

//########################################################################################################################
// Default Configuration
//########################################################################################################################
#define destNodeID 1
#define myNodeID 30      // RF12 node ID in the range 1-30
// 17 for LDC
#define AS_PHOTOCELL 1
//#define AS_TWO_TEMP_PROBES 1
#define STATIC_ONEWIRE_INDEXES 1 // Save time and 118 Bytes of code : good for ATTiny !

//########################################################################################################################
// General Configuration
//########################################################################################################################
#define network 100      // RF12 Network group
#define freq RF12_868MHZ // Frequency of RFM12B module

#define NEED_ACK 0
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 10       // Number of milliseconds to wait for an ack

#include <OneWire.h>   // http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip
#include <DallasTemperature.h>  // http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_371Beta.zip
//#define TEMPERATURE_PRECISION 9
#define ASYNC_DELAY 750 // 9bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms
 
#define ONE_WIRE_BUS PIN_A7    // pad 5 of the Funky
#define tempPower PIN_A3       // Power pin is connected pad 4 on the Funky
#define LEDpin PIN_A0

#ifdef AS_PHOTOCELL
#define PHOTOCELLpin PIN_A1 // the LDR will be connected to analog 1
#else
#define ADCFORVCCONLY 1
#endif

//########################################################################################################################
// Local typefdefs
//########################################################################################################################
typedef struct {
   byte nodeid;  // Node ID
   byte id;      // Packet ID
   int temp1;	   // Temperature reading  Probe 1
   unsigned int supplyV;	// Supply voltage
#ifdef AS_TWO_TEMP_PROBES
   int temp2;	   // Temperature reading  Probe 2
#endif
   byte numsensors;
#ifdef AS_PHOTOCELL
   byte photocell;
#endif
} Payload_t;

//########################################################################################################################
// Local functions
//########################################################################################################################
#ifdef ADCFORVCCONLY
static unsigned int vccRead(unsigned int count);
#else
static unsigned int adcMeanRead(byte adcmux, unsigned int count, bool tovdc);
#endif
static int readDS18120(void);
static void blinkLED(int ntimes, int time);
void RF_air_send(Payload_t *pl);

//########################################################################################################################
// Local variables
//########################################################################################################################
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
#ifdef STATIC_ONEWIRE_INDEXES
// addresses of sensors, MAX 4!!  
byte allAddress [4][8];  // 8 bytes per address
#endif

volatile bool adcDone;

// for low-noise/-power ADC readouts, we'll use ADC completion interrupts
ISR(ADC_vect) { adcDone = true; }

// this must be defined since we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

Payload_t staticpayload;

//########################################################################################################################
// LOCAL Functions
//########################################################################################################################
#ifdef ADCFORVCCONLY
static unsigned int vccRead(unsigned int count = 10) {
#else
static unsigned int vccRead(unsigned int count = 10) {
  return adcMeanRead(B00100001, count, true);
}
static unsigned int adcMeanRead(byte adcmux, unsigned int count = 10, bool tovdc = false) {
#endif
  unsigned long adccumulative = 0;
  unsigned int totalcount = count;
  
  ADCSRB = 0;
  ADCSRA = bit(ADEN) | bit(ADIE) | bit(ADATE); // Enable + Interrupt Enable Auto Trigger Enable - Start  
#ifdef ADCFORVCCONLY
  ADMUX = B00100001;
#else
  ADMUX = adcmux;
#endif

  set_sleep_mode(SLEEP_MODE_ADC);
  
  bitSet(ADCSRA, ADSC); // Start first conversion
  // Throw first value
  adcDone = false;
  while (!adcDone)
    sleep_mode();

  while (count-- > 0) {
    adcDone = false;
    while (!adcDone)
      sleep_mode();
    adccumulative += ADC;
  }
  ADCSRA = 0;  

#ifdef ADCFORVCCONLY
  adccumulative = 1125300L / (adccumulative / totalcount);
  return (unsigned int)adccumulative;
#else
  if (tovdc == true){
    adccumulative = 1125300L / (adccumulative / totalcount);
    return (unsigned int)adccumulative;
  } else {
    return (unsigned int)(adccumulative / totalcount);
  }
#endif
}

static int readDS18120(void)
{
  int result = 0;
  pinMode(tempPower, OUTPUT);     // set power pin for DS18B20 to output  
  digitalWrite(tempPower, HIGH);  // turn DS18B20 sensor on
  loseSomeTime(20);               // Allow 10ms for the sensor to be ready
  sensors.requestTemperatures();  // Send the command to get temperatures
  loseSomeTime(ASYNC_DELAY);           // Must wait for conversion, since we use ASYNC mode
  
#ifdef STATIC_ONEWIRE_INDEXES 
  staticpayload.temp1 = sensors.getTempC(allAddress[0])*100; // Read Probe 1
#ifdef AS_TWO_TEMP_PROBES
  staticpayload.temp2 = sensors.getTempC(allAddress[1])*100; // Read Probe 2
#endif
#else  
  staticpayload.temp1 = sensors.getTempCByIndex(0)*100; // Read Probe 1
#ifdef AS_TWO_TEMP_PROBES
  staticpayload.temp2 = sensors.getTempCByIndex(1)*100; // Read Probe 2
#endif
#endif

  digitalWrite(tempPower, LOW); // turn DS18B20 sensor off
  pinMode(tempPower, INPUT);
}

static void blinkLED(int ntimes, int time){
  for (int i = 0; i <= ntimes; ++i) {
      digitalWrite(LEDpin,LOW);
      loseSomeTime(time/2);
      //delay(time/2);
      digitalWrite(LEDpin,HIGH);
      //delay(time/2);
      loseSomeTime(time/2);
  }
  //delay(1000);
}

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//--------------------------------------------------------------------------------------------------
void RF_air_send(Payload_t *pl){
   bitClear(PRR, PRUSI); // enable USI h/w
   digitalWrite(LEDpin, LOW);
   
   for (byte i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
       rf12_sleep(RF12_WAKEUP);
       int i = 1; 
       while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
              
       byte header = RF12_HDR_ACK | RF12_HDR_DST | destNodeID;
  
       rf12_sendStart(header, pl, sizeof *pl);
       rf12_sendWait(2); // Wait for RF to finish sending while in standby mode
#if NEED_ACK
       byte acked = waitForAck();  // Wait for ACK
#endif
       rf12_sleep(RF12_SLEEP);
#if NEED_ACK
       if (acked) { break; }      // Return if ACK received
       loseSomeTime(RETRY_PERIOD * 500);     // If no ack received wait and try again
#else
        break;
#endif
   } 
   
   digitalWrite(LEDpin, HIGH);
   bitSet(PRR, PRUSI); // disable USI h/w
}

//########################################################################################################################
// GLOBAL Functions
//########################################################################################################################
void setup() {
   pinMode(LEDpin, OUTPUT);
   blinkLED(1, 50);
  
   cli();
   CLKPR = bit(CLKPCE);
#if defined(__AVR_ATtiny84__)
   CLKPR = 0; // div 1, i.e. speed up to 8 MHz
#else
   CLKPR = 1; // div 2, i.e. slow down to 8 MHz
#endif
   sei();
  
   rf12_initialize(myNodeID,freq,network); // Initialize RFM12 with settings defined above 
   // Adjust low battery voltage to 2.2V
   rf12_control(0xC040);
   rf12_sleep(RF12_SLEEP);  // Put the RFM12 to sleep
 
   PRR = bit(PRTIM1); // only keep timer 0 going

   rf12_recvDone();
   rf12_recvDone();
   rf12_recvDone();
  
   /* Setup Dallas Temp probe */
   pinMode(tempPower, OUTPUT); // set power pin for DS18B20 to output
   digitalWrite(tempPower, HIGH); // turn sensor power on
   loseSomeTime(50); // Allow 50ms for the sensor to be ready
   // Start up the library
   sensors.begin();
   sensors.setWaitForConversion(false); 
   staticpayload.numsensors = sensors.getDeviceCount();

#ifdef STATIC_ONEWIRE_INDEXES
    byte j=0;   // search for one wire devices and
    while ((j < staticpayload.numsensors) && (oneWire.search(allAddress[j]))) {        
      j++;
    }
#endif
   
#ifdef AS_PHOTOCELL
   digitalWrite(PHOTOCELLpin, LOW); // Turns pull up off
   pinMode(PHOTOCELLpin, INPUT);
#endif

   blinkLED(1, 50);
}

void loop() {
   bitClear(PRR, PRADC); // power up the ADC
   loseSomeTime(16); // Allow 10ms for the sensor to be ready
  
   staticpayload.supplyV = vccRead(4);
   // staticpayload temp1 & temp2 set according configuration
   readDS18120();

#ifdef AS_PHOTOCELL
   // Need to enable the pull-up to get a voltage drop over the LDR
   pinMode(PHOTOCELLpin, INPUT_PULLUP);
   staticpayload.photocell = (adcMeanRead(B10000001, 4, false) & 0x1FF) / 2;
   digitalWrite(PHOTOCELLpin, LOW); // Turns pull up off
   pinMode(PHOTOCELLpin, INPUT);
#endif
   bitSet(PRR, PRADC); // power down the ADC

   staticpayload.nodeid = myNodeID;
   staticpayload.id += 1;
    
   RF_air_send(&staticpayload);
  
   for (byte i = 0; i < 5; ++i)
     //loseSomeTime(60*1000);
     loseSomeTime(1*1000);
}

static byte waitForAck() {
  MilliTimer ackTimer;
  while (!ackTimer.poll(ACK_TIME)) {
   if (rf12_recvDone() && rf12_crc == 0 && rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
     return 1;
   }
   return 0;
}

void loseSomeTime(unsigned int ms){
    byte oldADCSRA=ADCSRA;      // Save ADC state
    byte oldADCSRB=ADCSRB;
    byte oldADMUX=ADMUX;
    
    Sleepy::loseSomeTime(ms);   // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
    
    ADCSRA=oldADCSRA;           // Restore ADC state
    ADCSRB=oldADCSRB;
    ADMUX=oldADMUX;    
}


