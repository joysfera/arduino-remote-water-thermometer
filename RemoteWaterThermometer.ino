/*
 * Remote Water Thermometer
 *
 * Petr Stehlik
 *
 * Released under GPL http://www.gnu.org/licenses/gpl.html
 */

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "WaterTempTransmitter.h"

//#define DEBUG

#define ONE_WIRE_BUS_PIN    2    // Data wire is plugged into port 2 on the Arduino
#define TX433MHZ_PIN        3    // Transmitter Arduino pin
#define TX_POWER_PIN        5    // power for transmitter

WaterTempTransmitter tx=WaterTempTransmitter(TX433MHZ_PIN, 0x00 /* sensor ID */, 2 /* transmit channel */);

#define TEMPERATURE_PRECISION 11

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS_PIN);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

#ifdef DEBUG
#define debug_init() Serial.begin(9600)
#define dprintexp(expression) { Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression ); }
#define dprint(text) Serial.print( text )
#define dprintln(text) Serial.println( text )
#else
#define debug_init()
#define dprintexp(expression)
#define dprint(text)
#define dprintln(text)
#endif

boolean beep_on_first_tx = true;

void setup()
{
    // start serial port
    debug_init();
    dprintln("DS18B20 433 MHz Thermometer");
    
    pinMode(TX_POWER_PIN, OUTPUT);
    
    // Start up the library
    sensors.begin();

    // Grab a count of devices on the wire
    int numberOfDevices = sensors.getDeviceCount();
    if (numberOfDevices == 0) {
        dprintln("Sensor(s) not found!");
        while(1);
    }     

    // locate devices on the bus
    dprintln("Locating devices...");

    dprintexp(numberOfDevices);

    // report parasite power requirements
    dprintexp(sensors.isParasitePowerMode());

    if(sensors.getAddress(tempDeviceAddress, 0)) {
        sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
        dprintexp(sensors.getResolution(tempDeviceAddress));
    }
    else {
        dprintln("Found ghost device but could not detect address. Check power and cabling!");
        while(true) ;
    }
}

void loop()
{
    dprint("Requesting temperatures... ");
    sensors.requestTemperatures(); // Send the command to get temperatures
    boolean ret = sensors.getAddress(tempDeviceAddress, 0);
    if (ret) {
        // transmit the temperature
        digitalWrite(TX_POWER_PIN, HIGH);    // power the transmitter
        delay(10);
        transmitTemperature(tempDeviceAddress); // Use a simple function to print out the data
        delay(10);
        digitalWrite(TX_POWER_PIN, LOW);     // disable the transmitter
    }

    for(int i=0; i<3; i++) {
        wdt_reset();                         // Get ready to go to sleep...
        watchdogEnable();                    // Turn on the watchdog timer
        sleepNow();                          // Go to sleep, watchdog timer will wake later
    }
}
 
void watchdogEnable() {                // Turn on watchdog timer; interrupt mode every 8.0s
    cli();
    MCUSR = 0;
    WDTCSR |= B00011000;
    //WDTCSR = B01000111;                // 2 Second Timeout
    //WDTCSR = B01100000;                // 4 Second Timeout
    WDTCSR = B01100001;                  // 8 Second Timeout
    sei();
}

// function to transmit the temperature of a device
void transmitTemperature(DeviceAddress deviceAddress)
{
    float tempC = sensors.getTempC(deviceAddress);

    tx.send(tempC, false, beep_on_first_tx);
    
    beep_on_first_tx = false;

    dprintln(tempC);
}

void sleepNow()
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and 
   * wake up sources are available in which sleep modes.
   *
   * In the avr/sleep.h file, the call names of these sleep modus are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings 
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   *  the power reduction management &lt;avr/power.h&gt;  is described in 
   *  http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
   */
      
  set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Sleep mode is set here
   
  sleep_enable();                      // Enables the sleep bit in the mcucr register
                                       // so sleep is possible. just a safety pin 
  sleep_mode();                        // Here the device is actually put to sleep!!
                                       // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();                     // Dirst thing after waking from sleep:
                                       // disable sleep...
}

ISR (WDT_vect) {                       // WDT Wakeup
    cli();
    wdt_disable();
    sei();
}