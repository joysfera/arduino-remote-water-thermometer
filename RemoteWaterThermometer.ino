/*
 * Remote Water Thermometer
 *
 * Petr Stehlik
 *
 * GPL
 */

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "WaterTempTransmitter.h"

#define ONE_WIRE_BUS_PIN    2    // Data wire is plugged into port 2 on the Arduino
#define TX433MHZ_PIN        3    // Transmitter Arduino pin
#define BUS_POWER_PIN       4    // power for DS18B20 one wire bus - doesn't work well for parasite mode
#define TX_POWER_PIN        5    // power for transmitter

WaterTempTransmitter tx=WaterTempTransmitter(TX433MHZ_PIN, 0x00 /* sensor ID */, 2 /* transmit channel */);

#define TEMPERATURE_PRECISION 11

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS_PIN);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

void setup()
{
    // start serial port
    Serial.begin(9600);
    Serial.println("DS18B20 433 MHz Thermometer");
    
    pinMode(BUS_POWER_PIN, OUTPUT);
    pinMode(TX_POWER_PIN, OUTPUT);
    
    digitalWrite(BUS_POWER_PIN, HIGH);    // power the one wire sensors bus

    // Start up the library
    sensors.begin();

    // Grab a count of devices on the wire
    int numberOfDevices = sensors.getDeviceCount();
    if (numberOfDevices == 0) {
        Serial.println("Sensor(s) not found!");
        while(1);
    }     

    // locate devices on the bus
    Serial.print("Locating devices...");

    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");

    // report parasite power requirements
    Serial.print("Parasite power is: "); 
    if (sensors.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");

    if(sensors.getAddress(tempDeviceAddress, 0)) {
        Serial.print("Device found, setting resolution to ");
        Serial.println(TEMPERATURE_PRECISION, DEC);

        // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
        sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

        Serial.print("Resolution actually set to: ");
        Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
        Serial.println();
    }
    else {
        Serial.print("Found ghost device but could not detect address. Check power and cabling");
        while(true) ;
    }
}

void loop()
{
    Serial.print("Requesting temperatures...");
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

    tx.send(tempC);

    Serial.print(" ");
    Serial.println(tempC);
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
