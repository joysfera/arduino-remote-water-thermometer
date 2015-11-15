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
#include <VirtualWire.h>

#include "WaterTempTransmitter.h"

//#define DEBUG

#define ONE_WIRE_BUS_PIN    2    // Data wire is plugged into port 2 on the Arduino
#define TX433MHZ_PIN        3    // Transmitter Arduino pin
#define ONEWIRE_POWER_PIN   4    // power for one wire bus
#define TX_POWER_PIN        5    // power for transmitter
#define LED_BLINK          13    // LED indicator of transmitting

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
long vcc;

void setup()
{
    // start serial port
    debug_init();
    dprintln("DS18B20 433 MHz Thermometer");

    pinMode(ONEWIRE_POWER_PIN, OUTPUT);
    pinMode(TX_POWER_PIN, OUTPUT);
    pinMode(LED_BLINK, OUTPUT);
    pinMode(A0, INPUT);

    digitalWrite(LED_BLINK, LOW);

    digitalWrite(ONEWIRE_POWER_PIN, HIGH);    // power the one wire bus
    delay(250);

    // Start up the library
    sensors.begin();

    // Grab a count of devices on the wire
    int numberOfDevices = sensors.getDeviceCount();
    if (numberOfDevices == 0) {
        dprintln("Sensor(s) not found!");
        dead_end();
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
        dead_end();
    }

    vw_set_tx_pin(TX433MHZ_PIN);
    vw_setup(2000);	 // Bits per sec
}

void loop()
{
    digitalWrite(ONEWIRE_POWER_PIN, HIGH);    // power the one wire bus
    delay(250);

    dprint("Requesting temperatures... ");
    sensors.requestTemperatures(); // Send the command to get temperatures

    // fetch address of device (to initialize parasitely powered device?) and read temperature
    // use -0.1 C to indicate error reading the temperature
    float tempC = sensors.getAddress(tempDeviceAddress, 0) ? sensors.getTempC(tempDeviceAddress) : -0.1f;

    digitalWrite(ONEWIRE_POWER_PIN, LOW);     // disable power to the one wire bus

    // transmit the temperature
    digitalWrite(TX_POWER_PIN, HIGH);    // power the transmitter

    vcc = readVcc();
    boolean low_battery = (vcc < 3700);

    tx.send(tempC, low_battery, beep_on_first_tx);

    digitalWrite(LED_BLINK, HIGH);       // blink to indicate the transmit
    delay(5);
    digitalWrite(LED_BLINK, LOW);

    vwSendTempAndMore(tempC);

    digitalWrite(TX_POWER_PIN, LOW);     // disable power to the transmitter
    beep_on_first_tx = false;
    dprintln(tempC);

    // sleep for 3 x 8 seconds
    for(int i=0; i<3; i++) {
        wdt_reset();                         // Get ready to go to sleep...
        watchdogEnable();                    // Turn on the watchdog timer
        sleepNow();                          // Go to sleep, watchdog timer will wake later
    }
}
 
// Turn on watchdog timer; interrupt mode every 8.0s
void watchdogEnable()
{
    cli();
    MCUSR = 0;
    WDTCSR |= B00011000;
    //WDTCSR = B01000111;                // 2 Second Timeout
    //WDTCSR = B01100000;                // 4 Second Timeout
    WDTCSR = B01100001;                  // 8 Second Timeout
    sei();
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

    byte adcsra = ADCSRA;                // save the ADC Control and Status Register A
    ADCSRA = 0;                          // disable the ADC

    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Sleep mode is set here

    sleep_enable();                      // Enables the sleep bit in the mcucr register
                                         // so sleep is possible. just a safety pin
    sleep_mode();                        // Here the device is actually put to sleep!!
                                         // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    sleep_disable();                     // First thing after waking from sleep:
                                         // disable sleep...
    ADCSRA = adcsra;                     // restore ADCSRA = re-enable the ADC
}

// WDT Wakeup
ISR (WDT_vect)
{
    cli();
    wdt_disable();
    sei();
}

void dead_end()
{
    while(true) {
        digitalWrite(LED_BLINK, HIGH);
        delay(50);
        digitalWrite(LED_BLINK, LOW);
        delay(450);
    }
}

void vwSendTempAndMore(float temp)
{
    char msg[VW_MAX_MESSAGE_LEN] = "Bazen:";

    int t = (int)(temp * 10);
    itoa(t, msg + strlen(msg), DEC);
    strcat(msg, ":");

    itoa(vcc, msg + strlen(msg), DEC);
    strcat(msg, ":");

    long solar = analogRead(A0) * vcc / 511L; // 1023 = vcc * 2 (voltage divider on A0)
    itoa(solar, msg + strlen(msg), DEC);

    vw_send((uint8_t *)msg, strlen(msg));
    vw_wait_tx(); // Wait until the whole message is gone
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1119162L / result; // Calculate Vcc (in mV); 1119162 = 1.094*1023*1000
  return result; // Vcc in millivolts
}
