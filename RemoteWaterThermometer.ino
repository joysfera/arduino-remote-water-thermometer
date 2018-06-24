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

#define TEMPERATURE_PRECISION 11 // 375 milliseconds for temperature conversion is better than 750 ms

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
unsigned int vcc;

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
    delay(50);

    // Start up the library
    sensors.begin();
    if(sensors.getAddress(tempDeviceAddress, 0)) {
        sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    }

    vw_set_tx_pin(TX433MHZ_PIN);
    vw_setup(2000);	 // Bits per sec
}

void loop()
{
    digitalWrite(ONEWIRE_POWER_PIN, HIGH);    // power the one wire bus
    delay(50);

    float tempC = -33.3f;
    if(sensors.getAddress(tempDeviceAddress, 0)) {
        dprint("Requesting temperatures... ");
        sensors.requestTemperatures(); // Send the command to get temperatures

        tempC = sensors.getTempC(tempDeviceAddress);
    }

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

#ifdef DEBUG
    delay(3000);
#else
    // sleep for 5 x 8 = 40 seconds
    for(byte i=0; i<5; i++) {
        wdt_reset();                         // Get ready to go to sleep...
        watchdogEnable();                    // Turn on the watchdog timer
        sleepNow();                          // Go to sleep, watchdog timer will wake us up in 8 seconds
    }
#endif
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
    ADCSRA = 0;                          // disable the ADC (saves 300 µA)

    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Sleep mode is set here
    cli();
    sleep_enable();                      // Enables the sleep bit in the mcucr register
                                         // so sleep is possible. just a safety pin
    sleep_bod_disable();                 // disable brown-out detection while sleeping (20-25 µA)
    sei();
    sleep_cpu();                         // Here the device is actually put to sleep!!
                                         // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    sleep_disable();                     // First thing after waking from sleep:
                                         // disable sleep...
    ADCSRA = adcsra;                     // restore ADCSRA = re-enable the ADC
}

// WDT Wakeup
ISR(WDT_vect)
{
    wdt_disable();
}

void vwSendTempAndMore(float temp)
{
    char msg[VW_MAX_MESSAGE_LEN] = "Bazen:";

    int t = (int)(temp * 10);
    itoa(t, msg + strlen(msg), DEC);
    strcat(msg, ":");

    itoa(vcc, msg + strlen(msg), DEC);
    strcat(msg, ":");

    unsigned int solar = analogRead(A0) * vcc / 512UL; // 1024 = vcc * 2 (voltage divider on A0)
    itoa(solar, msg + strlen(msg), DEC);

    vw_send((uint8_t *)msg, strlen(msg));
    vw_wait_tx(); // Wait until the whole message is sent
}

unsigned int readVcc() {
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
    unsigned int a = ADCW;
    unsigned int result = 1119162UL / a; // Calculate Vcc (in mV)
    return result; // Vcc in millivolts
}
