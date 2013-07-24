/*
 * Remote Water Thermometer
 *
 * Petr Stehlik
 *
 * GPL
 */

#include <OneWire.h>
#include <DallasTemperature.h>

#include "WaterTempTransmitter.h"

WaterTempTransmitter tx=WaterTempTransmitter(3 /* transmit Arduino pin */, 0x00 /* sensor ID */, 2 /* transmit channel */);

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

void setup()
{
    // start serial port
    Serial.begin(9600);
    Serial.println("DS18B20 433 MHz Thermometer");
    
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

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
        Serial.print("Found device with address: ");
        printAddress(tempDeviceAddress);
        Serial.println();

        Serial.print("Setting resolution to ");
        Serial.println(TEMPERATURE_PRECISION, DEC);

        // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
        sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

        Serial.print("Resolution actually set to: ");
        Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
        Serial.println();
    }
    else {
        Serial.print("Found ghost device but could not detect address. Check power and cabling");
    }
}

void loop()
{
    Serial.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    if(sensors.getAddress(tempDeviceAddress, 0))
    {
        // It responds almost immediately. Let's print out the data
        transmitTemperature(tempDeviceAddress); // Use a simple function to print out the data
        delay(3000);
    }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (deviceAddress[i] < 16) Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
    }
}

// function to transmit the temperature of a device
void transmitTemperature(DeviceAddress deviceAddress)
{
    float tempC = sensors.getTempC(deviceAddress);

    digitalWrite(13, HIGH);
    tx.send(tempC);
    digitalWrite(13, LOW);

    Serial.print(" ");
    Serial.println(tempC);
}

