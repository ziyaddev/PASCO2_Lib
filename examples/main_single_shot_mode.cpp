#include <Arduino.h>
#include <Wire.h>
#include "pasco2_lib.hpp"

// Initialize the XENSIV™ PASCO2 sensor object
PASCO2_Lib co2sensor;

#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 400000ul

#define PRESS_REFERENCE 1013
#define ALARM_THRESHOLD 1000
#define INT_PIN 2

void setup()
{
    Serial.begin(115200);
    Serial.println("\nSerial init OK");
    pinMode(INT_PIN, INPUT_PULLUP);
    Serial.println("2s delay ...");
    delay(2000);

    // Initialize the XENSIV™ PASCO2 sensor
    co2sensor.begin();
    // Time to sensor ready (see p.7 from datasheet)
    delay(1000);

    // Get device prod & rev id
    Serial.printf("\nDevice Product ID %d\n", co2sensor.getDeviceProductId());
    Serial.printf("Device Revision ID %d\n", co2sensor.getDeviceRevisionId());

    // Read device status
    Serial.printf("\nDevice status : 0x%X\n\t\t", co2sensor.getDeviceStatus());
    Serial.println(co2sensor.getDeviceStatus(), BIN);

    // Pressure reference configuration
    Serial.printf("Pressure ref : %u\n", co2sensor.getpressureRef());
    Serial.printf("Setting new pressure reference ... : %u\n", co2sensor.setPressureRef(PRESS_REFERENCE));

    // Read INT_CFG register
    Serial.printf("\nINTERRUPT_CFG REG : 0x%X\n", co2sensor.getInterruptCfg());

    // Set INT_CFG register as data ready notofication pin
    co2sensor.setInterruptReg(XENSIV_PASCO2_INT_FUNC_CFG_DRDY);

    Serial.printf("\nGet Baseline Offset Compensation Configuration : %d\n", co2sensor.getBaselineOffsetCompensationCfg());
    delay(2000);
}

void loop()
{
    // Setting single shot mode to trigger measurement
    co2sensor.setOpMode(XENSIV_PASCO2_OP_MODE_SINGLE);

    // Wait for measurement sequence to finish
    while (!digitalRead(INT_PIN))
    {
        Serial.print(".");
        delay(50);
    }
    Serial.println("");

    // Read CO2 PPM value from registers
    Serial.printf("CO2 concentration : %u ppm\n\n", co2sensor.getCO2Concentration());

    // Wait 30 sec before running new measurement
    delay(30000);
}
