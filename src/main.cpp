//////////////////CLOUD BEGIN//////////////////
#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif
//////////////////CLOUD END//////////////////

#include <Arduino.h>
#include <Wire.h>
#include "pasco2_lib.hpp"

//////////////////CLOUD BEGIN//////////////////
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include "creds.h"

// Time zone info
#define TZ_INFO "UTC1"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point pasco2_sensor("iaq_sensor");
//////////////////CLOUD END//////////////////

// Initialize the XENSIV™ PASCO2 sensor object
PASCO2_Lib co2sensor;

#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 400000

#define PRESS_REFERENCE 980
#define ALARM_THRESHOLD 1000

void setup()
{
    delay(2000);

    Serial.begin(115200);
    Serial.println("\nSerial init OK");
    if (!Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ))
        Serial.println("\nFailed to initialize i2c bus !\n");
    Serial.println("\nI2C init OK\n");
    Serial.println("3s delay ...");
    delay(3000);

    // Set the bus timeout given in milliseconds. The default value is 50ms.
    // Wire.setTimeOut(100);

    // Initialize the XENSIV™ PASCO2 sensor
    co2sensor.begin();

    // Read device status
    Serial.printf("\nDevice status : 0x%X\n", co2sensor.getDeviceStatus());

    delay(1000);

    Serial.println("\n------------------");
    Serial.printf("Actual pressure ref : %u\n", co2sensor.getpressureRef());
    co2sensor.setPressureRef(PRESS_REFERENCE);
    Serial.printf("New pressure ref : %u\n", co2sensor.getpressureRef());

    delay(500);

    // Get alarm threshold setting
    Serial.printf("\nAlarm threshold : %u\n",co2sensor.getAlarmThreshold());

    // Set alarm threshold
    Serial.printf("Alarm threshold set : %u\n", co2sensor.setAlarmThreshold(ALARM_THRESHOLD));
    // Get alarm threshold setting
    Serial.printf("Alarm threshold : %u\n", co2sensor.getAlarmThreshold());

    // Reset alarm notification bit
    Serial.printf("Reset alarm notification : %u\n", co2sensor.resetAlarmNotif());

    // Read alarm threshold

    // // Read MEAS_CFG register
    // uint8_t meas_reg = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_CFG);
    // Serial.printf("\nMEAS_CFG REG : 0x%X\n", meas_reg);

    // Read INT_CFG register
    Serial.printf("\nINTERRUPT_CFG REG : 0x%X\n", co2sensor.getInterruptCfg());

    // Set INT_CFG register
    co2sensor.setInterruptReg(XENSIV_PASCO2_INT_FUNC_CFG_DRDY);

    // Read again INT_CFG register
    Serial.printf("\nINTERRUPT_CFG REG after setting it : 0x%X\n", co2sensor.getInterruptCfg());

    // Set measurement rate
    Serial.printf("\nMeasurement rate before  : %u", co2sensor.getMeasRate());
    Serial.printf("\nMeasurement rate setting : %u", co2sensor.setMeasRate(XENSIV_PASCO2_MEAS_RATE_SET));
    Serial.printf("\nMeasurement rate after   : %u\n", co2sensor.getMeasRate());

    // Set operating mode in MEAS_CFG register
    Serial.printf("Operating mode [IDLE]       : 0x%X\n\n", co2sensor.setOpMode(XENSIV_PASCO2_OP_MODE_IDLE));
    delay(400);

    Serial.printf("Operating mode [SINGLE]     : 0x%X\n\n", co2sensor.setOpMode(XENSIV_PASCO2_OP_MODE_SINGLE));

    // Serial.printf("- Operating mode [CONTINUOUS] : 0x%X\n\n", co2sensor.setOpMode(XENSIV_PASCO2_OP_MODE_CONTINUOUS));

    Serial.printf("\nGet Baseline Offset Compensation Configuration : %d\n", co2sensor.getBaselineOffsetCompensationCfg());
    delay(2000);
}

void loop()
{
    if (co2sensor.checkDataReady())
    {
        Serial.printf("\nRead co2 concentration : %u ppm\n", co2sensor.getCO2Concentration());
    }
    delay(15000);
}

// void    loop()
// {
//     // Trigger single measurement
//     write_i2c_register(0x28, 0x04, 0x01);
//     delay(1000);
//     // Get PPM value
//     uint8_t value1 = read_i2c_register(0x28, 0x05);
//     delay(5);
//     uint8_t value2 = read_i2c_register(0x28, 0x06);
//     delay(5);
//     // Calculate ppm value
//     int16_t result = value1 << 8 | value2;
//     Serial.print("CO2: ");
//     Serial.print(result);
//     Serial.println(" ppm");
//     delay(10000);
// }