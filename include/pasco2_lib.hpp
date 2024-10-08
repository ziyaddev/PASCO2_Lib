#ifndef PASCO2_LIB_HPP
# define PASCO2_LIB_HPP

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_BusIO_Register.h>
#include <Wire.h>

#include "xensiv_pasco2_regs.hpp"

/************************************** Macros *******************************************/

/** Result code indicating a successful operation */
#define XENSIV_PASCO2_OK                    (0)
/** Result code indicating a communication error */
#define XENSIV_PASCO2_ERR_COMM              (1)
/** Result code indicating that an unexpectedly large I2C write was requested which is not supported */
#define XENSIV_PASCO2_ERR_WRITE_TOO_LARGE   (2)
/** Result code indicating that the sensor is not yet ready after reset */
#define XENSIV_PASCO2_ERR_NOT_READY         (3)
/** Result code indicating whether a non-valid command has been received by the serial communication interface */
#define XENSIV_PASCO2_ICCERR                (4)
/** Result code indicating whether a condition where VDD12V has been outside the specified valid range has been detected */
#define XENSIV_PASCO2_ORVS                  (5)
/** Result code indicating whether a condition where the temperature has been outside the specified valid range has been detected */
#define XENSIV_PASCO2_ORTMP                 (6)
/** Result code indicating that a new CO2 value is not yet ready */
#define XENSIV_PASCO2_READ_NRDY             (7)

/** Minimum allowed measurement rate */
#define XENSIV_PASCO2_MEAS_RATE_MIN         (5U)

/** Maximum allowed measurement rate */
#define XENSIV_PASCO2_MEAS_RATE_MAX         (4095U)

/** I2C address of the XENSIV™ PASCO2 sensor */
#define XENSIV_PASCO2_I2C_ADDR              (0x28U)


#define XENSIV_PASCO2_OP_MODE_IDLE          (0U)
#define XENSIV_PASCO2_OP_MODE_SINGLE        (1U)
#define XENSIV_PASCO2_OP_MODE_CONTINUOUS    (2U)

#define XENSIV_PASCO2_BOC_CFG_DISABLE       (0U)
#define XENSIV_PASCO2_BOC_CFG_ENABLE        (1U)
#define XENSIV_PASCO2_BOC_CFG_FORCE         (2U)


// 000b: Pin INT is inactive.
// 001b: Pin INT is configured as alarm threshold violation notification
// pin.
// 010b: Pin INT is configured as data ready notification pin.
// 011b: Pin INT is configured as sensor busy notification pin.
// 100b: Pin INT is configured as early measurement start notification
//          pin (this function only is available in continuous mode with
//          MEAS_CFG.OP_MODE = 10b, otherwise the pin is inactive).

#define XENSIV_PASCO2_INT_FUNC_CFG_INACTIVE     (0U)
#define XENSIV_PASCO2_INT_FUNC_CFG_ALARM_THR    (1U)
#define XENSIV_PASCO2_INT_FUNC_CFG_DRDY         (2U)
#define XENSIV_PASCO2_INT_FUNC_CFG_SENS_BUSY    (3U)
#define XENSIV_PASCO2_INT_FUNC_CFG_EARLY_MEAS   (4U)


#define XENSIV_PASCO2_MEAS_RATE_SET         (5U)

class PASCO2_Lib {
    public:
        PASCO2_Lib();

        bool begin(uint8_t addr = XENSIV_PASCO2_I2C_ADDR, TwoWire *wire = &Wire);

        void        softReset();
        uint8_t     getDeviceProductId();
        uint8_t     getDeviceRevisionId();
        uint8_t     getDeviceStatus();
        void        clearDeviceErrors();
        uint16_t    getpressureRef();
        uint16_t    setPressureRef(uint16_t press_ref);
        uint8_t     getBaselineOffsetCompensationCfg();
        uint8_t     setBaselineOffsetCompensationCfg(uint8_t boc_cfg);
        uint8_t     getOpMode();
        void        setOpMode(uint8_t op_mode);
        uint8_t     setMeasRate(uint16_t meas_rate);
        uint8_t     getMeasRate();
        uint16_t    getCO2Concentration();
        bool        checkDataReady();
        uint8_t     getInterruptCfg();
        uint8_t     setInterruptReg(uint8_t int_cfg);
        uint8_t     resetInterruptPin();
        uint8_t     resetAlarmNotif();
        uint16_t    getAlarmThreshold();
        uint16_t    setAlarmThreshold(uint16_t alarm_thres);
        uint8_t     getRegister(uint8_t reg);

    private:
        uint16_t    co2Concentration = 0;
        uint8_t     read_i2c_register(uint8_t i2c_dev_addr, uint8_t i2c_reg_addr);
        uint8_t     write_i2c_register(uint8_t i2c_dev_addr, uint8_t i2c_reg_addr, uint8_t data);
        void        writeReg(uint8_t config, uint8_t regAddr);
        uint8_t     readReg(uint8_t regAddr);
        Adafruit_I2CDevice *i2c_dev = NULL; // < Pointer to I2c bus interface
};

#endif
