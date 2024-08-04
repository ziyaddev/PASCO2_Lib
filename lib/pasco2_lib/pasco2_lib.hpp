#ifndef PASCO2_LIB_HPP
# define PASCO2_LIB_HPP

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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

/*
class PASCO2_Lib {
    public:
        PASCO2_Lib();

        bool begin(uint8_t addr = XENSIV_PASCO2_I2C_ADDR, TwoWire *wire = &Wire);

        void triggerMeasurement(void);
        
}
*/

// Prototypes declarations
uint8_t     read_i2c_register(uint8_t i2c_dev_addr, uint8_t i2c_reg_addr);
uint8_t     write_i2c_register(uint8_t i2c_dev_addr, uint8_t i2c_reg_addr, uint8_t data);
uint8_t     get_device_product_id();
uint8_t     get_device_revision_id();
uint8_t     clear_device_errors();
uint8_t     get_device_status();
uint16_t    set_pressure_ref(uint16_t press_ref);
uint16_t    get_pressure_ref();
uint8_t     set_op_mode(uint8_t op_mode);
uint8_t     set_meas_rate(uint16_t meas_rate);
uint8_t     get_meas_rate();
uint16_t    get_co2_concentration();
bool        check_drdy();
uint8_t     set_interrupt_reg(uint8_t int_cfg);
uint8_t     reset_interrupt_pin();
uint8_t     reset_alarm_notif();
uint16_t    get_alarm_threshold();
uint16_t    set_alarm_threshold(uint16_t alarm_thres);

// To-do :
// Automatic Baseline Offset Correction
// Forced compensation



#endif