#include "pasco2_lib.hpp"

PASCO2_Lib::PASCO2_Lib() {}

/*!
 * @brief Starts I2C connection
 * @param addr I2C address of the HDC2080
 * @param wire The TwoWire master, defaults to &Wire
 * @return Returns true if successful
 */
bool PASCO2_Lib::begin(uint8_t addr, TwoWire *wire)
{
    if (i2c_dev)
    {
        delete i2c_dev; // remove old inteface
    }
    i2c_dev = new Adafruit_I2CDevice(addr, wire);

    /* Try to instantiate the I2C device. */
    if (!i2c_dev->begin(false))
    {
        return false;
    }
}

uint8_t PASCO2_Lib::read_i2c_register(uint8_t i2c_dev_addr, uint8_t i2c_reg_addr)
{
    // This function writes data to the buffer.
    // Wire.write(uint8_t);
    // Or specify qty of bytes to send with second param
    // Wire.write(const uint8_t *, size_t);

    // This function will finish the communication and release all the allocated resources.
    // After calling end you need to use begin again in order to initialize the I2C driver again.
    // wire.end();
    
    // After writing to the buffer using i2c write, use the function endTransmission to send the message to the slave device address defined on the beginTransmission function.
    // uint8_t endTransmission(bool sendStop);
    // sendStop enables (true) or disables (false) the stop signal (only used in master mode).
    // Calling the this function without sendStop is equivalent to sendStop = true.
    // uint8_t endTransmission(void);

    Wire.beginTransmission(i2c_dev_addr);
    Wire.write(i2c_reg_addr);
    Wire.endTransmission();

    // Request byte from slave
    if (Wire.requestFrom(i2c_dev_addr, 1U, 1U) > 0)
        return (Wire.read());
    else
        return (0x0);
}

uint8_t PASCO2_Lib::write_i2c_register(uint8_t i2c_dev_addr, uint8_t i2c_reg_addr, uint8_t data)
{
    Wire.beginTransmission(i2c_dev_addr);
    Wire.write(i2c_reg_addr);
    Wire.write(data);

    return Wire.endTransmission(true);
}

uint8_t PASCO2_Lib::getDeviceProductId()
{
    // Gets device product ID and return :
    // - 2 for PAS CO2 Gen 1
    // - 3 for PAS CO2 Gen 1.5
    uint8_t prod_id;

    prod_id = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_PROD_ID);
    prod_id &= XENSIV_PASCO2_REG_PROD_ID_PROD_MSK;
    prod_id = (prod_id >> 5);
    return (prod_id);
}

uint8_t PASCO2_Lib::getDeviceRevisionId()
{
    // Gets device revision ID and return :
    // - 1 for revision 1
    // - 2 for revision 2
    // - 3 ...
    uint8_t rev_id;

    rev_id = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_PROD_ID);
    rev_id &= XENSIV_PASCO2_REG_PROD_ID_REV_MSK;
    return (rev_id);
}

uint8_t PASCO2_Lib::getDeviceStatus()
{
    return (read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_SENS_STS));
}

uint8_t PASCO2_Lib::clearDeviceErrors()
{
    uint8_t bits_to_set;

    bits_to_set = XENSIV_PASCO2_REG_SENS_STS_ICCER_CLR_MSK + XENSIV_PASCO2_REG_SENS_STS_ORVS_CLR_MSK + XENSIV_PASCO2_REG_SENS_STS_ORTMP_CLR_MSK;

    return (write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_SENS_STS, bits_to_set));
}

uint16_t    PASCO2_Lib::setPressureRef(uint16_t press_ref)
{
    // Compute hex values (H & L) from unsigned short int pressure ref
    // Set atmospheric pressure reference in hPa, range are from 750 hPa to 1150 hPa
    // To update the pressure value, the user has to write first PRES_REF_H and then PRES_REF_L.

    uint8_t press_to_set_l = 0;
    uint8_t press_to_set_h = 0;
    press_to_set_l = press_ref &0xFFU;
    press_to_set_h = (press_ref &0xFF00U) >> 8;

    // Set new pressure reference value 757 hPa
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_PRESS_REF_H, press_to_set_h);
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_PRESS_REF_L, press_to_set_l);

    // Then check if the new value is set correctly

    uint8_t press_h = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_PRESS_REF_H);
    uint8_t press_l = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_PRESS_REF_L);
    
    uint16_t pressure_ref_read = 0;
    pressure_ref_read = press_h << 8 | press_l;

    return (pressure_ref_read);
}

uint16_t    PASCO2_Lib::getpressureRef()
{
    uint16_t pressure_ref = 0;
    uint8_t press_h = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_PRESS_REF_H);
    uint8_t press_l = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_PRESS_REF_L);

    pressure_ref = press_h << 8 | press_l;

    return (pressure_ref);
}

uint8_t PASCO2_Lib::setOpMode(uint8_t op_mode)
{
    // If SINGLE is set new operating mode can't be set until a certain amount of time (maybe 920ms)
    uint8_t actual_cfg;
    actual_cfg = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_CFG);
    actual_cfg &= ~(1 << 1);
    actual_cfg &= ~(1 << 0);
    // Set op mode to IDLE
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_CFG, actual_cfg);
    actual_cfg |= op_mode;
    // Write new config to register
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_CFG, actual_cfg); // default 0x24

    return (actual_cfg);
}

uint8_t PASCO2_Lib::setMeasRate(uint16_t meas_rate)
{
    // Measurement rate must be between 5s and 4095s
    // Compute hex values (H & L) from uint16_t meas_rate using 2's complement principle
    // For single-shot measurement make sure to delay the following measurement
    //  sequence by at least 60 s for accurate readings.

    uint8_t meas_rate_to_set_l = 0;
    uint8_t meas_rate_to_set_h = 0;
    meas_rate_to_set_l = meas_rate &0xFFU;
    meas_rate_to_set_h = (meas_rate &0xFF00U) >> 8;

    // Set new measurement rate value
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_RATE_H, meas_rate_to_set_h);
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_RATE_L, meas_rate_to_set_l);

    // Then check if the new value is set correctly
    meas_rate_to_set_h = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_RATE_H);
    meas_rate_to_set_l = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_RATE_L);
    
    uint16_t meas_rate_read = 0;
    meas_rate_read = (meas_rate_to_set_h << 8) | meas_rate_to_set_l;

    return (meas_rate_read);
}

uint8_t PASCO2_Lib::getMeasRate()
{
    // Measurement rate must be between 5s and 4095s
    // Get measurement rate
    uint16_t meas_rate = 0;
    uint8_t meas_rate_h = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_RATE_H);
    uint8_t meas_rate_l = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_RATE_L);

    meas_rate = (meas_rate_h << 8) | meas_rate_l;

    return (meas_rate);
}

/*!
 * @brief Gets the CO2 concentration over I2C from sensor
 * @return Returns the CO2 concentration in ppm
 */
uint16_t    PASCO2_Lib::getCO2Concentration()
{
    // Get CO2 concentration in ppm
    uint8_t co2_concentration_h = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_CO2PPM_H);
    delay(5);
    uint8_t co2_concentration_l = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_CO2PPM_L);
    delay(5);

    // Set MEAS_STS.INT_STS_CLR register to force pin INT to inactive level
    resetInterruptPin();

    co2Concentration = (co2_concentration_h << 8) | co2_concentration_l;

    return (co2Concentration);
}

bool    PASCO2_Lib::checkDataReady()
{
    uint8_t reg_sts = 0;
    bool drdy_sts;
    reg_sts = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_STS);

    drdy_sts = (reg_sts & XENSIV_PASCO2_REG_MEAS_STS_DRDY_MSK) >> XENSIV_PASCO2_REG_MEAS_STS_DRDY_POS;

    return (drdy_sts);
}
/**
 * @brief Get interrupt pin function configuration
 * 
 * @param int_cfg 
 * @return 1 if the interrupt pin is configured as alarm threshold violation notification pin
 * @return 2 if the interrupt pin is configured as data ready notification pin
 * @return 3 if the interrupt pin is configured as sensor busy notification pin
 * @return 4 if the interrupt pin is configured as early measurement start notification pin
 */
uint8_t PASCO2_Lib::getInterruptCfg()
{
    uint8_t int_sts = 0;

    int_sts = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_INT_CFG);
    int_sts &= XENSIV_PASCO2_REG_INT_CFG_INT_FUNC_MSK >> XENSIV_PASCO2_REG_INT_CFG_INT_FUNC_POS;

    return (int_sts);
}

/**
 * @brief Set interrupt pin function configuration
 * 
 * @param int_cfg 
 * @return uint8_t Register value after setting the interrupt pin function configuration
 */
uint8_t PASCO2_Lib::setInterruptReg(uint8_t int_cfg)
{
    uint8_t value_to_set = 0;

    value_to_set = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_INT_CFG);

    // Reset the 2nd 3rd & 4th bits to 0
    value_to_set &= ~(XENSIV_PASCO2_REG_INT_CFG_INT_FUNC_MSK);

    // Set register
    value_to_set |= (int_cfg << 1);
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_INT_CFG, value_to_set);
    
    // Check if the register are set correctly
    value_to_set = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_INT_CFG);

    return (value_to_set);
}

uint8_t PASCO2_Lib::resetInterruptPin()
{
    uint8_t reg_to_set = 0;

    reg_to_set = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_STS);
    reg_to_set |= (XENSIV_PASCO2_REG_MEAS_STS_INT_STS_CLR_MSK);

    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_STS, reg_to_set);

    reg_to_set = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_STS);

    return (reg_to_set);
}

uint8_t PASCO2_Lib::resetAlarmNotif()
{
    uint8_t reg_to_set = 0;

    reg_to_set = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_STS);
    reg_to_set |= (XENSIV_PASCO2_REG_MEAS_STS_ALARM_CLR_MSK);

    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_STS, reg_to_set);
    
    reg_to_set = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_MEAS_STS);

    return (reg_to_set);
}

uint16_t    PASCO2_Lib::getAlarmThreshold()
{
    uint16_t    result = 0;
    uint8_t     result_h;
    uint8_t     result_l;
    result_h = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_ALARM_TH_H);
    delay(5);
    result_l = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_ALARM_TH_L);
    delay(5);

    result = (result_h << 8) | result_l;
    return (result);
}

uint16_t    PASCO2_Lib::setAlarmThreshold(uint16_t alarm_thres)
{
    // Set alarm threshold

    uint8_t reg_to_set_l = 0;
    uint8_t reg_to_set_h = 0;
    reg_to_set_l = alarm_thres &0xFFU;
    reg_to_set_h = (alarm_thres &0xFF00U) >> 8;

    // Set new value
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_ALARM_TH_H, reg_to_set_h);
    write_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_ALARM_TH_L, reg_to_set_l);

    // Then check if the new value is set correctly

    uint8_t alarm_h = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_ALARM_TH_H);
    uint8_t alarm_l = read_i2c_register(XENSIV_PASCO2_I2C_ADDR, XENSIV_PASCO2_REG_ALARM_TH_L);
    
    uint16_t alarm_thres_read = 0;
    alarm_thres_read = alarm_h << 8 | alarm_l;

    return (alarm_thres_read);
}
