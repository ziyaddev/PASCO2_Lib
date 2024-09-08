#include "pasco2_lib.hpp"

PASCO2_Lib::PASCO2_Lib() {}

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

    delay(400);
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
    delay(400);
    Wire.beginTransmission(i2c_dev_addr);
    Wire.write(i2c_reg_addr);
    Wire.write(data);

    return Wire.endTransmission(true);
}

/*!
 * @brief Reads to specified register
 */
uint8_t PASCO2_Lib::readReg(uint8_t regAddr) {
    Adafruit_BusIO_Register reg8 =
        Adafruit_BusIO_Register(i2c_dev, regAddr, 1);
    uint8_t read_result = reg8.read();
    delay(15);
    return (read_result);
}

/*!
 * @brief Writes to specified register
 * @param config Configuration settings to be written
 */
void PASCO2_Lib::writeReg(uint8_t config, uint8_t regAddr) {
    Adafruit_BusIO_Register reg8 =
        Adafruit_BusIO_Register(i2c_dev, regAddr, 1);
    reg8.write(config, 1);
    delay(15);
}

/*!
 * @brief Starts I2C connection
 * @param addr I2C address of the PASCO2 sensor
 * @param wire The TwoWire master, defaults to &Wire
 * @return Returns true if successful
 */
bool PASCO2_Lib::begin(uint8_t addr, TwoWire *wire)
{
    Serial.println("sensor init begin");
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

    softReset();

    delay(800);

    setOpMode(XENSIV_PASCO2_OP_MODE_IDLE);
    return true;
}

uint8_t PASCO2_Lib::getDeviceProductId()
{
    // Gets device product ID and return :
    // - 2 for PAS CO2 Gen 1
    // - 3 for PAS CO2 Gen 1.5
    uint8_t prod_id;

    prod_id = readReg(XENSIV_PASCO2_REG_PROD_ID);
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

    rev_id = readReg(XENSIV_PASCO2_REG_PROD_ID);
    rev_id &= XENSIV_PASCO2_REG_PROD_ID_REV_MSK;
    return (rev_id);
}

void PASCO2_Lib::softReset()
{
    // Soft reset the sensor
    writeReg(0xA3, XENSIV_PASCO2_REG_SENS_RST);
}

uint8_t PASCO2_Lib::getDeviceStatus()
{
    uint8_t dev_status = 0;
    dev_status = readReg(XENSIV_PASCO2_REG_SENS_STS);

    return (dev_status);
}

void PASCO2_Lib::clearDeviceErrors()
{
    uint8_t bits_to_set;

    bits_to_set = XENSIV_PASCO2_REG_SENS_STS_ICCER_CLR_MSK + XENSIV_PASCO2_REG_SENS_STS_ORVS_CLR_MSK + XENSIV_PASCO2_REG_SENS_STS_ORTMP_CLR_MSK;

    return (writeReg(bits_to_set, XENSIV_PASCO2_REG_SENS_STS));
}

uint16_t    PASCO2_Lib::getpressureRef()
{
    uint16_t pressure_ref = 0;

    pressure_ref = (readReg(XENSIV_PASCO2_REG_PRESS_REF_H) << 8) | readReg(XENSIV_PASCO2_REG_PRESS_REF_L);

    return (pressure_ref);
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

    // Set new pressure reference value
    writeReg(press_to_set_h, XENSIV_PASCO2_REG_PRESS_REF_H);
    writeReg(press_to_set_l, XENSIV_PASCO2_REG_PRESS_REF_L);

    // Check if the register are set correctly
    uint16_t pressure_ref_read = getpressureRef();

    return (pressure_ref_read);
}

/**
 * @brief Get baseline offset compensation configuration
 * 
 * @return  0 Automatic Baseline Offset Correction disabled.
 *          1 Automatic Baseline Offset Correction enabled. The offset is periodically updated at each BOC computation.
 *          2 forced compensation.
 */
uint8_t PASCO2_Lib::getBaselineOffsetCompensationCfg()
{
    uint8_t boc_cfg = 0;
    boc_cfg = readReg(XENSIV_PASCO2_REG_MEAS_CFG);
    boc_cfg &= (XENSIV_PASCO2_REG_MEAS_CFG_BOC_CFG_MSK >> XENSIV_PASCO2_REG_MEAS_CFG_BOC_CFG_POS);

    return (boc_cfg);
}

/**
 * @brief Set baseline offset compensation configuration
 * 
 * @param XENSIV_PASCO2_BOC_CFG_DISABLE, XENSIV_PASCO2_BOC_CFG_ENABLE, XENSIV_PASCO2_BOC_CFG_FORCE
 * @return  0 Automatic Baseline Offset Correction disabled.
 *          1 Automatic Baseline Offset Correction enabled. The offset is periodically updated at each BOC computation.
 *          2 forced compensation.
 */
uint8_t PASCO2_Lib::setBaselineOffsetCompensationCfg(uint8_t boc_cfg)
{
    uint8_t reg_to_set = 0;
    reg_to_set = readReg(XENSIV_PASCO2_REG_MEAS_CFG);

    // Reset the 3rd & 4th bits to 0
    reg_to_set &= ~(XENSIV_PASCO2_REG_MEAS_CFG_BOC_CFG_MSK);

    // Set register with parameter value
    reg_to_set |= boc_cfg;
    writeReg(reg_to_set, XENSIV_PASCO2_REG_MEAS_CFG);

    // Check if the register are set correctly
    reg_to_set = getBaselineOffsetCompensationCfg();

    return (reg_to_set);
}

/**
 * @brief Get the operating mode of the sensor
 * 
 * @return uint8_t 0 for IDLE, 1 for SINGLE, 2 for CONTINUOUS
 */
uint8_t PASCO2_Lib::getOpMode()
{
    uint8_t op_mode = 0;
    op_mode = readReg(XENSIV_PASCO2_REG_MEAS_CFG);

    op_mode &= (XENSIV_PASCO2_REG_MEAS_CFG_OP_MODE_MSK >> XENSIV_PASCO2_REG_MEAS_CFG_OP_MODE_POS);

    return (op_mode);
}

/**
 * @brief Set the operating mode of the sensor
 * 
 * @param op_mode XENSIV_PASCO2_OP_MODE_IDLE, XENSIV_PASCO2_OP_MODE_SINGLE, XENSIV_PASCO2_OP_MODE_CONTINUOUS
 * @return uint8_t 
 */
void PASCO2_Lib::setOpMode(uint8_t op_mode)
{
    // If SINGLE is set new operating mode can't be set until a certain amount of time (maybe 920ms)
    uint8_t reg_to_set;
    reg_to_set = readReg(XENSIV_PASCO2_REG_MEAS_CFG);

    // Reset the 1st & 2nd bits to 0
    reg_to_set &= ~(XENSIV_PASCO2_REG_MEAS_CFG_OP_MODE_MSK << XENSIV_PASCO2_REG_MEAS_CFG_OP_MODE_POS);

    delay(50);

    // Set operating mode
    reg_to_set |= op_mode;
    writeReg(op_mode, XENSIV_PASCO2_REG_MEAS_CFG);
}

uint8_t PASCO2_Lib::getMeasRate()
{
    // Measurement rate must be between 5s and 4095s
    // Get measurement rate

    uint16_t meas_rate = 0;

    meas_rate = (readReg(XENSIV_PASCO2_REG_MEAS_RATE_H) << 8) | readReg(XENSIV_PASCO2_REG_MEAS_RATE_L);

    return (meas_rate);
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
    writeReg(meas_rate_to_set_h, XENSIV_PASCO2_REG_MEAS_RATE_H);
    writeReg(meas_rate_to_set_l, XENSIV_PASCO2_REG_MEAS_RATE_L);

    // Check if the register are set correctly
    uint16_t meas_rate_read = 0;
    meas_rate_read = getMeasRate();

    return (meas_rate_read);
}

/*!
 * @brief Gets the CO2 concentration over I2C from sensor
 * @return Returns the CO2 concentration in ppm
 */
uint16_t    PASCO2_Lib::getCO2Concentration()
{
    // Get CO2 concentration in ppm
    uint8_t co2_concentration_h = readReg(XENSIV_PASCO2_REG_CO2PPM_H);
    delay(50);
    uint8_t co2_concentration_l = readReg(XENSIV_PASCO2_REG_CO2PPM_L);
    delay(50);

    // Set MEAS_STS.INT_STS_CLR register to force pin INT to inactive level
    resetInterruptPin();

    co2Concentration = (co2_concentration_h << 8) | co2_concentration_l;

    return (co2Concentration);
}

bool    PASCO2_Lib::checkDataReady()
{
    uint8_t reg_sts = 0;
    bool drdy_sts;
    reg_sts = readReg(XENSIV_PASCO2_REG_MEAS_STS);

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

    int_sts = readReg(XENSIV_PASCO2_REG_INT_CFG);
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

    value_to_set = readReg(XENSIV_PASCO2_REG_INT_CFG);

    // Reset the 2nd 3rd & 4th bits to 0
    value_to_set &= ~(XENSIV_PASCO2_REG_INT_CFG_INT_FUNC_MSK);

    // Set register
    value_to_set |= (int_cfg << 1);
    writeReg(value_to_set, XENSIV_PASCO2_REG_INT_CFG);

    // Check if the register are set correctly
    value_to_set = getInterruptCfg();

    return (value_to_set);
}

uint8_t PASCO2_Lib::resetInterruptPin()
{
    uint8_t reg_to_set = 0;

    reg_to_set = readReg(XENSIV_PASCO2_REG_MEAS_STS);
    reg_to_set |= (XENSIV_PASCO2_REG_MEAS_STS_INT_STS_CLR_MSK);

    writeReg(reg_to_set, XENSIV_PASCO2_REG_MEAS_STS);

    reg_to_set = readReg(XENSIV_PASCO2_REG_MEAS_STS);

    return (reg_to_set);
}

uint8_t PASCO2_Lib::resetAlarmNotif()
{
    uint8_t reg_to_set = 0;

    reg_to_set = readReg(XENSIV_PASCO2_REG_MEAS_STS);
    reg_to_set |= (XENSIV_PASCO2_REG_MEAS_STS_ALARM_CLR_MSK);

    writeReg(reg_to_set, XENSIV_PASCO2_REG_MEAS_STS);

    reg_to_set = readReg(XENSIV_PASCO2_REG_MEAS_STS);

    return (reg_to_set);
}

uint16_t    PASCO2_Lib::getAlarmThreshold()
{
    uint16_t result = 0;
    uint8_t     result_h;
    uint8_t     result_l;
    result_h = readReg(XENSIV_PASCO2_REG_ALARM_TH_H);
    delay(5);
    result_l = readReg(XENSIV_PASCO2_REG_ALARM_TH_L);
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
    writeReg(reg_to_set_h, XENSIV_PASCO2_REG_ALARM_TH_H);
    writeReg(reg_to_set_l, XENSIV_PASCO2_REG_ALARM_TH_L);

    // Check if the register are set correctly
    uint16_t alarm_thres_read = getAlarmThreshold();

    return (alarm_thres_read);
}

uint8_t PASCO2_Lib::getRegister(uint8_t reg)
{
    uint8_t reg_result = 0;

    reg_result = readReg(reg);
    return (reg_result);
}