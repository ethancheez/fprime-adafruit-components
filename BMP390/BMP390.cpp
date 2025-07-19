// ======================================================================
// \title  BMP390.cpp
// \author ethan
// \brief  cpp file for BMP390 component implementation class
// ======================================================================

#include "Components/BMP390/BMP390.hpp"
#include <Fw/Logger/Logger.hpp>
#include <cmath>

namespace Adafruit {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

BMP390 ::BMP390(const char* const compName) : BMP390ComponentBase(compName) {}

BMP390 ::~BMP390() {}

void BMP390::config() {
    Drv::I2cStatus stat;
    U8 ret;

    // Software reset
    bool resetSuccess = this->reset();
    FW_ASSERT(resetSuccess, resetSuccess);

    // Read chip ID
    stat = this->readRegister(BMP3_REG_CHIP_ID, ret);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
    FW_ASSERT(ret == BMP3_CHIP_ID || ret == BMP390_CHIP_ID, ret);
    Fw::Logger::log("BMP390 Chip ID: 0x%02X\n", ret);

    // Get calibration data
    U8 reg_data[BMP3_LEN_CALIB_DATA] = {0};
    Fw::Buffer calib_data(reg_data, BMP3_LEN_CALIB_DATA);
    stat = this->readRegisterBlock(BMP3_REG_CALIB_DATA, calib_data);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

    // Verify calibration data checksum
    U8 crc = 0xFF;
    for (U32 i = 0; i < BMP3_LEN_CALIB_DATA; i++) {
        crc = static_cast<U8>(cal_crc(crc, calib_data.getData()[i]));
    }
    crc = (crc ^ 0xFF);
    stat = this->readRegister(0x30, ret);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
    FW_ASSERT(ret == crc, ret);

    // Parse calibration data
    this->parse_calib_data(calib_data);

    // Default settings
    this->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    this->setPressureOversampling(BMP3_OVERSAMPLING_4X);
    this->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    this->setOutputDataRate(BMP3_ODR_200_HZ);

    // don't do anything till we request a reading
    this->settings.op_mode = BMP3_MODE_FORCED;

    Fw::Logger::log("BMP390 Init Success\n");
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void BMP390 ::run_handler(FwIndexType portNum, U32 context) {
    if (!this->performReading()) {
        return;
    }

    F64 altitude = 44307.7 * (1.0 - pow(this->m_pressure / this->m_seaLevelPressure, 0.190284));

    BMP390_DataUnit_F64 altitudeData;
    altitudeData.set(altitude, Fw::String("m"));
    BMP390_DataUnit_F64 temperatureData;
    temperatureData.set(this->m_temperature, Fw::String("C"));
    BMP390_DataUnit_F64 pressureData;
    pressureData.set(this->m_pressure, Fw::String("hPa"));

    this->tlmWrite_Altitude(altitudeData);
    this->tlmWrite_TemperatureC(temperatureData);
    this->tlmWrite_Pressure(pressureData);

    // Fw::Logger::log("BMP390 Readings: Altitude=%.2f m, Temperature=%.2f C, Pressure=%.2f hPa\n", altitude,
    //                 this->m_temperature, this->m_pressure);
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void BMP390 ::SET_SEA_LEVEL_PRESSURE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F64 sea_level_pressure) {
    this->m_seaLevelPressure = sea_level_pressure;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Helper Functions
// ----------------------------------------------------------------------

Drv::I2cStatus BMP390::readRegister(U8 startRegister, U8& byte) {
    U8 reg_data[1] = {0};
    Fw::Buffer buffer(reg_data, 1);
    buffer.getData()[0] = startRegister;

    Drv::I2cStatus status = this->i2cWriteRead_out(0, this->m_i2cDevAddress, buffer, buffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(status);
        return status;
    }
    byte = buffer.getData()[0];
    return status;
}

Drv::I2cStatus BMP390::readRegisterBlock(U8 startRegister, Fw::Buffer& buffer) {
    Drv::I2cStatus status;
    status = this->setupReadRegister(startRegister);
    if (status == Drv::I2cStatus::I2C_OK) {
        status = this->i2cRead_out(0, m_i2cDevAddress, buffer);
    }
    if (status != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(status);
    }

    return status;
}

Drv::I2cStatus BMP390::setupReadRegister(U8 reg) {
    U8 reg_data[1] = {0};
    Fw::Buffer buffer(reg_data, 1);
    buffer.getData()[0] = reg;
    Drv::I2cStatus stat = this->i2cWrite_out(0, this->m_i2cDevAddress, buffer);
    return stat;
}

Drv::I2cStatus BMP390::writeRegister(U8 startRegister, U8 byte) {
    U8 reg_data[2] = {0};
    Fw::Buffer buffer(reg_data, 2);
    buffer.getData()[0] = startRegister;
    buffer.getData()[1] = byte;
    Drv::I2cStatus stat = this->i2cWrite_out(0, this->m_i2cDevAddress, buffer);

    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
    }

    return stat;
}

Drv::I2cStatus BMP390::writeRegisterBlock(U8 startRegister, Fw::Buffer& buffer) {
    U8 reg_data[buffer.getSize() + 1];
    Fw::Buffer data(reg_data, buffer.getSize() + 1);
    data.getData()[0] = startRegister;
    memcpy(&data.getData()[1], buffer.getData(), buffer.getSize());
    Drv::I2cStatus stat = this->i2cWrite_out(0, this->m_i2cDevAddress, data);

    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
    }

    return stat;
}

bool BMP390::reset() {
    Drv::I2cStatus stat;

    // Software Reset
    stat = this->writeRegister(BMP3_REG_CMD, BMP3_SOFT_RESET);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return false;
    }

    // Wait 2ms for reset to complete
    Os::Task::delay(Fw::TimeInterval(0, 2000));

    // Read for command error status
    U8 ret;
    stat = this->readRegister(BMP3_REG_ERR, ret);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return false;
    }
    if (ret & BMP3_REG_CMD) {
        return false;
    }

    return true;
}

void BMP390::setTemperatureOversampling(U8 oversample) {
    if (oversample > BMP3_OVERSAMPLING_32X)
        FW_ASSERT(0);

    this->settings.odr_filter.temp_os = oversample;

    if (oversample == BMP3_NO_OVERSAMPLING)
        this->_tempOSEnabled = false;
    else
        this->_tempOSEnabled = true;
}

void BMP390::setPressureOversampling(U8 oversample) {
    if (oversample > BMP3_OVERSAMPLING_32X)
        FW_ASSERT(0);

    this->settings.odr_filter.press_os = oversample;

    if (oversample == BMP3_NO_OVERSAMPLING)
        this->_presOSEnabled = false;
    else
        this->_presOSEnabled = true;
}

bool BMP390::setIIRFilterCoeff(U8 filtercoeff) {
    if (filtercoeff > BMP3_IIR_FILTER_COEFF_127)
        return false;

    this->settings.odr_filter.iir_filter = filtercoeff;

    if (filtercoeff == BMP3_IIR_FILTER_DISABLE)
        this->_filterEnabled = false;
    else
        this->_filterEnabled = true;

    return true;
}

bool BMP390::setOutputDataRate(U8 odr) {
    if (odr > BMP3_ODR_0_001_HZ)
        return false;

    this->settings.odr_filter.odr = odr;

    this->_ODREnabled = true;

    return true;
}

bool BMP390::performReading(void) {
    Drv::I2cStatus stat;
    U16 settings_sel = 0;
    U8 sensor_comp = 0;

    /* Select the pressure and temperature sensor to be enabled */
    this->settings.temp_en = BMP3_ENABLE;
    settings_sel |= BMP3_SEL_TEMP_EN;
    sensor_comp |= BMP3_TEMP;
    if (this->_tempOSEnabled) {
        settings_sel |= BMP3_SEL_TEMP_OS;
    }

    this->settings.press_en = BMP3_ENABLE;
    settings_sel |= BMP3_SEL_PRESS_EN;
    sensor_comp |= BMP3_PRESS;
    if (this->_presOSEnabled) {
        settings_sel |= BMP3_SEL_PRESS_OS;
    }

    if (this->_filterEnabled) {
        settings_sel |= BMP3_SEL_IIR_FILTER;
    }

    if (this->_ODREnabled) {
        settings_sel |= BMP3_SEL_ODR;
    }

    stat = this->setSensorSettings(settings_sel);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return false;
    }

    /* Set the power mode */
    stat = this->writeRegister(BMP3_REG_PWR_CTRL, 0x13);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return false;
    }

    /* Get sensor data */
    struct bmp3_data data;
    U8 reg_data[BMP3_LEN_P_T_DATA] = {0};
    struct bmp3_uncomp_data uncomp_data = {0};

    Fw::Buffer buffer(reg_data, BMP3_LEN_P_T_DATA);
    stat = this->readRegisterBlock(BMP3_REG_DATA, buffer);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return false;
    }

    /* Temporary variables to store the sensor data */
    U32 data_xlsb;
    U32 data_lsb;
    U32 data_msb;

    /* Store the parsed register values for pressure data */
    data_xlsb = (U32)buffer.getData()[0];
    data_lsb = (U32)buffer.getData()[1] << 8;
    data_msb = (U32)buffer.getData()[2] << 16;
    uncomp_data.pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_xlsb = (U32)buffer.getData()[3];
    data_lsb = (U32)buffer.getData()[4] << 8;
    data_msb = (U32)buffer.getData()[5] << 16;
    uncomp_data.temperature = data_msb | data_lsb | data_xlsb;

    /* Compensate the pressure/temperature/both data read from the sensor */
    /* If pressure or temperature component is selected */
    if (sensor_comp * (BMP3_PRESS | BMP3_TEMP)) {
        data.temperature = this->compensate_temperature(&uncomp_data);
    }
    if (sensor_comp & BMP3_PRESS) {
        data.pressure = this->compensate_pressure(&uncomp_data);
    }

    this->m_temperature = data.temperature;
    this->m_pressure = data.pressure / 100.0;  // Convert to hPa

    return true;
}

Drv::I2cStatus BMP390::sleep() {
    Drv::I2cStatus stat;
    U8 reg_data;

    stat = this->readRegister(BMP3_REG_PWR_CTRL, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    /* Set the power mode */
    reg_data = reg_data & (~(BMP3_OP_MODE_MSK));

    /* Write the power mode in the register */
    return this->writeRegister(BMP3_REG_PWR_CTRL, reg_data);
}

Drv::I2cStatus BMP390::setSensorSettings(U32 desired_settings) {
    Drv::I2cStatus stat = Drv::I2cStatus::I2C_OK;

    /* Set the power control settings */
    if (BMP3_POWER_CNTL & desired_settings) {
        stat = set_pwr_ctrl_settings(desired_settings);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
    }

    /* Set the over sampling, ODR and filter settings*/
    if (BMP3_ODR_FILTER & desired_settings) {
        stat = set_odr_filter_settings(desired_settings);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
    }

    if (BMP3_INT_CTRL & desired_settings) {
        /* Set the interrupt control settings */
        stat = set_int_ctrl_settings(desired_settings);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
    }

    if (BMP3_ADV_SETT & desired_settings) {
        /* Set the advance settings */
        stat = set_advance_settings(desired_settings);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
    }

    return stat;
}

Drv::I2cStatus BMP390::set_pwr_ctrl_settings(U32 desired_settings) {
    Drv::I2cStatus stat;
    U8 reg_data;

    stat = this->readRegister(BMP3_REG_PWR_CTRL, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    if (desired_settings & BMP3_SEL_PRESS_EN) {
        /* Set the pressure enable settings in the register variable */
        reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_PRESS_EN, this->settings.press_en);
    }

    if (desired_settings & BMP3_SEL_TEMP_EN) {
        /* Set the temperature enable settings in the register variable */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_EN, this->settings.temp_en);
    }

    return this->writeRegister(desired_settings, reg_data);
}

Drv::I2cStatus BMP390::set_odr_filter_settings(U32 desired_settings) {
    Drv::I2cStatus stat;
    U8 reg_addr[3] = {0};
    U8 len = 0;

    U8 reg_data[4] = {0};
    Fw::Buffer buffer(reg_data, 4);
    stat = this->readRegisterBlock(BMP3_REG_OSR, buffer);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    if ((BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS) & desired_settings) {
        this->fill_osr_data(desired_settings, reg_addr, buffer, len);
    }

    if (BMP3_SEL_ODR & desired_settings) {
        this->fill_odr_data(reg_addr, buffer, len);
    }

    if (BMP3_SEL_IIR_FILTER & desired_settings) {
        this->fill_filter_data(reg_addr, buffer, len);
    }

    if (this->settings.op_mode == BMP3_MODE_NORMAL) {
        validate_osr_and_odr_settings();
    }

    buffer.setSize(len);
    return this->writeRegisterBlock(BMP3_REG_OSR, buffer);
}

Drv::I2cStatus BMP390::set_int_ctrl_settings(U32 desired_settings) {
    Drv::I2cStatus stat;
    U8 reg_data;

    stat = this->readRegister(BMP3_REG_INT_CTRL, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    struct bmp3_int_ctrl_settings int_settings = this->settings.int_settings;

    if (desired_settings & BMP3_SEL_OUTPUT_MODE) {
        /* Set the interrupt output mode bits */
        reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_INT_OUTPUT_MODE, int_settings.output_mode);
    }

    if (desired_settings & BMP3_SEL_LEVEL) {
        /* Set the interrupt level bits */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_INT_LEVEL, int_settings.level);
    }

    if (desired_settings & BMP3_SEL_LATCH) {
        /* Set the interrupt latch bits */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_INT_LATCH, int_settings.latch);
    }

    if (desired_settings & BMP3_SEL_DRDY_EN) {
        /* Set the interrupt data ready bits */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_INT_DRDY_EN, int_settings.drdy_en);
    }

    return this->writeRegister(BMP3_REG_INT_CTRL, reg_data);
}

Drv::I2cStatus BMP390::set_advance_settings(U32 desired_settings) {
    Drv::I2cStatus stat;
    U8 reg_data;

    struct bmp3_adv_settings adv_settings = this->settings.adv_settings;

    stat = this->readRegister(BMP3_REG_IF_CONF, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    if (desired_settings & BMP3_SEL_I2C_WDT_EN) {
        /* Set the i2c watch dog enable bits */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_I2C_WDT_EN, adv_settings.i2c_wdt_en);
    }

    if (desired_settings & BMP3_SEL_I2C_WDT) {
        /* Set the i2c watch dog select bits */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_I2C_WDT_SEL, adv_settings.i2c_wdt_sel);
    }

    return this->writeRegister(BMP3_REG_IF_CONF, reg_data);
}

Drv::I2cStatus BMP390::set_op_mode(U8 opmode) {
    Drv::I2cStatus stat;

    U8 curr_mode = this->settings.op_mode;

    // Get previous opmode
    U8 last_set_mode;
    stat = this->get_op_mode(last_set_mode);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    // If the sensor is not in sleep mode put the device to sleep mode
    if (last_set_mode != BMP3_MODE_SLEEP) {
        stat = this->sleep();
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }

        // Give some time to sleep
        Os::Task::delay(Fw::TimeInterval(0, 5000));  // 5 us
    }

    // Set the power mode
    if (curr_mode == BMP3_MODE_NORMAL) {
        stat = this->set_normal_mode();
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
    } else if (curr_mode == BMP3_MODE_FORCED) {
        stat = this->write_power_mode();
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
    }

    return stat;
}

Drv::I2cStatus BMP390::get_op_mode(U8& opmode) {
    Drv::I2cStatus stat;
    U8 reg_data;

    /* Read the power mode register */
    stat = this->readRegister(BMP3_REG_PWR_CTRL, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    /* Assign the power mode in the device structure */
    opmode = BMP3_GET_BITS(reg_data, BMP3_OP_MODE);

    return stat;
}

Drv::I2cStatus BMP390::set_normal_mode() {
    Drv::I2cStatus stat;

    stat = this->validate_normal_mode_settings();
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    stat = this->write_power_mode();
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    U8 conf_err_status;
    stat = this->readRegister(BMP3_REG_ERR, conf_err_status);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    if (conf_err_status & BMP3_ERR_CONF) {
        FW_ASSERT(0);  // TODO?
    }

    return stat;
}

Drv::I2cStatus BMP390::write_power_mode() {
    Drv::I2cStatus stat;
    U8 op_mode_reg_val;

    /* Read the power mode register */
    stat = this->readRegister(BMP3_REG_PWR_CTRL, op_mode_reg_val);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    /* Set the power mode */
    op_mode_reg_val = BMP3_SET_BITS(op_mode_reg_val, BMP3_OP_MODE, this->settings.op_mode);
    return this->writeRegister(BMP3_REG_PWR_CTRL, op_mode_reg_val);
}

Drv::I2cStatus BMP390::validate_normal_mode_settings() {
    Drv::I2cStatus stat = this->get_odr_filter_settings();
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    this->validate_osr_and_odr_settings();

    return stat;
}

Drv::I2cStatus BMP390::get_odr_filter_settings() {
    U8 reg_data[4] = {0};
    Fw::Buffer buffer(reg_data, 4);
    Drv::I2cStatus stat = this->readRegisterBlock(BMP3_REG_OSR, buffer);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    this->settings.odr_filter.press_os = BMP3_GET_BITS_POS_0(buffer.getData()[0], BMP3_PRESS_OS);
    this->settings.odr_filter.temp_os = BMP3_GET_BITS(buffer.getData()[0], BMP3_TEMP_OS);
    this->settings.odr_filter.odr = BMP3_GET_BITS_POS_0(buffer.getData()[1], BMP3_ODR);
    this->settings.odr_filter.iir_filter = BMP3_GET_BITS(buffer.getData()[3], BMP3_IIR_FILTER);

    return stat;
}

void BMP390::parse_calib_data(Fw::Buffer& buffer) {
    /* Temporary variable to store the aligned trim data */
    struct bmp3_reg_calib_data* reg_calib_data = &this->calib_data.reg_calib_data;
    struct bmp3_quantized_calib_data* quantized_calib_data = &this->calib_data.quantized_calib_data;

    /* Temporary variable */
    F64 temp_var;

    /* 1 / 2^8 */
    temp_var = 0.00390625f;
    reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(buffer.getData()[1], buffer.getData()[0]);
    quantized_calib_data->par_t1 = ((F64)reg_calib_data->par_t1 / temp_var);
    reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(buffer.getData()[3], buffer.getData()[2]);
    temp_var = 1073741824.0f;
    quantized_calib_data->par_t2 = ((F64)reg_calib_data->par_t2 / temp_var);
    reg_calib_data->par_t3 = (int8_t)buffer.getData()[4];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_t3 = ((F64)reg_calib_data->par_t3 / temp_var);
    reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(buffer.getData()[6], buffer.getData()[5]);
    temp_var = 1048576.0f;
    quantized_calib_data->par_p1 = ((F64)(reg_calib_data->par_p1 - (16384)) / temp_var);
    reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(buffer.getData()[8], buffer.getData()[7]);
    temp_var = 536870912.0f;
    quantized_calib_data->par_p2 = ((F64)(reg_calib_data->par_p2 - (16384)) / temp_var);
    reg_calib_data->par_p3 = (int8_t)buffer.getData()[9];
    temp_var = 4294967296.0f;
    quantized_calib_data->par_p3 = ((F64)reg_calib_data->par_p3 / temp_var);
    reg_calib_data->par_p4 = (int8_t)buffer.getData()[10];
    temp_var = 137438953472.0f;
    quantized_calib_data->par_p4 = ((F64)reg_calib_data->par_p4 / temp_var);
    reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(buffer.getData()[12], buffer.getData()[11]);

    /* 1 / 2^3 */
    temp_var = 0.125f;
    quantized_calib_data->par_p5 = ((F64)reg_calib_data->par_p5 / temp_var);
    reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(buffer.getData()[14], buffer.getData()[13]);
    temp_var = 64.0f;
    quantized_calib_data->par_p6 = ((F64)reg_calib_data->par_p6 / temp_var);
    reg_calib_data->par_p7 = (int8_t)buffer.getData()[15];
    temp_var = 256.0f;
    quantized_calib_data->par_p7 = ((F64)reg_calib_data->par_p7 / temp_var);
    reg_calib_data->par_p8 = (int8_t)buffer.getData()[16];
    temp_var = 32768.0f;
    quantized_calib_data->par_p8 = ((F64)reg_calib_data->par_p8 / temp_var);
    reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(buffer.getData()[18], buffer.getData()[17]);
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p9 = ((F64)reg_calib_data->par_p9 / temp_var);
    reg_calib_data->par_p10 = (int8_t)buffer.getData()[19];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p10 = ((F64)reg_calib_data->par_p10 / temp_var);
    reg_calib_data->par_p11 = (int8_t)buffer.getData()[20];
    temp_var = 36893488147419103232.0f;
    quantized_calib_data->par_p11 = ((F64)reg_calib_data->par_p11 / temp_var);
}

void BMP390::fill_osr_data(U32 settings, U8* addr, Fw::Buffer& reg_data, U8& len) {
    struct bmp3_odr_filter_settings osr_settings = this->settings.odr_filter;

    if (settings & (BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS)) {
        /* Pressure over sampling settings check */
        if (settings & BMP3_SEL_PRESS_OS) {
            /* Set the pressure over sampling settings in the register variable */
            reg_data.getData()[len] = BMP3_SET_BITS_POS_0(reg_data.getData()[0], BMP3_PRESS_OS, osr_settings.press_os);
        }

        /* Temperature over sampling settings check */
        if (settings & BMP3_SEL_TEMP_OS) {
            /* Set the temperature over sampling settings in the register variable */
            reg_data.getData()[len] = BMP3_SET_BITS(reg_data.getData()[0], BMP3_TEMP_OS, osr_settings.temp_os);
        }

        /* 0x1C is the register address of over sampling register */
        addr[len] = BMP3_REG_OSR;
        len++;
    }
}

void BMP390::fill_odr_data(U8* addr, Fw::Buffer& reg_data, U8& len) {
    struct bmp3_odr_filter_settings osr_settings = this->settings.odr_filter;

    /* Limit the ODR to 0.001525879 Hz*/
    if (osr_settings.odr > BMP3_ODR_0_001_HZ) {
        osr_settings.odr = BMP3_ODR_0_001_HZ;
    }

    /* Set the ODR settings in the register variable */
    reg_data.getData()[len] = BMP3_SET_BITS_POS_0(reg_data.getData()[1], BMP3_ODR, osr_settings.odr);

    /* 0x1D is the register address of output data rate register */
    addr[len] = BMP3_REG_ODR;
    len++;
}

void BMP390::fill_filter_data(U8* addr, Fw::Buffer& reg_data, U8& len) {
    struct bmp3_odr_filter_settings osr_settings = this->settings.odr_filter;

    /* Set the iir settings in the register variable */
    reg_data.getData()[len] = BMP3_SET_BITS(reg_data.getData()[3], BMP3_IIR_FILTER, osr_settings.iir_filter);

    /* 0x1F is the register address of iir filter register */
    addr[len] = BMP3_REG_CONFIG;
    len++;
}

void BMP390::validate_osr_and_odr_settings() {
    /* According to BMP388 datasheet at Section 3.9.2. "Measurement rate in
     * forced mode and normal mode" there is also the constant of 234us also to
     * be considered in the sum. */
    U32 meas_t = 234;
    U32 meas_t_p = 0;

    /* Sampling period corresponding to ODR in microseconds  */
    U32 odr[18] = {5000,    10000,   20000,    40000,    80000,    160000,   320000,    640000,    1280000,
                   2560000, 5120000, 10240000, 20480000, 40960000, 81920000, 163840000, 327680000, 655360000};

    if (this->settings.press_en) {
        /* Calculate the pressure measurement duration */
        struct bmp3_odr_filter_settings odr_filter = this->settings.odr_filter;

        F32 base = 2.0;
        F32 partial_out = pow(base, odr_filter.press_os);
        U32 press_meas = (BMP3_SETTLE_TIME_PRESS + partial_out * BMP3_ADC_CONV_TIME);
        meas_t_p += press_meas;
    }

    if (this->settings.temp_en) {
        /* Calculate the temperature measurement duration */
        struct bmp3_odr_filter_settings odr_filter = this->settings.odr_filter;

        F32 base = 2.0;
        F32 partial_out = pow(base, odr_filter.temp_os);
        U32 temp_meas = (BMP3_SETTLE_TIME_TEMP + partial_out * BMP3_ADC_CONV_TIME);
        meas_t_p += temp_meas;
    }

    /* Constant 234us added to the summation of temperature and pressure measurement duration */
    meas_t += meas_t_p;

    // Verify measurement time and ODR duration
    if (meas_t >= odr[this->settings.odr_filter.odr]) {
        FW_ASSERT(0);  // TODO
    }
}

F64 BMP390::compensate_temperature(const struct bmp3_uncomp_data* uncomp_data) {
    U32 uncomp_temp = uncomp_data->temperature;
    F64 partial_data1;
    F64 partial_data2;

    partial_data1 = (F64)(uncomp_temp - this->calib_data.quantized_calib_data.par_t1);
    partial_data2 = (F64)(partial_data1 * this->calib_data.quantized_calib_data.par_t2);

    /* Update the compensated temperature in calib structure since this is
     * needed for pressure calculation */
    this->calib_data.quantized_calib_data.t_lin =
        partial_data2 + (partial_data1 * partial_data1) * this->calib_data.quantized_calib_data.par_t3;

    /* Returns compensated temperature */
    return this->calib_data.quantized_calib_data.t_lin;
}

F64 BMP390::compensate_pressure(const struct bmp3_uncomp_data* uncomp_data) {
    const struct bmp3_quantized_calib_data* quantized_calib_data = &this->calib_data.quantized_calib_data;

    /* Variable to store the compensated pressure */
    F64 comp_press;

    /* Temporary variables used for compensation */
    F64 partial_data1;
    F64 partial_data2;
    F64 partial_data3;
    F64 partial_data4;
    F64 partial_out1;
    F64 partial_out2;

    partial_data1 = quantized_calib_data->par_p6 * quantized_calib_data->t_lin;
    partial_data2 = quantized_calib_data->par_p7 * pow(quantized_calib_data->t_lin, 2);
    partial_data3 = quantized_calib_data->par_p8 * pow(quantized_calib_data->t_lin, 3);
    partial_out1 = quantized_calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = quantized_calib_data->par_p2 * quantized_calib_data->t_lin;
    partial_data2 = quantized_calib_data->par_p3 * pow(quantized_calib_data->t_lin, 2);
    partial_data3 = quantized_calib_data->par_p4 * pow(quantized_calib_data->t_lin, 3);
    partial_out2 =
        uncomp_data->pressure * (quantized_calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = pow((F64)uncomp_data->pressure, 2);
    partial_data2 = quantized_calib_data->par_p9 + quantized_calib_data->par_p10 * quantized_calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + pow((F64)uncomp_data->pressure, 3) * quantized_calib_data->par_p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}

I8 BMP390::cal_crc(U8 seed, U8 data) {
    I8 poly = 0x1D;
    I8 var2;
    U8 i;

    for (i = 0; i < 8; i++) {
        if ((seed & 0x80) ^ (data & 0x80)) {
            var2 = 1;
        } else {
            var2 = 0;
        }

        seed = (seed & 0x7F) << 1;
        data = (data & 0x7F) << 1;
        seed = seed ^ (U8)(poly * var2);
    }

    return static_cast<I8>(seed);
}

}  // namespace Adafruit
