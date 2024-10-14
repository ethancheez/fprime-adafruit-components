// ======================================================================
// \title  MPL3115A2.cpp
// \author ethancheez
// \brief  cpp file for MPL3115A2 component implementation class
// ======================================================================

#include "Components/MPL3115A2/MPL3115A2.hpp"
#include "FpConfig.hpp"
#include "Fw/Logger/Logger.hpp"

#include "MPL3115A2_Registers.hpp"

namespace Sensors {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

MPL3115A2::MPL3115A2(const char* const compName)
    : MPL3115A2ComponentBase(compName),
      m_polyDb_offset(0),
      m_i2cDevAddress(0x60),
      m_seaLevelPressure(1019.1),
      m_altitude{0.0, "m"},
      m_temperature{0.0, "C"},
      m_pressure{0.0, "kPa"} {}

MPL3115A2::~MPL3115A2() {}

void MPL3115A2::config() {
    Drv::I2cStatus stat;

    U8 byte = 0;
    Fw::Buffer buf;

    // Verify Barometer exists
    byte = MPL3115A2_WHO_AM_I;
    buf.set(&byte, sizeof byte);
    stat = this->i2cWriteRead_out(0, this->m_i2cDevAddress, buf, buf);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

    FW_ASSERT(buf.getData()[0] == 0xC4, buf.getData()[0]);

    // Reset
    byte = MPL3115A2_RST_F;
    buf.set(&byte, sizeof byte);
    // Do not handle error. Device resets too quickly, causing I2C Write Error.
    (void)this->writeRegisterBlock(MPL3115A2_CTRL_REG1, buf);
    Os::Task::delay(100);  // Give time to reset

    // Wait for device to come back online
    U32 iter = 0;
    do {
        buf.set(&byte, sizeof byte);
        stat = this->readRegisterBlock(MPL3115A2_CTRL_REG1, buf);
        FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

        // Prevent infinite loop
        FW_ASSERT(iter < 1000000, iter);

        Os::Task::delay(10);
    } while (buf.getData()[0] & MPL3115A2_RST_F);

    // Set oversampling and altitude mode
    byte = (MPL3115A2_OS_F | MPL3115A2_ALT_F) + 1;
    buf.set(&byte, sizeof byte);
    stat = this->writeRegisterBlock(MPL3115A2_CTRL_REG1, buf);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

    // Enable data ready events for pressure/altitude and temperature
    byte = MPL3115A2_PT_DATA_CFG_TDEFE | MPL3115A2_PT_DATA_CFG_PDEFE | MPL3115A2_PT_DATA_CFG_DREM;
    buf.set(&byte, sizeof byte);
    stat = this->writeRegisterBlock(MPL3115A2_PT_DATA_CFG, buf);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

    // Set default sea level pressure
    this->setSeaLevelPressure(this->m_seaLevelPressure);
}

/**
 * \brief Sets the offset index to store telemetry into PolyDB
 *
 * \param offset: the offset index value
 */
void MPL3115A2::setPolyDbOffset(U32 offset) {
    this->m_polyDb_offset = offset;
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void MPL3115A2 ::SET_SEA_LEVEL_PRESSURE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F64 sea_level_pressure) {
    Drv::I2cStatus stat = this->setSeaLevelPressure(sea_level_pressure);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

void MPL3115A2 ::run_handler(NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context) {
    F64 altitude;
    F64 temperature;
    if (this->getAltitude(&altitude) == Drv::I2cStatus::I2C_OK) {
        this->m_altitude.setvalue(altitude);
        this->tlmWrite_Altitude(this->m_altitude);
    }
    if (this->getTemperature(&temperature) == Drv::I2cStatus::I2C_OK) {
        this->m_temperature.setvalue(temperature);
        this->tlmWrite_TemperatureC(this->m_temperature);
    }

    if (this->isConnected_setPolyDb_OutputPort(0)) {
        Fw::PolyType vals[] = {
            this->m_altitude.getvalue(),
            this->m_temperature.getvalue(),
        };
        Svc::MeasurementStatus mstat = Svc::MeasurementStatus::OK;
        Fw::Time ts(TB_NONE, 0, 0);

        for (U32 entry = 0; entry < 2; entry++) {
            this->setPolyDb_out(0, entry + m_polyDb_offset, mstat, ts, vals[entry]);

            if (mstat != Svc::MeasurementStatus::OK) {
                this->log_WARNING_HI_PolyDbSetError(mstat);
            }
        }
    }
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

/**
 * \brief Sets up the barometer to know what register the next read should be from
 *
 * The barometer requires a write call with a register's address before a read will function correctly. This helper
 * sets up that read address by writing it to the barometer via the I2C write port.
 *
 * \param reg: barometer internal address to the first register to be read
 * \return: I2C status from the write call
 */
Drv::I2cStatus MPL3115A2::setupReadRegister(U8 reg) {
    Fw::Buffer buffer(&reg, sizeof reg);
    return this->i2cWrite_out(0, this->m_i2cDevAddress, buffer);
}

/**
 * \brief Writes a block of data to the specified register of the barometer
 *
 * The data to be written to the I2C port is constructed by appending the buffer bytes to the `startRegister`
 * byte. It then calls the write port of the I2C bus to write data to the barometer, where the first byte of that
 * buffer will indicate which register to write to.
 *
 * \param startRegister: register address to write to
 * \param buffer: buffer to write
 * \return: I2C status of transactions
 */
Drv::I2cStatus MPL3115A2::writeRegisterBlock(U8 startRegister, Fw::Buffer& buffer) {
    U8 buf[sizeof startRegister + buffer.getSize()];
    buf[0] = startRegister;
    memcpy(&buf[1], buffer.getData(), buffer.getSize());
    Fw::Buffer reg_data(buf, sizeof buf);
    return this->i2cWrite_out(0, this->m_i2cDevAddress, reg_data);
}

/**
 * \brief Reads a block of data from the specified register of the barometer
 *
 * This function starts by writing the startRegister to the barometer by passing it to `setupReadRegister`. It then
 * calls the read port of the I2C bus to read data from the barometer. It will read `buffer.getSize()` bytes from the
 * I2C device and as such the caller must set this up.
 *
 * \param startRegister: register address to start reading from
 * \param buffer: buffer to read into. Determines size of read.
 * \return: I2C status of transactions
 */
Drv::I2cStatus MPL3115A2::readRegisterBlock(U8 startRegister, Fw::Buffer& buffer) {
    Drv::I2cStatus status;
    status = this->setupReadRegister(startRegister);
    if (status == Drv::I2cStatus::I2C_OK) {
        status = this->i2cRead_out(0, m_i2cDevAddress, buffer);
    }
    return status;
}

Drv::I2cStatus MPL3115A2::pollCtrlReg1Mask(U8 mask, U8* regValue) {
    Drv::I2cStatus stat;
    U8 byte;
    Fw::Buffer buf;

    U32 iter = 0;
    do {
        buf.set(&byte, sizeof byte);
        stat = this->readRegisterBlock(MPL3115A2_CTRL_REG1, buf);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }

        if (iter > 1000) {
            return Drv::I2cStatus::I2C_OTHER_ERR;
        }

        Os::Task::delay(10);
    } while (buf.getData()[0] & mask > 0);

    *regValue = buf.getData()[0];
    return stat;
}

Drv::I2cStatus MPL3115A2::pollStatus(U8 mask) {
    Drv::I2cStatus stat;
    U8 byte;
    Fw::Buffer buf;

    U32 iter = 0;
    do {
        buf.set(&byte, sizeof byte);
        stat = this->readRegisterBlock(MPL3115A2_STATUS, buf);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }

        if (iter > 1000) {
            return Drv::I2cStatus::I2C_OTHER_ERR;
        }

        Os::Task::delay(10);
    } while (buf.getData()[0] & mask == 0);

    return stat;
}

Drv::I2cStatus MPL3115A2::setSeaLevelPressure(F64 pressure) {
    U16 bar = static_cast<U16>(pressure * 50);
    U8 bytes[] = {static_cast<U8>((bar >> 8) & 0xFF), static_cast<U8>(bar & 0xFF)};
    Fw::Buffer buf(bytes, sizeof bytes);

    Drv::I2cStatus stat = this->writeRegisterBlock(MPL3115A2_BAR_IN_MSB, buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    this->m_seaLevelPressure = pressure;
    return stat;
}

Drv::I2cStatus MPL3115A2::getPressure(F64* pressure) {
    Drv::I2cStatus stat;
    U8 byte;
    Fw::Buffer buf;

    // Poll for a measurement to be finished
    U8 regValue;
    stat = this->pollCtrlReg1Mask(MPL3115A2_OST_F, &regValue);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    regValue = MPL3115A2_OS_F + 1;

    // Set control bits for pressure reading.
    buf.set(&regValue, sizeof regValue);
    stat = this->writeRegisterBlock(MPL3115A2_CTRL_REG1, buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    // Poll status for PDR to be set = press conversion complete
    stat = this->pollStatus(MPL3115A2_PDR_F);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    U8 data[6] = {0};
    buf.set(data, sizeof data);
    stat = this->readRegisterBlock(0x00, buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    I32 pressure_i32 = ((buf.getData()[1] * 65536) + (buf.getData()[2] * 256 + (buf.getData()[3] & 0xF0))) / 16;
    *pressure = static_cast<F64>((pressure_i32 / 4.0) / 1000.0);

    return stat;
}

Drv::I2cStatus MPL3115A2::getAltitude(F64* altitude) {
    Drv::I2cStatus stat;
    U8 byte;
    Fw::Buffer buf;

    // Poll for a measurement to be finished
    U8 regValue;
    stat = this->pollCtrlReg1Mask(MPL3115A2_OST_F, &regValue);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    regValue = (MPL3115A2_OS_F | MPL3115A2_ALT_F) + 1;

    // Set control bits for pressure/altitude reading.
    buf.set(&regValue, sizeof regValue);
    stat = this->writeRegisterBlock(MPL3115A2_CTRL_REG1, buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    // Poll status for PDR to be set = press conversion complete
    stat = this->pollStatus(MPL3115A2_PDR_F);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    U8 data[6] = {0};
    buf.set(data, sizeof data);
    stat = this->readRegisterBlock(0x00, buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    I32 altitude_i32 = ((buf.getData()[1] * 65536) + (buf.getData()[2] * 256 + (buf.getData()[3] & 0xF0)) / 16);
    *altitude = static_cast<F64>(altitude_i32) / 16.0;

    return stat;
}

Drv::I2cStatus MPL3115A2::getTemperature(F64* temperature) {
    Drv::I2cStatus stat;
    U8 byte;
    Fw::Buffer buf;

    // Poll for a measurement to be finished
    U8 regValue;
    stat = this->pollCtrlReg1Mask(MPL3115A2_OST_F, &regValue);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    regValue = (MPL3115A2_OS_F | MPL3115A2_ALT_F) + 1;

    // Initatiate a one-shot measurement
    buf.set(&regValue, sizeof regValue);
    stat = this->writeRegisterBlock(MPL3115A2_CTRL_REG1, buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    // Poll status for TDR to be set = press conversion complete
    stat = this->pollStatus(MPL3115A2_TDR_F);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    U8 data[6] = {0};
    buf.set(data, sizeof data);
    stat = this->readRegisterBlock(0x00, buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return stat;
    }

    I16 temperature_i16 = ((buf.getData()[4] * 256) + (buf.getData()[5] & 0xF0)) / 16;
    *temperature = static_cast<F64>(temperature_i16) / 16.0;

    return stat;
}

}  // namespace Sensors
