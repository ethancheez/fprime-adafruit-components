// ======================================================================
// \title  PCA9685.cpp
// \author ethancheez
// \brief  cpp file for PCA9685 component implementation class
// ======================================================================

#include "Components/PCA9685/PCA9685.hpp"
#include "FpConfig.hpp"
#include "Fw/Logger/Logger.hpp"

namespace Sensors {

/**
 * \brief Construct PCA9685 object
 */
PCA9685 ::PCA9685(const char* const compName)
    : PCA9685ComponentBase(compName),
      m_polyDb_offset(0),
      m_i2cDevAddress(0x40),
      m_channels(16),
      m_referenceClockSpeed(25000000),
      m_frequency(50),
      m_minPWM(100),
      m_maxPWM(500),
      m_defaultAngle(0) {}

/**
 * \brief Destroy PCA9685 object
 */
PCA9685 ::~PCA9685() {}

/**
 * \brief Configure the PCA9685 Servo Driver
 *
 * Resets the device and sets the PWM frequency
 */
void PCA9685::config(U8 numChannels) {
    Drv::I2cStatus stat;

    // Reset
    stat = this->sendByteCommand(REG_MODE_REG1, 0x00);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
    stat = this->sendByteCommand(REG_MODE_REG2, 0x04);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

    // Set PWM Frequency
    U8 prescale = static_cast<U8>((this->m_referenceClockSpeed / 4096 / this->m_frequency + 0.5) - 1);
    stat = this->sendByteCommand(this->REG_MODE_REG1, (0x00 & 0x7F) | 0x10);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
    stat = this->sendByteCommand(this->REG_PRESCALE, prescale);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
    stat = this->sendByteCommand(this->REG_MODE_REG1, 0x00);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
    stat = this->sendByteCommand(this->REG_MODE_REG2, 0x04);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

    // Reset all channels to default angle
    for (U8 channel = 0; channel < 16; channel++) {
        Drv::I2cStatus stat = this->setPWM(channel, this->angleToPWM(this->m_defaultAngle));
        FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
    }

    this->m_channels = numChannels;
}

/**
 * \brief Sets the offset index to store telemetry into PolyDB
 *
 * \param offset: the offset index value
 */
void PCA9685::setPolyDbOffset(U32 offset) {
    this->m_polyDb_offset = offset;
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

//! Handler implementation for run
void PCA9685 ::run_handler(NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context) {
    Drv::I2cStatus stat;

    for (U8 channel = 0; channel < 4; channel++) {
        U16 pwm;
        stat = this->getPWM(channel, &pwm);
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_I2cError(stat);
            return;
        }
        I16 angle = (180.0 * (pwm - this->m_minPWM)) / (this->m_maxPWM - this->m_minPWM) - this->m_offsets[channel];
        if (angle < 0) {
            angle = 0;
        } else if (angle > 180) {
            angle = 180;
        }

        this->m_angles[channel] = static_cast<U8>(angle);
    }

    if (this->isConnected_setPolyDb_OutputPort(0)) {
        Svc::MeasurementStatus mstat = Svc::MeasurementStatus::OK;
        Fw::Time ts(TB_NONE, 0, 0);
        for (U32 i = 0; i < this->m_channels; i++) {
            Fw::PolyType val = this->m_angles[i];
            this->setPolyDb_out(0, i + m_polyDb_offset, mstat, ts, val);

            if (mstat != Svc::MeasurementStatus::OK) {
                this->log_WARNING_HI_PolyDbSetError(mstat);
            }
        }
    }

    if (this->isConnected_tlmOut_OutputPort(0)) {
        this->tlmWrite_Servo_Angles(this->m_angles);
    }
}

//! Handler implementation for setAngle
void PCA9685::setAngle_handler(NATIVE_INT_TYPE portNum, U8 channel, U8 angle) {
    if (channel > 16 || angle > 180) {
        this->log_WARNING_HI_SetAngleInvalidRange(angle);
        return;
    }

    Drv::I2cStatus stat = this->setAngle(channel, angle);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return;
    }
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

//! Handler implementation for command CALIBRATE
void PCA9685 ::CALIBRATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    Drv::I2cStatus stat;

    for (U32 channel = 0; channel < this->m_channels; channel++) {
        stat = this->setAngle(channel, 45);
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_I2cError(stat);
            this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
            return;
        }
    }
    Os::Task::delay(1000);
    for (U32 channel = 0; channel < this->m_channels; channel++) {
        stat = this->setAngle(channel, 90);
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_I2cError(stat);
            this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
            return;
        }
    }
    Os::Task::delay(1000);
    for (U32 channel = 0; channel < this->m_channels; channel++) {
        stat = this->setAngle(channel, 135);
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_I2cError(stat);
            this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
            return;
        }
    }
    Os::Task::delay(1000);

    for (U32 angle = 45; angle <= 135; angle++) {
        for (U32 channel = 0; channel < this->m_channels; channel++) {
            stat = this->setAngle(channel, angle);
            if (stat != Drv::I2cStatus::I2C_OK) {
                this->log_WARNING_HI_I2cError(stat);
                this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
                return;
            }
        }
        Os::Task::delay(10);
    }
    Os::Task::delay(1000);
    for (U32 angle = 135; angle >= 45; angle--) {
        for (U32 channel = 0; channel < this->m_channels; channel++) {
            stat = this->setAngle(channel, angle);
            if (stat != Drv::I2cStatus::I2C_OK) {
                this->log_WARNING_HI_I2cError(stat);
                this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
                return;
            }
        }
        Os::Task::delay(10);
    }
    Os::Task::delay(1000);

    for (U32 channel = 0; channel < this->m_channels; channel++) {
        stat = this->setAngle(channel, this->m_defaultAngle);
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_I2cError(stat);
            this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
            return;
        }
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

//! Handler implementation for command SET_ANGLE_OFFSET
void PCA9685 ::SET_ANGLE_OFFSET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 channel, I8 angle_offset) {
    // Force offset boundries to be +/- 20 degrees
    if (channel > 16 || angle_offset > 20 || angle_offset < -20) {
        this->log_WARNING_HI_SetAngleOffsetInvalidRange(angle_offset);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    I16 trueAngle = this->m_angles[channel] + angle_offset;
    if (trueAngle < 0) {
        trueAngle = 0;
    } else if (trueAngle > 180) {
        trueAngle = 180;
    }
    Drv::I2cStatus stat = this->setPWM(channel, this->angleToPWM(trueAngle));
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->m_offsets[channel] = angle_offset;

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

//! Handler implementation for command SET_ANGLE
void PCA9685 ::SET_ANGLE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 channel, U8 angle) {
    if (channel > 16 || angle > 180) {
        this->log_WARNING_HI_SetAngleInvalidRange(angle);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    Drv::I2cStatus stat = this->setAngle(channel, angle);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

//! Handler implementation for command SET_PWM_RAW
void PCA9685 ::SET_PWM_RAW_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 channel, U16 pwm) {
    if (channel > 16) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    Drv::I2cStatus stat = this->setPWM(channel, pwm);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

//! Handler implementation for command SET_PWM_RANGE
void PCA9685 ::SET_PWM_RANGE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 channel, U16 min_pwm, U16 max_pwm) {
    if (channel > 16) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->m_minPWM = min_pwm;
    this->m_maxPWM = max_pwm;

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

//! Handler implementation for command RESET_ALL_ANGLES
void PCA9685 ::RESET_ALL_ANGLES_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    for (U8 channel = 0; channel < 16; channel++) {
        Drv::I2cStatus stat = this->setAngle(channel, this->m_defaultAngle);
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_I2cError(stat);
            this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
            return;
        }
    }
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

//! Handler implementation for command RESET_ALL_ANGLE_OFFSETS
void PCA9685 ::RESET_ALL_ANGLE_OFFSETS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    for (U8 channel = 0; channel < 16; channel++) {
        Drv::I2cStatus stat = this->setPWM(channel, this->angleToPWM(this->m_angles[channel]));
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_I2cError(stat);
            this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
            return;
        }

        this->m_offsets[channel] = 0;
    }
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

/**
 * \brief Sets the angle to the specified channel
 *
 * \param channel: the PWM channel to set (0-15)
 * \param angle: the angle in degrees (0-180)
 * \return: I2C status of transactions
 */
Drv::I2cStatus PCA9685::setAngle(U8 channel, U8 angle) {
    if (channel > 16 || angle > 180) {
        return Drv::I2cStatus::I2C_OTHER_ERR;
    }

    I16 trueAngle = angle + this->m_offsets[channel];
    if (trueAngle < 0) {
        trueAngle = 0;
    } else if (trueAngle > 180) {
        trueAngle = 180;
    }
    Drv::I2cStatus stat = this->setPWM(channel, this->angleToPWM(static_cast<U8>(trueAngle)));
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief Converts an angle (in degrees) to a PWM value
 *
 * PWM value is dependent on the minimum and maximum PWM values.
 *
 * \param angle: the angle in degrees (0-180)
 * \return: the PWM value
 */
U16 PCA9685::angleToPWM(U8 angle) {
    return static_cast<U16>((F64)angle / 180.0 * (F64)(this->m_maxPWM - this->m_minPWM) + this->m_minPWM);
}

/**
 * \brief Sets the PWM value to the specified channel
 *
 * \param channel: the PWM channel (0-15)
 * \param pwm: the PWM value
 * \return: I2C status of transactions
 */
Drv::I2cStatus PCA9685::setPWM(U8 channel, U16 pwm) {
    Drv::I2cStatus stat;

    stat = this->sendByteCommand(REG_CHANNEL0_ON_L + CHANNEL_MULTIPLYER * channel, 0 & 0xFF);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    stat = this->sendByteCommand(REG_CHANNEL0_ON_H + CHANNEL_MULTIPLYER * channel, 0 >> 8);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    stat = this->sendByteCommand(REG_CHANNEL0_OFF_L + CHANNEL_MULTIPLYER * channel, pwm & 0xFF);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    return this->sendByteCommand(REG_CHANNEL0_OFF_H + CHANNEL_MULTIPLYER * channel, pwm >> 8);
}

/**
 * \brief Gets the PWM value of the specified channel
 *
 * \param channel: the PWM channel (0-15)
 * \return: the PWM value
 */
Drv::I2cStatus PCA9685::getPWM(U8 channel, U16* pwm) {
    Drv::I2cStatus stat;

    U8 offL;
    U8 offH;
    stat = this->readRegisterByte(REG_CHANNEL0_OFF_L + CHANNEL_MULTIPLYER * channel, &offL);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    stat = this->readRegisterByte(REG_CHANNEL0_OFF_H + CHANNEL_MULTIPLYER * channel, &offH);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    *pwm = (offH << 8) | offL;

    return stat;
}

/**
 * \brief Sets up the device to know what register the next read should be from
 *
 * The device requires a write call with a register's address before a read will function correctly. This helper
 * sets up that read address by writing it to the device via the I2C write port.
 *
 * \param reg: device internal address to the first register to be read
 * \return: I2C status from the write call
 */
Drv::I2cStatus PCA9685::setupReadRegister(U8 reg) {
    Fw::Buffer buffer(&reg, sizeof reg);
    return this->i2cWrite_out(0, this->m_i2cDevAddress, buffer);
}

/**
 * \brief Writes a byte of data to the specified register of the device
 *
 * The data to be written to the I2C port is constructed by appending the buffer bytes to the `startRegister`
 * byte. It then calls the write port of the I2C bus to write data to the device, where the first
 * byte of that buffer will indicate which register to write to.
 *
 * \param startRegister: register address to write to
 * \param byte: byte to write
 * \return: I2C status of transactions
 */
Drv::I2cStatus PCA9685::sendByteCommand(U8 startRegister, U8 byte) {
    U8 buf[2];
    buf[0] = startRegister;
    buf[1] = byte;
    Fw::Buffer reg_data(buf, sizeof buf);
    return this->i2cWrite_out(0, this->m_i2cDevAddress, reg_data);
}

/**
 * \brief Reads a byte of data from the specified register of the device
 *
 * This function starts by writing the startRegister to the device by passing it to `setupReadRegister`. It then
 * calls the read port of the I2C bus to read data from the device.
 *
 * \param startRegister: register address to start reading from
 * \param buffer: byte to read into
 * \return: I2C status of transactions
 */
Drv::I2cStatus PCA9685::readRegisterByte(U8 startRegister, U8* byte) {
    U8 data;
    Fw::Buffer buffer(&data, sizeof data);

    Drv::I2cStatus status;
    status = this->setupReadRegister(startRegister);
    if (status == Drv::I2cStatus::I2C_OK) {
        status = this->i2cRead_out(0, this->m_i2cDevAddress, buffer);
        *byte = buffer.getData()[0];
    }
    return status;
}

}  // namespace Sensors
