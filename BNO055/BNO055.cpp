// ======================================================================
// \title  BNO055.cpp
// \author ethancheez
// \brief  cpp file for BNO055 component implementation class
// ======================================================================

#include "Components/BNO055/BNO055.hpp"
#include "FpConfig.hpp"
#include "Fw/Logger/Logger.hpp"

namespace Sensors {

/**
 * \brief Construct BNO055 object
 */
BNO055::BNO055(const char* const compName)
    : BNO055ComponentBase(compName),
      m_polyDb_offset(0),
      m_i2cDevAddress(0x28),
      m_accel({0, "m/s^2"}, {0, "m/s^2"}, {0, "m/s^2"}),
      m_gyro({0, "deg/s"}, {0, "deg/s"}, {0, "deg/s"}),
      m_mag({0, "uT"}, {0, "uT"}, {0, "uT"}),
      m_euler({0, "degrees"}, {0, "degrees"}, {0, "degrees"}),
      m_quaternion({0, ""}, {0, ""}, {0, ""}, {0, ""}),
      m_linear_accel({0, "m/s^2"}, {0, "m/s^2"}, {0, "m/s^2"}),
      m_gravity({0, "m/s^2"}, {0, "m/s^2"}, {0, "m/s^2"}) {}

/**
 * \brief Destroy BNO055 object
 */
BNO055::~BNO055() {}

/**
 * \brief Configure the IMU
 *
 * Sets the default page ID of the IMU's page ID register to 0x00, configures the power mode to `Normal`,
 * and finally sets the operation mode to `NDOF` to begin reading IMU data.
 */
void BNO055::config() {
    Drv::I2cStatus stat;

    Fw::Buffer reg_data;

    U8 data_u8 = BNO055_INIT_VALUE;
    U8 bno055_page_zero_u8 = BNO055_PAGE_ZERO;

    /* Write the default page as zero*/
    reg_data.set(&bno055_page_zero_u8, sizeof bno055_page_zero_u8);
    stat = this->writeRegisterBlock(BNO055_PAGE_ID_REG, reg_data);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

    /* Read the page id from the register 0x07 */
    reg_data.set(&data_u8, sizeof data_u8);
    stat = this->readRegisterBlock(BNO055_PAGE_ID_REG, reg_data);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
    this->m_page_id = reg_data.getData()[0];

    /* Set power mode */
    stat = this->setPowerMode(BNO055_POWER_MODE_NORMAL);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);

    /* Set opmode to read sensor data */
    stat = this->setOperationMode(BNO055_OPERATION_MODE_NDOF);
    FW_ASSERT(stat == Drv::I2cStatus::I2C_OK, stat);
}

/**
 * \brief Sets the offset index to store telemetry into PolyDB
 *
 * \param offset: the offset index value
 */
void BNO055::setPolyDbOffset(U32 offset) {
    this->m_polyDb_offset = offset;
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

void BNO055::run_handler(NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context) {
    this->getCalibrations();
    this->updateAccel();
    this->updateGyro();
    this->updateMag();
    this->updateEuler();
    this->updateQuaternion();
    this->updateLinearAccel();
    this->updateGravity();

    if (this->isConnected_setPolyDb_OutputPort(0)) {
        Fw::PolyType vals[] = {
            this->m_accel.getx().getvalue(),        this->m_accel.gety().getvalue(),
            this->m_accel.getz().getvalue(),        this->m_gyro.getx().getvalue(),
            this->m_gyro.gety().getvalue(),         this->m_gyro.getz().getvalue(),
            this->m_mag.getx().getvalue(),          this->m_mag.gety().getvalue(),
            this->m_mag.getz().getvalue(),          this->m_euler.getx().getvalue(),
            this->m_euler.gety().getvalue(),        this->m_euler.getz().getvalue(),
            this->m_quaternion.getw().getvalue(),   this->m_quaternion.getx().getvalue(),
            this->m_quaternion.gety().getvalue(),   this->m_quaternion.getz().getvalue(),
            this->m_linear_accel.getx().getvalue(), this->m_linear_accel.gety().getvalue(),
            this->m_linear_accel.getz().getvalue(), this->m_gravity.getx().getvalue(),
            this->m_gravity.gety().getvalue(),      this->m_gravity.getz().getvalue(),
        };
        Svc::MeasurementStatus mstat = Svc::MeasurementStatus::OK;
        Fw::Time ts(TB_NONE, 0, 0);

        for (U32 entry = 0; entry < 22; entry++) {
            this->setPolyDb_out(0, entry + m_polyDb_offset, mstat, ts, vals[entry]);

            if (mstat != Svc::MeasurementStatus::OK) {
                this->log_WARNING_HI_PolyDbSetError(mstat);
            }
        }
    }

    if (this->isConnected_tlmOut_OutputPort(0)) {
        this->tlmWrite_CalibrationStatus(this->m_calibrations);
        this->tlmWrite_Accel(this->m_accel);
        this->tlmWrite_Gyro(this->m_gyro);
        this->tlmWrite_Mag(this->m_mag);
        this->tlmWrite_Euler(this->m_euler);
        this->tlmWrite_Quat(this->m_quaternion);
        this->tlmWrite_LinearAccel(this->m_linear_accel);
        this->tlmWrite_Gravity(this->m_gravity);
    }
}

/**
 * \brief Sets up the IMU to know what register the next read should be from
 *
 * The IMU requires a write call with a register's address before a read will function correctly. This helper
 * sets up that read address by writing it to the IMU via the I2C write port.
 *
 * \param reg: IMU internal address to the first register to be read
 * \return: I2C status from the write call
 */
Drv::I2cStatus BNO055::setupReadRegister(U8 reg) {
    Fw::Buffer buffer(&reg, sizeof reg);
    return this->i2cWrite_out(0, this->m_i2cDevAddress, buffer);
}

/**
 * \brief Writes a block of data to the specified register of the IMU
 *
 * The data to be written to the I2C port is constructed by appending the buffer bytes to the `startRegister`
 * byte. It then calls the write port of the I2C bus to write data to the IMU, where the first byte of that
 * buffer will indicate which register to write to.
 *
 * \param startRegister: register address to write to
 * \param buffer: buffer to write
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::writeRegisterBlock(U8 startRegister, Fw::Buffer& buffer) {
    U8 buf[sizeof startRegister + buffer.getSize()];
    buf[0] = startRegister;
    memcpy(&buf[1], buffer.getData(), buffer.getSize());
    Fw::Buffer reg_data(buf, sizeof buf);
    return this->i2cWrite_out(0, this->m_i2cDevAddress, reg_data);
}

/**
 * \brief Reads a block of data from the specified register of the IMU
 *
 * This function starts by writing the startRegister to the IMU by passing it to `setupReadRegister`. It then calls
 * the read port of the I2C bus to read data from the IMU. It will read `buffer.getSize()` bytes from the I2C device
 * and as such the caller must set this up.
 *
 * \param startRegister: register address to start reading from
 * \param buffer: buffer to read into. Determines size of read.
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::readRegisterBlock(U8 startRegister, Fw::Buffer& buffer) {
    Drv::I2cStatus status;
    status = this->setupReadRegister(startRegister);
    if (status == Drv::I2cStatus::I2C_OK) {
        status = this->i2cRead_out(0, m_i2cDevAddress, buffer);
    }
    return status;
}

/**
 * \brief This API used to write the page id register 0x07
 *
 * This function starts by writing the startRegister to the IMU by passing it to `setupReadRegister`. It then calls
 * the read port of the I2C bus to read data from the IMU. It will read `buffer.getSize()` bytes from the I2C device
 * and as such the caller must set this up.
 *
 * \param pageId: the value of page id
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::writePageId(U8 pageId) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_PAGE_ID_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    data_u8r = BNO055_SET_BITSLICE(reg_data.getData()[0], BNO055_PAGE_ID, pageId);

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->writeRegisterBlock(BNO055_PAGE_ID_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    /* Read the page id from the register 0x07 */
    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_PAGE_ID_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    this->m_page_id = reg_data.getData()[0];

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to read the operation mode from register from 0x3D bit 0 to 3
 *
 * operation_mode |              result               | comments
 * ---------------|-----------------------------------|----------------------------
 *  0x00          | BNO055_OPERATION_MODE_CONFIG      | Configuration mode
 *  0x01          | BNO055_OPERATION_MODE_ACCONLY     | Reads accel data alone
 *  0x02          | BNO055_OPERATION_MODE_MAGONLY     | Reads mag data alone
 *  0x03          | BNO055_OPERATION_MODE_GYRONLY     | Reads gyro data alone
 *  0x04          | BNO055_OPERATION_MODE_ACCMAG      | Reads accel and mag data
 *  0x05          | BNO055_OPERATION_MODE_ACCGYRO     | Reads accel and gyro data
 *  0x06          | BNO055_OPERATION_MODE_MAGGYRO     | Reads accel and mag data
 *  0x07          | OPERATION_MODE_ANY_MOTION         | Reads accel mag and gyro data
 *  0x08          | BNO055_OPERATION_MODE_IMUPLUS     | Inertial measurement unit
 *   -            |       -                           | Reads accel,gyro and fusion data
 *  0x09          | BNO055_OPERATION_MODE_COMPASS     | Reads accel, mag data
 *   -            |       -                           | and fusion data
 *  0x0A          | BNO055_OPERATION_MODE_M4G         | Reads accel, mag data
 *    -           |       -                           | and fusion data
 *  0x0B          | BNO055_OPERATION_MODE_NDOF_FMC_OFF| Nine degrees of freedom with
 *   -            |       -                           | fast magnetic calibration
 *   -            |       -                           | Reads accel,mag, gyro
 *   -            |       -                           | and fusion data
 *  0x0C          | BNO055_OPERATION_MODE_NDOF        | Nine degrees of freedom
 *   -            |       -                           | Reads accel,mag, gyro
 *   -            |       -                           | and fusion data
 *
 * \param operation_mode: pointer to store operation mode value
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::getOperationMode(U8* operation_mode) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;

    if (this->m_page_id != BNO055_PAGE_ZERO) {
        /* Write the page zero*/
        stat = this->writePageId(BNO055_PAGE_ZERO);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
    }

    if (this->m_page_id == BNO055_PAGE_ZERO) {
        Fw::Buffer reg_data(&data_u8r, sizeof data_u8r);
        stat = this->readRegisterBlock(BNO055_OPERATION_MODE_REG, reg_data);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
        *operation_mode = BNO055_GET_BITSLICE(reg_data.getData()[0], BNO055_OPERATION_MODE);
    }

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to write the operation mode from register from 0x3D bit 0 to 3
 *
 * operation_mode |              result               | comments
 * ---------------|-----------------------------------|----------------------------
 *  0x00          | BNO055_OPERATION_MODE_CONFIG      | Configuration mode
 *  0x01          | BNO055_OPERATION_MODE_ACCONLY     | Reads accel data alone
 *  0x02          | BNO055_OPERATION_MODE_MAGONLY     | Reads mag data alone
 *  0x03          | BNO055_OPERATION_MODE_GYRONLY     | Reads gyro data alone
 *  0x04          | BNO055_OPERATION_MODE_ACCMAG      | Reads accel and mag data
 *  0x05          | BNO055_OPERATION_MODE_ACCGYRO     | Reads accel and gyro data
 *  0x06          | BNO055_OPERATION_MODE_MAGGYRO     | Reads accel and mag data
 *  0x07          | OPERATION_MODE_ANY_MOTION         | Reads accel mag and gyro data
 *  0x08          | BNO055_OPERATION_MODE_IMUPLUS     | Inertial measurement unit
 *   -            |       -                           | Reads accel,gyro and fusion data
 *  0x09          | BNO055_OPERATION_MODE_COMPASS     | Reads accel, mag data
 *   -            |       -                           | and fusion data
 *  0x0A          | BNO055_OPERATION_MODE_M4G         | Reads accel, mag data
 *    -           |       -                           | and fusion data
 *  0x0B          | BNO055_OPERATION_MODE_NDOF_FMC_OFF| Nine degrees of freedom with
 *   -            |       -                           | fast magnetic calibration
 *   -            |       -                           | Reads accel,mag, gyro
 *   -            |       -                           | and fusion data
 *  0x0C          | BNO055_OPERATION_MODE_NDOF        | Nine degrees of freedom
 *   -            |       -                           | Reads accel,mag, gyro
 *   -            |       -                           | and fusion data
 *
 * \param operation_mode: the value of operation mode
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::setOperationMode(U8 operation_mode) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;
    U8 prev_opmode = BNO055_OPERATION_MODE_CONFIG;

    stat = this->getOperationMode(&prev_opmode);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }

    /* If the previous operation mode is config it is directly write the operation mode */
    if (prev_opmode == BNO055_OPERATION_MODE_CONFIG) {
        reg_data.set(&data_u8r, sizeof data_u8r);
        stat = this->readRegisterBlock(BNO055_OPERATION_MODE_REG, reg_data);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
        data_u8r = BNO055_SET_BITSLICE(reg_data.getData()[0], BNO055_OPERATION_MODE, operation_mode);
        reg_data.set(&data_u8r, sizeof data_u8r);
        stat = this->writeRegisterBlock(BNO055_OPERATION_MODE_REG, reg_data);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }

        /* Config mode to other operation mode switching required delay of 600ms*/
        Os::Task::delay(BNO055_MODE_SWITCHING_DELAY);
    }
    /* If the previous operation mode is not config it is write the config mode */
    else {
        reg_data.set(&data_u8r, sizeof data_u8r);
        stat = this->readRegisterBlock(BNO055_OPERATION_MODE_REG, reg_data);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
        data_u8r = BNO055_SET_BITSLICE(reg_data.getData()[0], BNO055_OPERATION_MODE, BNO055_OPERATION_MODE_CONFIG);
        reg_data.set(&data_u8r, sizeof data_u8r);
        stat = this->writeRegisterBlock(BNO055_OPERATION_MODE_REG, reg_data);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }

        /* Config mode to other operation mode switching required delay of 600ms*/
        Os::Task::delay(BNO055_MODE_SWITCHING_DELAY);

        /* Write the operation mode */
        if (operation_mode != BNO055_OPERATION_MODE_CONFIG) {
            reg_data.set(&data_u8r, sizeof data_u8r);
            stat = this->readRegisterBlock(BNO055_OPERATION_MODE_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
            data_u8r = BNO055_SET_BITSLICE(reg_data.getData()[0], BNO055_OPERATION_MODE, operation_mode);
            reg_data.set(&data_u8r, sizeof data_u8r);
            stat = this->writeRegisterBlock(BNO055_OPERATION_MODE_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }

            /* Config mode to other operation mode switching required delay of 600ms*/
            Os::Task::delay(BNO055_MODE_SWITCHING_DELAY);
        }
    }

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to write the power mode from register from 0x3E bit 0 to 1
 *
 *  power_mode  |           result           | comments
 * -------------|----------------------------|---------------------------------
 *  0x00        | BNO055_POWER_MODE_NORMAL   | In the NORMAL mode the register
 *    -         |       -                    | map and the internal peripherals
 *    -         |       -                    | of the MCU are always
 *    -         |       -                    | operative in this mode
 *  0x01        | BNO055_POWER_MODE_LOWPOWER | This is first level of power
 *              |       -                    | saving mode
 *  0x02        | BNO055_POWER_MODE_SUSPEND  | In suspend mode the system is
 *    -         |       -                    | paused and all the sensors and
 *    -         |       -                    | the micro controller are
 *    -         |       -                    | put into sleep mode.
 *
 * \param operation_mode: the value of power mode
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::setPowerMode(U8 power_mode) {
    Drv::I2cStatus stat;
    U8 data_u8 = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    U8 prev_opmode = BNO055_OPERATION_MODE_CONFIG;
    if (this->getOperationMode(&prev_opmode) == Drv::I2cStatus::I2C_OK) {
        if (prev_opmode != BNO055_OPERATION_MODE_CONFIG) {
            stat = this->setOperationMode(BNO055_OPERATION_MODE_CONFIG);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }

            reg_data.set(&data_u8, sizeof data_u8);
            stat = this->readRegisterBlock(BNO055_POWER_MODE_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
            data_u8 = BNO055_SET_BITSLICE(reg_data.getData()[0], BNO055_POWER_MODE, power_mode);
            reg_data.set(&data_u8, sizeof data_u8);
            stat = this->writeRegisterBlock(BNO055_POWER_MODE_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
        }
    }
    if (prev_opmode != BNO055_OPERATION_MODE_CONFIG) {
        /* set the operation mode of previous operation mode*/
        stat = this->setOperationMode(prev_opmode);
        if (stat != Drv::I2cStatus::I2C_OK) {
            return stat;
        }
    }

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to read the accel unit from register from 0x3B bit 0
 *
 *    accel_unit_u8   |     result
 *   ---------------  | ---------------
 *        0x00        | BNO055_ACCEL_UNIT_MSQ
 *        0x01        | BNO055_ACCEL_UNIT_MG
 *
 * \param operation_mode: the value of accel unit
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::getAccelUnit(U8* accel_unit) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_ACCEL_UNIT_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    *accel_unit = BNO055_GET_BITSLICE(reg_data.getData()[0], BNO055_ACCEL_UNIT);

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to write the accel unit from register from 0x3B bit 0
 *
 *    accel_unit_u8   |     result
 *   ---------------  | ---------------
 *        0x00        | BNO055_ACCEL_UNIT_MSQ
 *        0x01        | BNO055_ACCEL_UNIT_MG
 *
 * \param operation_mode: the value of accel unit
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::setAccelUnit(U8 accel_unit) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    U8 prev_opmode = BNO055_OPERATION_MODE_CONFIG;
    if (this->getOperationMode(&prev_opmode) == Drv::I2cStatus::I2C_OK) {
        if (prev_opmode != BNO055_OPERATION_MODE_CONFIG) {
            stat = this->setOperationMode(BNO055_OPERATION_MODE_CONFIG);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }

            reg_data.set(&data_u8r, sizeof data_u8r);
            stat = this->readRegisterBlock(BNO055_ACCEL_UNIT_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
            data_u8r = BNO055_SET_BITSLICE(reg_data.getData()[0], BNO055_ACCEL_UNIT, accel_unit);
            reg_data.set(&data_u8r, sizeof data_u8r);
            stat = this->writeRegisterBlock(BNO055_ACCEL_UNIT_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
        }
        if (prev_opmode != BNO055_OPERATION_MODE_CONFIG) {
            /* set the operation mode of previous operation mode*/
            stat = this->setOperationMode(prev_opmode);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
        }
    }

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to read the gyro unit from register from 0x3B bit 1
 *
 *  gyro_unit_u8    |  result
 *  -------------   | -----------
 *    0x00          | BNO055_GYRO_UNIT_DPS
 *    0x01          | BNO055_GYRO_UNIT_RPS
 *
 * \param operation_mode: the value of gyro unit
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::getGyroUnit(U8* gyro_unit) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_GYRO_UNIT_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    *gyro_unit = BNO055_GET_BITSLICE(reg_data.getData()[0], BNO055_GYRO_UNIT);

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to write the gyro unit from register from 0x3B bit 1
 *
 *  gyro_unit_u8    |  result
 *  -------------   | -----------
 *    0x00          | BNO055_GYRO_UNIT_DPS
 *    0x01          | BNO055_GYRO_UNIT_RPS
 *
 * \param operation_mode: the value of gyro unit
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::setGyroUnit(U8 gyro_unit) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    U8 prev_opmode = BNO055_OPERATION_MODE_CONFIG;
    if (this->getOperationMode(&prev_opmode) == Drv::I2cStatus::I2C_OK) {
        if (prev_opmode != BNO055_OPERATION_MODE_CONFIG) {
            stat = this->setOperationMode(BNO055_OPERATION_MODE_CONFIG);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }

            reg_data.set(&data_u8r, sizeof data_u8r);
            stat = this->readRegisterBlock(BNO055_GYRO_UNIT_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
            data_u8r = BNO055_SET_BITSLICE(reg_data.getData()[0], BNO055_GYRO_UNIT, gyro_unit);
            reg_data.set(&data_u8r, sizeof data_u8r);
            stat = this->writeRegisterBlock(BNO055_GYRO_UNIT_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
        }
        if (prev_opmode != BNO055_OPERATION_MODE_CONFIG) {
            /* set the operation mode of previous operation mode*/
            stat = this->setOperationMode(prev_opmode);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
        }
    }

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to read the euler unit from register from 0x3B bit 2
 *
 *    euler_unit_u8   | result
 *   --------------   | -----------
 *      0x00          | BNO055_EULER_UNIT_DEG
 *      0x01          | BNO055_EULER_UNIT_RAD
 *
 * \param operation_mode: the value of euler unit
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::getEulerUnit(U8* euler_unit) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_EULER_UNIT_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    *euler_unit = BNO055_GET_BITSLICE(reg_data.getData()[0], BNO055_EULER_UNIT);

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief This API used to write the euler unit from register from 0x3B bit 2
 *
 *    euler_unit_u8   | result
 *   --------------   | -----------
 *      0x00          | BNO055_EULER_UNIT_DEG
 *      0x01          | BNO055_EULER_UNIT_RAD
 *
 * \param operation_mode: the value of euler unit
 * \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::setEulerUnit(U8 euler_unit) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    U8 prev_opmode = BNO055_OPERATION_MODE_CONFIG;
    if (this->getOperationMode(&prev_opmode) == Drv::I2cStatus::I2C_OK) {
        if (prev_opmode != BNO055_OPERATION_MODE_CONFIG) {
            stat = this->setOperationMode(BNO055_OPERATION_MODE_CONFIG);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }

            reg_data.set(&data_u8r, sizeof data_u8r);
            stat = this->readRegisterBlock(BNO055_EULER_UNIT_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
            data_u8r = BNO055_SET_BITSLICE(reg_data.getData()[0], BNO055_EULER_UNIT, euler_unit);
            reg_data.set(&data_u8r, sizeof data_u8r);
            stat = this->writeRegisterBlock(BNO055_EULER_UNIT_REG, reg_data);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
        }
        if (prev_opmode != BNO055_OPERATION_MODE_CONFIG) {
            /* set the operation mode of previous operation mode*/
            stat = this->setOperationMode(prev_opmode);
            if (stat != Drv::I2cStatus::I2C_OK) {
                return stat;
            }
        }
    }

    return Drv::I2cStatus::I2C_OK;
}

/*!
 *  \brief This API used to read
 *  mag calibration status from register from 0x35 bit 0 and 1
 *
 *  \param mag_calib_u8 : The value of mag calib status
 *
 *  \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::getMagCalibration(U8* mag_calib_u8) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_MAG_CALIB_STAT_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    *mag_calib_u8 = BNO055_GET_BITSLICE(reg_data.getData()[0], BNO055_MAG_CALIB_STAT);

    return Drv::I2cStatus::I2C_OK;
}

/*!
 *  \brief This API used to read
 *  accel calibration status from register from 0x35 bit 2 and 3
 *
 *  \param accel_calib_u8 : The value of accel calib status
 *
 *  \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::getAccelCalibration(U8* accel_calib_u8) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_ACCEL_CALIB_STAT_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    *accel_calib_u8 = BNO055_GET_BITSLICE(reg_data.getData()[0], BNO055_ACCEL_CALIB_STAT);

    return Drv::I2cStatus::I2C_OK;
}

/*!
 *  \brief This API used to read
 *  gyro calibration status from register from 0x35 bit 4 and 5
 *
 *  \param gyro_calib_u8 : The value of gyro calib status
 *
 *  \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::getGyroCalibration(U8* gyro_calib_u8) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_GYRO_CALIB_STAT_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    *gyro_calib_u8 = BNO055_GET_BITSLICE(reg_data.getData()[0], BNO055_GYRO_CALIB_STAT);

    return Drv::I2cStatus::I2C_OK;
}

/*!
 *  \brief This API used to read
 *  system calibration status from register from 0x35 bit 6 and 7
 *
 *  \param sys_calib_u8 : The value of system calib status
 *
 *  \return: I2C status of transactions
 */
Drv::I2cStatus BNO055::getSysCalibration(U8* sys_calib_u8) {
    Drv::I2cStatus stat;

    U8 data_u8r = BNO055_INIT_VALUE;
    Fw::Buffer reg_data;

    reg_data.set(&data_u8r, sizeof data_u8r);
    stat = this->readRegisterBlock(BNO055_SYS_CALIB_STAT_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        return stat;
    }
    *sys_calib_u8 = BNO055_GET_BITSLICE(reg_data.getData()[0], BNO055_SYS_CALIB_STAT);

    return Drv::I2cStatus::I2C_OK;
}

/**
 * \brief Get calibration statuses of IMU
 */
void BNO055::getCalibrations() {
    Drv::I2cStatus stat;

    U8 mag_calib = 0;
    U8 accel_calib = 0;
    U8 gyro_calib = 0;
    U8 sys_calib = 0;

    stat - this->getMagCalibration(&mag_calib);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("MAG CALIBRATE", stat);
    }

    stat - this->getAccelCalibration(&accel_calib);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("ACCEL CALIBRATE", stat);
    }

    stat - this->getGyroCalibration(&gyro_calib);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("GYRO CALIBRATE", stat);
    }

    stat - this->getSysCalibration(&sys_calib);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("SYS CALIBRATE", stat);
    }

    this->m_calibrations[0].setlabel("Magnetometer");
    this->m_calibrations[0].setvalue(mag_calib);

    this->m_calibrations[1].setlabel("Accelerometer");
    this->m_calibrations[1].setvalue(accel_calib);

    this->m_calibrations[2].setlabel("Gyroscope");
    this->m_calibrations[2].setvalue(gyro_calib);

    this->m_calibrations[3].setlabel("System");
    this->m_calibrations[3].setvalue(sys_calib);
}

/**
 * \brief Read accelerometer data from IMU
 */
void BNO055::updateAccel() {
    Drv::I2cStatus stat;

    bno055_accel_t accel;
    U8 data_u8[BNO055_ACCEL_XYZ_DATA_SIZE] = {BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
                                              BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE};
    Fw::Buffer reg_data;

    U8 accel_unit = BNO055_INIT_VALUE;
    stat = this->getAccelUnit(&accel_unit);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("ACCELEROMETER", stat);
        return;
    }
    if (accel_unit != BNO055_ACCEL_UNIT_MSQ) {
        stat = this->setAccelUnit(BNO055_ACCEL_UNIT_MSQ);
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_ImuUpdateError("ACCELEROMETER", stat);
            return;
        }
    }

    // Accel XYZ
    reg_data.set(data_u8, sizeof data_u8);
    stat = this->readRegisterBlock(BNO055_ACCEL_DATA_X_LSB_VALUEX_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("ACCELEROMETER", stat);
        return;
    }

    /* Data X */
    data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB], BNO055_ACCEL_DATA_X_LSB_VALUEX);
    data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB], BNO055_ACCEL_DATA_X_MSB_VALUEX);
    accel.x = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                    (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

    /* Data Y */
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB], BNO055_ACCEL_DATA_Y_LSB_VALUEY);
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB], BNO055_ACCEL_DATA_Y_MSB_VALUEY);
    accel.y = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                    (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

    /* Data Z */
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB], BNO055_ACCEL_DATA_Z_LSB_VALUEZ);
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB], BNO055_ACCEL_DATA_Z_MSB_VALUEZ);
    accel.z = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                    (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));

    this->m_accel.set({(F64)(accel.x / BNO055_ACCEL_DIV_MSQ), "m/s^2"},
                      {(F64)(accel.y / BNO055_ACCEL_DIV_MSQ), "m/s^2"},
                      {(F64)(accel.z / BNO055_ACCEL_DIV_MSQ), "m/s^2"});
}

/**
 * \brief Read gyroscope data from IMU
 */
void BNO055::updateGyro() {
    Drv::I2cStatus stat;

    bno055_gyro_t gyro;
    U8 data_u8[BNO055_GYRO_XYZ_DATA_SIZE] = {BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
                                             BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE};
    Fw::Buffer reg_data;

    U8 gyro_unit = BNO055_INIT_VALUE;
    stat = this->getGyroUnit(&gyro_unit);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("GYROSCOPE", stat);
        return;
    }
    if (gyro_unit != BNO055_GYRO_UNIT_DPS) {
        stat = this->setGyroUnit(BNO055_GYRO_UNIT_DPS);
        if (stat != Drv::I2cStatus::I2C_OK) {
            this->log_WARNING_HI_ImuUpdateError("GYROSCOPE", stat);
            return;
        }
    }

    // Gyro XYZ
    reg_data.set(data_u8, sizeof data_u8);
    stat = this->readRegisterBlock(BNO055_GYRO_DATA_X_LSB_VALUEX_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("GYROSCOPE", stat);
        return;
    }

    /* Data x*/
    data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB], BNO055_GYRO_DATA_X_LSB_VALUEX);
    data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB], BNO055_GYRO_DATA_X_MSB_VALUEX);
    gyro.x = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                   (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

    /* Data y*/
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB], BNO055_GYRO_DATA_Y_LSB_VALUEY);
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB], BNO055_GYRO_DATA_Y_MSB_VALUEY);
    gyro.y = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                   (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

    /* Data z*/
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB], BNO055_GYRO_DATA_Z_LSB_VALUEZ);
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB], BNO055_GYRO_DATA_Z_MSB_VALUEZ);
    gyro.z = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                   (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));

    this->m_gyro.set({(F64)(gyro.x / BNO055_GYRO_DIV_DPS), "deg/s"}, {(F64)(gyro.y / BNO055_GYRO_DIV_DPS), "deg/s"},
                     {(F64)(gyro.z / BNO055_GYRO_DIV_DPS), "deg/s"});
}

/**
 * \brief Read magnetometer data from IMU
 */
void BNO055::updateMag() {
    Drv::I2cStatus stat;

    bno055_mag_t mag;
    U8 data_u8[BNO055_MAG_XYZ_DATA_SIZE] = {BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
                                            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE};
    Fw::Buffer reg_data;

    // Mag XYZ
    reg_data.set(data_u8, sizeof data_u8);
    stat = this->readRegisterBlock(BNO055_MAG_DATA_X_LSB_VALUEX_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("MAGNETOMETER", stat);
        return;
    }

    /* Data X*/
    data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB], BNO055_MAG_DATA_X_LSB_VALUEX);
    data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB], BNO055_MAG_DATA_X_MSB_VALUEX);
    mag.x = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                  (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

    /* Data Y*/
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB], BNO055_MAG_DATA_Y_LSB_VALUEY);
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB], BNO055_MAG_DATA_Y_MSB_VALUEY);
    mag.y = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                  (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

    /* Data Z*/
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB], BNO055_MAG_DATA_Z_LSB_VALUEZ);
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB], BNO055_MAG_DATA_Z_MSB_VALUEZ);
    mag.z = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                  (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));

    this->m_mag.set({(F64)(mag.x / BNO055_MAG_DIV_UT), "uT"}, {(F64)(mag.y / BNO055_MAG_DIV_UT), "uT"},
                    {(F64)(mag.z / BNO055_MAG_DIV_UT), "uT"});
}

/**
 * \brief Read euler data from IMU
 */
void BNO055::updateEuler() {
    Drv::I2cStatus stat;

    bno055_euler_t euler;
    U8 data_u8[BNO055_EULER_HRP_DATA_SIZE] = {BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
                                              BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE};
    Fw::Buffer reg_data;

    // Euler HRP
    reg_data.set(data_u8, sizeof data_u8);
    stat = this->readRegisterBlock(BNO055_EULER_H_LSB_VALUEH_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("EULER", stat);
        return;
    }

    /* Data h*/
    data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB], BNO055_EULER_H_LSB_VALUEH);
    data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB], BNO055_EULER_H_MSB_VALUEH);
    euler.h = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                    (data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB]));

    /* Data r*/
    data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB], BNO055_EULER_R_LSB_VALUER);
    data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB], BNO055_EULER_R_MSB_VALUER);
    euler.r = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                    (data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]));

    /* Data p*/
    data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB], BNO055_EULER_P_LSB_VALUEP);
    data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB], BNO055_EULER_P_MSB_VALUEP);
    euler.p = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                    (data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]));

    this->m_euler.set({(F64)(euler.h / BNO055_EULER_DIV_DEG), "degrees"},
                      {(F64)(euler.r / BNO055_EULER_DIV_DEG), "degrees"},
                      {(F64)(euler.p / BNO055_EULER_DIV_DEG), "degrees"});
}

/**
 * \brief Read quaternion data from IMU
 */
void BNO055::updateQuaternion() {
    Drv::I2cStatus stat;

    bno055_quaternion_t quat;
    U8 data_u8[BNO055_QUATERNION_WXYZ_DATA_SIZE] = {BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
                                                    BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
                                                    BNO055_INIT_VALUE, BNO055_INIT_VALUE};
    Fw::Buffer reg_data;

    // Quaternion WXYZ
    reg_data.set(data_u8, sizeof data_u8);
    stat = this->readRegisterBlock(BNO055_QUATERNION_DATA_W_LSB_VALUEW_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("QUATERNION", stat);
        return;
    }

    /* Data W*/
    data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB], BNO055_QUATERNION_DATA_W_LSB_VALUEW);
    data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB], BNO055_QUATERNION_DATA_W_MSB_VALUEW);
    quat.w = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                   (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB]));

    /* Data X*/
    data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB], BNO055_QUATERNION_DATA_X_LSB_VALUEX);
    data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB], BNO055_QUATERNION_DATA_X_MSB_VALUEX);
    quat.x = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                   (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB]));

    /* Data Y*/
    data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB], BNO055_QUATERNION_DATA_Y_LSB_VALUEY);
    data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB], BNO055_QUATERNION_DATA_Y_MSB_VALUEY);
    quat.y = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                   (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB]));

    /* Data Z*/
    data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB], BNO055_QUATERNION_DATA_Z_LSB_VALUEZ);
    data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB], BNO055_QUATERNION_DATA_Z_MSB_VALUEZ);
    quat.z = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                   (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB]));

    this->m_quaternion.set({(F64)quat.w / (F64)(1 << 14), ""}, {(F64)quat.x / (F64)(1 << 14), ""},
                           {(F64)quat.y / (F64)(1 << 14), ""}, {(F64)quat.z / (F64)(1 << 14), ""});
}

/**
 * \brief Read linear acceleration data from IMU
 */
void BNO055::updateLinearAccel() {
    Drv::I2cStatus stat;

    bno055_linear_accel_t linear_accel;
    U8 data_u8[BNO055_ACCEL_XYZ_DATA_SIZE] = {BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
                                              BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE};
    Fw::Buffer reg_data;

    // Linear Accel XYZ
    reg_data.set(data_u8, sizeof data_u8);
    stat = this->readRegisterBlock(BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("LINEAR ACCELERATION", stat);
        return;
    }

    /* Data x*/
    data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB], BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX);
    data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB], BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX);
    linear_accel.x = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                           (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

    /* Data y*/
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB], BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY);
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB], BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY);
    linear_accel.y = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                           (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

    /* Data z*/
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB], BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ);
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB], BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ);
    linear_accel.z = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                           (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));

    this->m_linear_accel.set({(F64)(linear_accel.x / BNO055_LINEAR_ACCEL_DIV_MSQ), "m/s^2"},
                             {(F64)(linear_accel.y / BNO055_LINEAR_ACCEL_DIV_MSQ), "m/s^2"},
                             {(F64)(linear_accel.z / BNO055_LINEAR_ACCEL_DIV_MSQ), "m/s^2"});
}

/**
 * \brief Read gravitational acceleration data from IMU
 */
void BNO055::updateGravity() {
    Drv::I2cStatus stat;

    bno055_gravity_t gravity;
    U8 data_u8[BNO055_GRAVITY_XYZ_DATA_SIZE] = {BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
                                                BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE};

    Fw::Buffer reg_data;

    // Gravity XYZ
    reg_data.set(data_u8, sizeof data_u8);
    stat = this->readRegisterBlock(BNO055_GRAVITY_DATA_X_LSB_VALUEX_REG, reg_data);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_ImuUpdateError("GRAVITATIONAL ACCELERATION", stat);
        return;
    }

    /* Data x*/
    data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB], BNO055_GRAVITY_DATA_X_LSB_VALUEX);
    data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB], BNO055_GRAVITY_DATA_X_MSB_VALUEX);
    gravity.x = (I16)(((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB]) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

    /* Data y*/
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB], BNO055_GRAVITY_DATA_Y_LSB_VALUEY);
    data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB], BNO055_GRAVITY_DATA_Y_MSB_VALUEY);
    gravity.y = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

    /* Data z*/
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB], BNO055_GRAVITY_DATA_Z_LSB_VALUEZ);
    data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] =
        BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB], BNO055_GRAVITY_DATA_Z_MSB_VALUEZ);
    gravity.z = (I16)((((I32)((I8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));

    this->m_gravity.set({(F64)(gravity.x / BNO055_GRAVITY_DIV_MSQ), "m/s^2"},
                        {(F64)(gravity.y / BNO055_GRAVITY_DIV_MSQ), "m/s^2"},
                        {(F64)(gravity.z / BNO055_GRAVITY_DIV_MSQ), "m/s^2"});
}

}  // namespace Sensors
