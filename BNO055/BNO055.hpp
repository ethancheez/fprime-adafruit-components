// ======================================================================
// \title  BNO055.hpp
// \author ethancheez
// \brief  hpp file for BNO055 component implementation class
// ======================================================================

#ifndef Adafruit_BNO055_HPP
#define Adafruit_BNO055_HPP

#include "BNO055_Registers.hpp"
#include "Components/BNO055/BNO055ComponentAc.hpp"

namespace Adafruit {

class BNO055 : public BNO055ComponentBase {
  public:
    /**
     * \brief Construct BNO055 object
     */
    BNO055(const char* const compName  //!< The component name
    );

    /**
     * \brief Destroy BNO055 object
     */
    ~BNO055();

    /**
     * \brief Configure the IMU
     *
     * Sets the default page ID of the IMU's page ID register to 0x00, configures the power mode to `Normal`,
     * and finally sets the operation mode to `NDOF` to begin reading IMU data.
     */
    void config();

    /**
     * \brief Sets the offset index to store telemetry into PolyDB
     *
     * \param offset: the offset index value
     */
    void setPolyDbOffset(U32 offset);

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    void run_handler(NATIVE_INT_TYPE portNum,  //!< The port number
                     NATIVE_UINT_TYPE context  //!< The call order
    );

    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    /**
     * \brief Sets up the IMU to know what register the next read should be from
     *
     * The IMU requires a write call with a register's address before a read will function correctly. This helper
     * sets up that read address by writing it to the IMU via the I2C write port.
     *
     * \param reg: IMU internal address to the first register to be read
     * \return: I2C status from the write call
     */
    Drv::I2cStatus setupReadRegister(U8 reg);

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
    Drv::I2cStatus writeRegisterBlock(U8 startRegister, Fw::Buffer& buffer);

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
    Drv::I2cStatus readRegisterBlock(U8 startRegister, Fw::Buffer& buffer);

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
    Drv::I2cStatus writePageId(U8 pageId);

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
    Drv::I2cStatus getOperationMode(U8* operation_mode);

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
    Drv::I2cStatus setOperationMode(U8 operation_mode);

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
    Drv::I2cStatus setPowerMode(U8 power_mode);

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
    Drv::I2cStatus getAccelUnit(U8* accel_unit);

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
    Drv::I2cStatus setAccelUnit(U8 accel_unit);

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
    Drv::I2cStatus getGyroUnit(U8* gyro_unit);

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
    Drv::I2cStatus setGyroUnit(U8 gyro_unit);

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
    Drv::I2cStatus getEulerUnit(U8* euler_unit);

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
    Drv::I2cStatus setEulerUnit(U8 euler_unit);

    /**
     * \brief Read accelerometer data from IMU
     */
    void updateAccel();

    /**
     * \brief Read gyroscope data from IMU
     */
    void updateGyro();

    /**
     * \brief Read magnetometer data from IMU
     */
    void updateMag();

    /**
     * \brief Read euler data from IMU
     */
    void updateEuler();

    /**
     * \brief Read quaternion data from IMU
     */
    void updateQuaternion();

    /**
     * \brief Read linear acceleration data from IMU
     */
    void updateLinearAccel();

    /**
     * \brief Read gravitational acceleration data from IMU
     */
    void updateGravity();

    // ----------------------------------------------------------------------
    // Member Variables
    // ----------------------------------------------------------------------

    U32 m_i2cDevAddress;
    U8 m_page_id;

    U32 m_polyDb_offset;

    Adafruit::IMU_XYZ_F64 m_accel;
    Adafruit::IMU_XYZ_F64 m_gyro;
    Adafruit::IMU_XYZ_F64 m_mag;
    Adafruit::IMU_XYZ_F64 m_euler;
    Adafruit::IMU_WXYZ_F64 m_quaternion;
    Adafruit::IMU_XYZ_F64 m_linear_accel;
    Adafruit::IMU_XYZ_F64 m_gravity;
};

}  // namespace Adafruit

#endif
