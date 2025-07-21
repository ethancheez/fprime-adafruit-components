// ======================================================================
// \title  BNO085.hpp
// \author ethan
// \brief  hpp file for BNO085 component implementation class
// ======================================================================

#ifndef Sensors_BNO085_HPP
#define Sensors_BNO085_HPP

#include "Components/BNO085/BNO085ComponentAc.hpp"

namespace Sensors {

class BNO085 final : public BNO085ComponentBase {
    static const U8 HEADER_SIZE = 4;                                    //!< Size of the header in bytes
    static const U32 DATA_BUFFER_SIZE = 512;                            //!< Size of the data buffer for reading packets
    static const U32 MAX_PACKET_SIZE = HEADER_SIZE + DATA_BUFFER_SIZE;  //!< Maximum packet size including header
    static const U32 _BNO_DEFAULT_REPORT_INTERVAL = 50000;              // 50 ms

    typedef enum {
        _BNO_CHANNEL_SHTP_COMMAND = 0x00,
        _BNO_CHANNEL_EXE,
        _BNO_CHANNEL_CONTROL,
        _BNO_CHANNEL_INPUT_SENSOR_REPORTS,
        _BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS,
        _BNO_CHANNEL_GYRO_ROTATION_VECTOR,
    } BNO085_Channels;

    typedef enum {
        _SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8,
        _SHTP_REPORT_PRODUCT_ID_REQUEST = 0xF9,
    } BNO085_SHTP_Report;

    typedef enum {
        _BNO_REPORT_ACCELEROMETER = 0x01,         // Calibrated Acceleration (m/s2)
        _BNO_REPORT_GYROSCOPE = 0x02,             // Calibrated Angular Velocity (rad/s)
        _BNO_REPORT_MAGNETOMETER = 0x03,          // Calibrated Magnetic Field (uT)
        _BNO_REPORT_LINEAR_ACCELERATION = 0x04,   // Calibrated Linear Acceleration (m/s2)
        _BNO_REPORT_ROTATION_VECTOR = 0x05,       // Rotation Vector (quaternion)
        _BNO_REPORT_GRAVITY = 0x06,               // Gravity Vector (m/s2)
        _BNO_REPORT_GAME_ROTATION_VECTOR = 0x08,  // Game Rotation Vector (quaternion)
        _BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
        _BNO_REPORT_STEP_COUNTER = 0x11,
        _BNO_REPORT_RAW_ACCELEROMETER = 0x14,
        _BNO_REPORT_RAW_GYROSCOPE = 0x15,
        _BNO_REPORT_RAW_MAGNETOMETER = 0x16,
        _BNO_REPORT_SHAKE_DETECTOR = 0x19,
        _BNO_REPORT_STABILITY_CLASSIFIER = 0x13,
        _BNO_REPORT_ACTIVITY_CLASSIFIER = 0x1E,
        _BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A,
    } BNO085_Features;

    typedef enum {
        _GET_FEATURE_REQUEST = 0xFE,
        _SET_FEATURE_COMMAND = 0xFD,
        _GET_FEATURE_RESPONSE = 0xFC,
        _BASE_TIMESTAMP = 0xFB,
        _TIMESTAMP_REBASE = 0xFA,
    } BNO085_Feature_Commands;

    typedef struct {
        U16 packet_length;   // Length of the packet in bytes
        U8 channel;          // Channel number
        U8 sequence_number;  // Sequence number of the packet
    } BNO085_Packet_Header;

    typedef struct {
        BNO085_Packet_Header header;  // Header of the packet
        U8 data[DATA_BUFFER_SIZE];    // Data buffer for the packet
    } BNO085_Packet;

  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct BNO085 object
    BNO085(const char* const compName  //!< The component name
    );

    //! Destroy BNO085 object
    ~BNO085();

    void config_i2c(U32 address);

    void config();

  private:
    //! Handler implementation for run
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
    );

  private:
    Drv::I2cStatus sendPacket(U8 channel, Fw::Buffer& buffer);

    Drv::I2cStatus readPacket(Fw::Buffer& buffer);

    bool findPacket(U8 channel, U8 reportId);

    void packetize(U8 channel, Fw::Buffer data, Fw::Buffer& ret);

    BNO085_Packet unpacketize(Fw::Buffer buffer);

    BNO085_Packet_Header unpacketizeHeader(Fw::Buffer buffer);

    Drv::I2cStatus softwareReset();

    void getDeviceId();

    Drv::I2cStatus enableFeature(U8 featureId);

    void parseReadings(BNO085_Packet packet);

    U32 reportLengthLookup(U8 reportId);

    void quatToEuler(BNO085_IMU_WXYZ_F64 quat);

  private:
    U32 m_i2cDevAddress = 0x4A;

    U8 m_sequenceNumberTable[6] = {0};
    U8 m_featureEnabledTable[16] = {0};

    BNO085_IMU_XYZ_F64 m_accel;
    BNO085_IMU_XYZ_F64 m_gyro;
    BNO085_IMU_XYZ_F64 m_mag;
    BNO085_IMU_XYZ_F64 m_euler;
    BNO085_IMU_WXYZ_F64 m_quaternion;
    BNO085_IMU_XYZ_F64 m_linear_accel;
    BNO085_IMU_XYZ_F64 m_gravity;
};

}  // namespace Sensors

#endif
