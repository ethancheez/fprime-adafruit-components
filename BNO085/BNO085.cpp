// ======================================================================
// \title  BNO085.cpp
// \author ethan
// \brief  cpp file for BNO085 component implementation class
// ======================================================================

#include "Components/BNO085/BNO085.hpp"
#include <cmath>
#include "Fw/Logger/Logger.hpp"

namespace Adafruit {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

BNO085 ::BNO085(const char* const compName)
    : BNO085ComponentBase(compName),
      m_accel({0, Fw::String("m/s^2")}, {0, Fw::String("m/s^2")}, {0, Fw::String("m/s^2")}),
      m_gyro({0, Fw::String("deg/s")}, {0, Fw::String("deg/s")}, {0, Fw::String("deg/s")}),
      m_mag({0, Fw::String("uT")}, {0, Fw::String("uT")}, {0, Fw::String("uT")}),
      m_euler({0, Fw::String("deg")}, {0, Fw::String("deg")}, {0, Fw::String("deg")}),
      m_quaternion({0, Fw::String("")}, {0, Fw::String("")}, {0, Fw::String("")}, {0, Fw::String("")}),
      m_linear_accel({0, Fw::String("m/s^2")}, {0, Fw::String("m/s^2")}, {0, Fw::String("m/s^2")}),
      m_gravity({0, Fw::String("m/s^2")}, {0, Fw::String("m/s^2")}, {0, Fw::String("m/s^2")}) {}

BNO085 ::~BNO085() {}

void BNO085::config_i2c(U32 address) {
    // Set the I2C device address
    this->m_i2cDevAddress = address;
}

void BNO085::config() {
    this->softwareReset();
    this->getDeviceId();

    this->enableFeature(_BNO_REPORT_ACCELEROMETER);
    this->enableFeature(_BNO_REPORT_GYROSCOPE);
    this->enableFeature(_BNO_REPORT_MAGNETOMETER);
    this->enableFeature(_BNO_REPORT_LINEAR_ACCELERATION);
    this->enableFeature(_BNO_REPORT_ROTATION_VECTOR);
    this->enableFeature(_BNO_REPORT_GRAVITY);
}

void BNO085::run_handler(FwIndexType portNum,  //!< The port number
                         U32 context           //!< The call order
) {
    Drv::I2cStatus status;

    U8 read[DATA_BUFFER_SIZE] = {0};
    Fw::Buffer readBuffer(read, DATA_BUFFER_SIZE);
    status = this->readPacket(readBuffer);
    FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

    BNO085_Packet packet = this->unpacketize(readBuffer);
    this->parseReadings(packet);
}

Drv::I2cStatus BNO085::sendPacket(U8 channel, Fw::Buffer& buffer) {
    Drv::I2cStatus status = Drv::I2cStatus::I2C_OPEN_ERR;

    U8 packet[buffer.getSize() + 4];
    Fw::Buffer packetBuffer(packet, buffer.getSize() + 4);
    this->packetize(channel, buffer, packetBuffer);  // Create a packet with the data to be sent

    status = this->i2cWrite_out(0, this->m_i2cDevAddress, packetBuffer);
    FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

    this->m_sequenceNumberTable[channel] =
        (this->m_sequenceNumberTable[channel] + 1) % 256;  // Increment sequence number

    return status;
}

Drv::I2cStatus BNO085::readPacket(Fw::Buffer& buffer) {
    Drv::I2cStatus status = Drv::I2cStatus::I2C_OPEN_ERR;

    // Read header
    U8 buf[4] = {0};
    Fw::Buffer headerBuffer(buf, 4);

    U8 maxIter = 255;
    while (headerBuffer.getData()[0] == 0 && headerBuffer.getData()[1] == 0 && headerBuffer.getData()[2] == 0 &&
           headerBuffer.getData()[3] == 0 && maxIter > 0) {
        status = this->i2cRead_out(0, this->m_i2cDevAddress, headerBuffer);
        FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);
        maxIter--;
    }

    BNO085_Packet_Header header = this->unpacketizeHeader(headerBuffer);

    // Fw::Logger::log("Expecting %d bytes... ", header.packet_length - 4);

    this->m_sequenceNumberTable[header.channel] = header.sequence_number;
    if (header.packet_length == 0) {
        memset(buffer.getData(), 0, buffer.getSize());  // Clear the buffer if packet length is zero
        buffer.setSize(0);                              // Set the size of the buffer to zero
        return Drv::I2cStatus::I2C_OK;
    }
    U16 dataLen = header.packet_length - 4;  // Adjust packet length to exclude header

    if (dataLen > buffer.getSize()) {
        Fw::Logger::log("Error: Packet length exceeds buffer size\n");
        return Drv::I2cStatus::I2C_OTHER_ERR;
    }

    // Read data
    status = this->i2cRead_out(0, this->m_i2cDevAddress, buffer);
    buffer.setSize(dataLen);  // Set the size of the buffer to the packet length
    FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

    // Fw::Logger::log("Read %d bytes...\n", buffer.getSize());

    // Update sequence number
    this->m_sequenceNumberTable[header.channel] = header.sequence_number;

    return status;
}

void BNO085::packetize(U8 channel, Fw::Buffer data, Fw::Buffer& ret) {
    U16 writeLen = data.getSize() + 4;
    U8 buffer[writeLen];
    buffer[0] = writeLen & 0xFF;                         // LSB packet length
    buffer[1] = (writeLen >> 8) & 0xFF;                  // MSB packet length
    buffer[2] = channel;                                 // Channel number
    buffer[3] = this->m_sequenceNumberTable[channel];    // Sequence number
    memcpy(&buffer[4], data.getData(), data.getSize());  // Copy the data to be sent

    FW_ASSERT(writeLen <= ret.getSize(), writeLen, ret.getSize());
    memcpy(ret.getData(), buffer, writeLen);  // Copy the packet to the return

    // Fw::Logger::log("============PACKET============\n");
    // Fw::Logger::log("Data Length: %d\n", writeLen - 4);
    // Fw::Logger::log("Channel: %d\n", channel);
    // Fw::Logger::log("Sequence Number: %d\n", this->m_sequenceNumberTable[channel]);
    // Fw::Logger::log("==========PACKETIZED==========\n");
}

BNO085::BNO085_Packet BNO085::unpacketize(Fw::Buffer buffer) {
    FW_ASSERT(buffer.getSize() >= sizeof(BNO085_Packet_Header), buffer.getSize());

    BNO085_Packet packet;
    packet.header = this->unpacketizeHeader(buffer);
    memcpy(packet.data, &buffer.getData()[4], buffer.getSize() - 4);  // Copy the data to the packet

    // Fw::Logger::log("============PACKET============\n");
    // Fw::Logger::log("Data Length: %d\n", packet.header.packet_length - 4);
    // Fw::Logger::log("Channel: %d\n", packet.header.channel);
    // Fw::Logger::log("Sequence Number: %d\n", packet.header.sequence_number);
    // Fw::Logger::log("=========UNPACKETIZED=========\n");

    return packet;
}

BNO085::BNO085_Packet_Header BNO085::unpacketizeHeader(Fw::Buffer buffer) {
    FW_ASSERT(buffer.getSize() >= sizeof(BNO085_Packet_Header), buffer.getSize());

    BNO085_Packet_Header header;
    header.packet_length = buffer.getData()[0] | (buffer.getData()[1] << 8);  // Read length from header
    header.packet_length &= ~0x8000;
    header.channel = buffer.getData()[2];          // Read channel from header
    header.sequence_number = buffer.getData()[3];  // Read sequence number from header

    return header;
}

Drv::I2cStatus BNO085::softwareReset() {
    Fw::Logger::log("BNO085: Software Reset...\n");
    Drv::I2cStatus status;

    U8 buffer[1] = {1};
    Fw::Buffer packet(buffer, 1);

    this->sendPacket(_BNO_CHANNEL_EXE, packet);
    Os::Task::delay(Fw::TimeInterval(0, 500000));  // Delay to allow the reset to complete
    this->sendPacket(_BNO_CHANNEL_EXE, packet);
    Os::Task::delay(Fw::TimeInterval(0, 500000));  // Delay to allow the reset to complete

    U8 read[DATA_BUFFER_SIZE] = {0};
    Fw::Buffer readBuffer(read, DATA_BUFFER_SIZE);
    status = this->readPacket(readBuffer);
    FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

    FW_ASSERT(readBuffer.getSize() > 0, readBuffer.getSize());

    return status;
}

void BNO085::getDeviceId() {
    Drv::I2cStatus status;

    U8 buffer[2] = {_SHTP_REPORT_PRODUCT_ID_REQUEST, 0x00};
    Fw::Buffer packet(buffer, 2);

    status = this->sendPacket(_BNO_CHANNEL_CONTROL, packet);
    FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

    U8 read[DATA_BUFFER_SIZE] = {0};
    Fw::Buffer readBuffer(read, DATA_BUFFER_SIZE);
    status = this->readPacket(readBuffer);
    FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

    U8 channel = readBuffer.getData()[2];
    U8 reportId = readBuffer.getData()[4];  // Report ID in first index of data buffer
    for (U32 i = 0; channel != 2 || reportId != _SHTP_REPORT_PRODUCT_ID_RESPONSE; i++) {
        FW_ASSERT(i < 6, i);
        status = this->sendPacket(_BNO_CHANNEL_CONTROL, packet);
        FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

        memset(readBuffer.getData(), 0, readBuffer.getSize());  // Clear the buffer
        readBuffer.setSize(DATA_BUFFER_SIZE);
        status = this->readPacket(readBuffer);
        FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

        channel = readBuffer.getData()[2];
        reportId = readBuffer.getData()[4];  // Report ID in first index of data buffer
    }

    U8 sw_major = readBuffer.getData()[4 + 2];  // Software major version
    U8 sw_minor = readBuffer.getData()[4 + 3];  // Software minor version
    U16 sw_patch;
    memcpy(&sw_patch, &readBuffer.getData()[4 + 12], sizeof(U16));  // Software patch version
    U32 sw_part_number;
    memcpy(&sw_part_number, &readBuffer.getData()[4 + 4], sizeof(U32));  // Software part number
    U32 sw_build_number;
    memcpy(&sw_build_number, &readBuffer.getData()[4 + 8], sizeof(U32));  // Software build number

    Fw::Logger::log("[BNO085] Software Version: %d.%d.%d\n", sw_major, sw_minor, sw_patch);
    Fw::Logger::log("[BNO085] Part Number: %d\n", sw_part_number);
    Fw::Logger::log("[BNO085] Build Number: %d\n", sw_build_number);
}

Drv::I2cStatus BNO085::enableFeature(U8 featureId) {
    U32 reportInterval = _BNO_DEFAULT_REPORT_INTERVAL;  // Default report interval
    U32 sensorSpecificConfig = 0;                       // Default sensor-specific configuration

    if (featureId == _BNO_REPORT_ACTIVITY_CLASSIFIER) {
        sensorSpecificConfig = 0x1FF;  // Enable activity classifier
    }

    U8 set_feature_report[17] = {0};
    set_feature_report[0] = _SET_FEATURE_COMMAND;
    set_feature_report[1] = featureId;
    memcpy(&set_feature_report[5], &reportInterval, sizeof(U32));
    memcpy(&set_feature_report[13], &sensorSpecificConfig, sizeof(U32));

    bool hasFeatureDependency = false;
    U8 featureDependency = 0x00;
    switch (featureId) {
        case _BNO_REPORT_ACCELEROMETER:
            featureDependency = _BNO_REPORT_RAW_ACCELEROMETER;
            hasFeatureDependency = true;
            break;
        case _BNO_REPORT_GYROSCOPE:
            featureDependency = _BNO_REPORT_RAW_GYROSCOPE;
            hasFeatureDependency = true;
            break;
        case _BNO_REPORT_MAGNETOMETER:
            featureDependency = _BNO_REPORT_RAW_MAGNETOMETER;
            hasFeatureDependency = true;
            break;
        default:
            break;
    }

    if (hasFeatureDependency) {
        bool alreadyEnabled = false;
        for (U8 i = 0; i < sizeof(this->m_featureEnabledTable); i++) {
            if (this->m_featureEnabledTable[i] == featureDependency) {
                Fw::Logger::log("Feature %d already enabled, skipping dependency check\n", featureDependency);
                alreadyEnabled = true;
                break;
            }
        }

        if (!alreadyEnabled) {
            this->enableFeature(featureDependency);  // Ew recursion. Sorry I broke the rules.
        }
    }

    Fw::Buffer buffer(set_feature_report, sizeof(set_feature_report));
    Drv::I2cStatus status = this->sendPacket(_BNO_CHANNEL_CONTROL, buffer);
    FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

    for (U8 i = 0; i < 10; i++) {
        U8 read[DATA_BUFFER_SIZE] = {0};
        Fw::Buffer readBuffer(read, DATA_BUFFER_SIZE);
        status = this->readPacket(readBuffer);
        FW_ASSERT(status == Drv::I2cStatus::I2C_OK, status);

        if (readBuffer.getSize() > 4 && readBuffer.getData()[4] == _GET_FEATURE_RESPONSE) {
            U8 responseFeatureId = readBuffer.getData()[5];
            Fw::Logger::log("Feature 0x%02X enabled successfully\n", responseFeatureId);

            // Add to feature enabled table
            for (U8 i = 0; i < sizeof(this->m_featureEnabledTable); i++) {
                if (this->m_featureEnabledTable[i] == 0) {
                    this->m_featureEnabledTable[i] = responseFeatureId;  // Add the feature ID
                }
            }
        }
    }

    return status;
}

void BNO085::parseReadings(BNO085_Packet packet) {
    U32 dataLen = packet.header.packet_length - 4;  // Adjust packet length to exclude header

    // for (U32 i = 0; i < dataLen; i++) {
    //     Fw::Logger::log("0x%02X ", packet.data[i]);
    // }
    // Fw::Logger::log("\n");

    for (U32 idx = 0; idx < dataLen;) {
        U8 reportId = packet.data[idx];
        U32 reportLength = this->reportLengthLookup(reportId);
        FW_ASSERT(reportLength > 0, reportId);

        U32 unprocessed_byte_count = dataLen - idx;
        FW_ASSERT(unprocessed_byte_count >= reportLength, unprocessed_byte_count, reportLength);

        U8 slice[reportLength] = {0};
        memcpy(slice, &packet.data[idx], reportLength);  // Copy the report data

        // Fw::Logger::log("SLICE: ");
        // for (U32 j = 0; j < reportLength; j++) {
        //     Fw::Logger::log("0x%02X ", slice[j]);
        // }
        // Fw::Logger::log("\n");

        // Accuracy
        U8 accuracy = slice[2];
        accuracy &= 0b11;

        switch (slice[0]) {
            case _BNO_REPORT_ACCELEROMETER: {
                // Process Accelerometer data
                F64 x = static_cast<I16>(slice[4] | (slice[5] << 8)) * (1 / 256.0f);
                F64 y = static_cast<I16>(slice[6] | (slice[7] << 8)) * (1 / 256.0f);
                F64 z = static_cast<I16>(slice[8] | (slice[9] << 8)) * (1 / 256.0f);
                Fw::Logger::log("Accuracy: %d\n", accuracy);
                Fw::Logger::log("Accelerometer: X=%.2f, Y=%.2f, Z=%.2f\n", x, y, z);

                this->m_accel.set({x, Fw::String("m/s^2")}, {y, Fw::String("m/s^2")}, {z, Fw::String("m/s^2")});
                this->tlmWrite_Accel(this->m_accel);

                break;
            }
            case _BNO_REPORT_GYROSCOPE: {
                // Process Raw Accelerometer data
                F64 x = static_cast<I16>(slice[4] | (slice[5] << 8)) * (1 / 512.0f);
                F64 y = static_cast<I16>(slice[6] | (slice[7] << 8)) * (1 / 512.0f);
                F64 z = static_cast<I16>(slice[8] | (slice[9] << 8)) * (1 / 512.0f);
                // Fw::Logger::log("Gyroscope: X=%.2f, Y=%.2f, Z=%.2f\n", x, y, z);

                this->m_gyro.set({x, Fw::String("deg/s")}, {y, Fw::String("deg/s")}, {z, Fw::String("deg/s")});
                this->tlmWrite_Gyro(this->m_gyro);

                break;
            }
            case _BNO_REPORT_MAGNETOMETER: {
                // Process Magnetometer data
                F64 x = static_cast<I16>(slice[4] | (slice[5] << 8)) * (1 / 16.0f);
                F64 y = static_cast<I16>(slice[6] | (slice[7] << 8)) * (1 / 16.0f);
                F64 z = static_cast<I16>(slice[8] | (slice[9] << 8)) * (1 / 16.0f);
                // Fw::Logger::log("Magnetometer: X=%.2f, Y=%.2f, Z=%.2f\n", x, y, z);

                this->m_mag.set({x, Fw::String("uT")}, {y, Fw::String("uT")}, {z, Fw::String("uT")});
                this->tlmWrite_Mag(this->m_mag);

                break;
            }
            case _BNO_REPORT_LINEAR_ACCELERATION: {
                // Process Linear Acceleration data
                F64 x = static_cast<I16>(slice[4] | (slice[5] << 8)) * (1 / 256.0f);
                F64 y = static_cast<I16>(slice[6] | (slice[7] << 8)) * (1 / 256.0f);
                F64 z = static_cast<I16>(slice[8] | (slice[9] << 8)) * (1 / 256.0f);
                // Fw::Logger::log("Linear Acceleration: X=%.2f, Y=%.2f, Z=%.2f\n", x, y, z);

                this->m_linear_accel.set({x, Fw::String("m/s^2")}, {y, Fw::String("m/s^2")}, {z, Fw::String("m/s^2")});
                this->tlmWrite_LinearAccel(this->m_linear_accel);

                break;
            }
            case _BNO_REPORT_ROTATION_VECTOR: {
                // Process Rotation Vector data
                F64 w = static_cast<I16>(slice[4] | (slice[5] << 8)) * (1 / 16384.0f);
                F64 x = static_cast<I16>(slice[6] | (slice[7] << 8)) * (1 / 16384.0f);
                F64 y = static_cast<I16>(slice[8] | (slice[9] << 8)) * (1 / 16384.0f);
                F64 z = static_cast<I16>(slice[10] | (slice[11] << 8)) * (1 / 16384.0f);
                // Fw::Logger::log("Rotation Vector: W=%.2f, X=%.2f, Y=%.2f, Z=%.2f\n", w, x, y, z);

                this->m_quaternion.set({w, Fw::String("")}, {x, Fw::String("")}, {y, Fw::String("")},
                                       {z, Fw::String("")});
                this->tlmWrite_Quat(this->m_quaternion);

                this->quatToEuler(this->m_quaternion);  // Convert quaternion to Euler angles
                this->tlmWrite_Euler(this->m_euler);

                break;
            }
            case _BNO_REPORT_GRAVITY: {
                // Process Gravity data
                F64 x = static_cast<I16>(slice[4] | (slice[5] << 8)) * (1 / 256.0f);
                F64 y = static_cast<I16>(slice[6] | (slice[7] << 8)) * (1 / 256.0f);
                F64 z = static_cast<I16>(slice[8] | (slice[9] << 8)) * (1 / 256.0f);
                // Fw::Logger::log("Gravity: X=%.2f, Y=%.2f, Z=%.2f\n", x, y, z);

                this->m_gravity.set({x, Fw::String("m/s^2")}, {y, Fw::String("m/s^2")}, {z, Fw::String("m/s^2")});
                this->tlmWrite_Gravity(this->m_gravity);

                break;
            }
            default:
                break;
        }

        idx += reportLength;  // Move to the next report
    }
}

U32 BNO085::reportLengthLookup(U8 reportId) {
    switch (reportId) {
        // Sensor Reports
        case _BNO_REPORT_ACCELEROMETER:
        case _BNO_REPORT_GYROSCOPE:
        case _BNO_REPORT_MAGNETOMETER:
        case _BNO_REPORT_LINEAR_ACCELERATION:
        case _BNO_REPORT_GRAVITY:
            return 10;
        case _BNO_REPORT_ROTATION_VECTOR:
        case _BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR:
            return 14;
        case _BNO_REPORT_GAME_ROTATION_VECTOR:
        case _BNO_REPORT_STEP_COUNTER:
            return 12;
        case _BNO_REPORT_SHAKE_DETECTOR:
        case _BNO_REPORT_STABILITY_CLASSIFIER:
            return 6;
        case _BNO_REPORT_ACTIVITY_CLASSIFIER:
        case _BNO_REPORT_RAW_ACCELEROMETER:
        case _BNO_REPORT_RAW_GYROSCOPE:
        case _BNO_REPORT_RAW_MAGNETOMETER:
            return 16;
        // Other Reports
        case _SHTP_REPORT_PRODUCT_ID_RESPONSE:
            return 16;
        case _GET_FEATURE_RESPONSE:
            return 17;
        case _BASE_TIMESTAMP:
        case _TIMESTAMP_REBASE:
            return 5;
        default:
            return 0;  // Unknown report ID (or unimplemented)
    }
}

void BNO085::quatToEuler(BNO085_IMU_WXYZ_F64 quat) {
    F64 w = quat.getw().getvalue();
    F64 x = quat.getx().getvalue();
    F64 y = quat.gety().getvalue();
    F64 z = quat.getz().getvalue();

    // Construct rotation matrix elements matching getTbn convention
    // For a rotation matrix R, in terms of yaw (ψ), pitch (θ), and roll (φ):
    // R = Rz(ψ)Ry(θ)Rx(φ)
    // This gives matrix elements:
    F64 R11 = w * w + x * x - y * y - z * z;  // cos(θ)cos(ψ)
    F64 R12 = 2.0 * (x * y - w * z);          // cos(θ)sin(ψ)
    F64 R13 = 2.0 * (x * z + w * y);          // -sin(θ)
    F64 R23 = 2.0 * (y * z - w * x);          // cos(θ)sin(φ)
    F64 R33 = w * w - x * x - y * y + z * z;  // cos(θ)cos(φ)
    F64 R31 = 2.0 * (x * z - w * y);          // -sin(θ)
    F64 R32 = 2.0 * (y * z + w * x);          // cos(θ)sin(φ)

    // Initialize Euler angles
    F64 yaw, pitch, roll;

    // Check for gimbal lock (pitch at ±90 degrees)
    // Gimbal lock occurs when pitch (θ) = ±90°, or when R31 = -sin(θ) = ∓1
    // At these angles, cos(θ) = 0, causing yaw and roll rotations to occur around the same axis
    const F64 GIMBAL_LOCK_THRESHOLD = 0.9999999;  // Near ±90° accounting for numerical precision

    if (R31 > GIMBAL_LOCK_THRESHOLD) {
        // When R31 = 1, pitch = -90°
        // At this point:
        // R11 = R12 = 0, R13 = -1
        // R21 = 0, R22 = cos(ψ-φ), R23 = 0
        // R31 = 1, R32 = sin(ψ-φ), R33 = 0
        pitch = -M_PI / 2.0;
        // In gimbal lock, yaw and roll become coupled - they rotate around the same axis
        // Only their difference (at pitch = -90°) or sum (at pitch = 90°) is meaningful
        // We conventionally set yaw = 0 and calculate roll
        yaw = 0.0;
        roll = std::atan2(-R12, R11);
    } else if (R31 < -GIMBAL_LOCK_THRESHOLD) {
        // When R31 = -1, pitch = 90°
        // At this point:
        // R11 = R12 = 0, R13 = 1
        // R21 = 0, R22 = cos(ψ+φ), R23 = 0
        // R31 = -1, R32 = sin(ψ+φ), R33 = 0
        pitch = M_PI / 2.0;
        yaw = 0.0;
        roll = std::atan2(-R12, R11);
    } else {
        // Normal case - no gimbal lock
        // Standard calculations from rotation matrix elements
        pitch = std::atan2(-R31, std::sqrt(R32 * R32 + R33 * R33));
        yaw = std::atan2(R12, R11);
        roll = std::atan2(R32, R33);
    }

    // Convert radians to degrees for output
    yaw = yaw * (180.0 / M_PI);
    pitch = pitch * (180.0 / M_PI);
    roll = roll * (180.0 / M_PI);

    // Fw::Logger::log("Yaw: %.2f deg, Pitch: %.2f deg, Roll: %.2f deg\n", yaw, pitch, roll);

    this->m_euler.set({yaw, Fw::String("deg")}, {pitch, Fw::String("deg")}, {roll, Fw::String("deg")});
}

}  // namespace Adafruit
