// ======================================================================
// \title  PCA9685.hpp
// \author ethancheez
// \brief  hpp file for PCA9685 component implementation class
// ======================================================================

#ifndef Components_PCA9685_HPP
#define Components_PCA9685_HPP

#include "Components/PCA9685/PCA9685ComponentAc.hpp"

namespace Components {

class PCA9685 : public PCA9685ComponentBase {
  public:
    /**
     * \brief Construct PCA9685 object
     */
    PCA9685(const char* const compName  //!< The component name
    );

    /**
     * \brief Destroy PCA9685 object
     */
    ~PCA9685();

    /**
     * \brief Configure the PCA9685 Servo Driver
     *
     * Resets the device and sets the PWM frequency
     */
    void config(U8 numChannels);

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

    //! Handler implementation for setAngle
    //!
    //! Port to set a channel angle called by other components
    void setAngle_handler(NATIVE_INT_TYPE portNum,  //!< The port number
                          U8 channel,               //!< Channel number to set
                          U8 angle                  //!< Angle to set channel
    );

    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command CALIBRATE
    //!
    //! Performs a series of motions to verify servos
    void CALIBRATE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                              U32 cmdSeq            //!< The command sequence number
    );

    //! Handler implementation for command SET_ANGLE_OFFSET
    //!
    //! Command to set the angle offset
    void SET_ANGLE_OFFSET_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                     U32 cmdSeq,           //!< The command sequence number
                                     U8 channel,
                                     I8 angle_offset);

    //! Handler implementation for command SET_ANGLE
    //!
    //! Command to set the angle, with consideration of the offset
    void SET_ANGLE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                              U32 cmdSeq,           //!< The command sequence number
                              U8 channel,
                              U8 angle);

    //! Handler implementation for command SET_PWM_RAW
    //!
    //! Command to directly set a PWM value to the channel
    void SET_PWM_RAW_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                U32 cmdSeq,           //!< The command sequence number
                                U8 channel,
                                U16 pwm);

    //! Handler implementation for command SET_PWM_RANGE
    //!
    //! Command to set the minimum and maximum values of the PWM value
    void SET_PWM_RANGE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                  U32 cmdSeq,           //!< The command sequence number
                                  U8 channel,
                                  U16 min_pwm,
                                  U16 max_pwm);

    //! Handler implementation for command RESET_ALL_ANGLES
    //!
    //! Reset all channel angles
    void RESET_ALL_ANGLES_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                     U32 cmdSeq            //!< The command sequence number
    );

    //! Handler implementation for command RESET_ALL_ANGLE_OFFSETS
    //!
    //! Reset all channel angle offsets
    void RESET_ALL_ANGLE_OFFSETS_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                            U32 cmdSeq            //!< The command sequence number
    );

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
    Drv::I2cStatus setAngle(U8 channel, U8 angle);

    /**
     * \brief Converts an angle (in degrees) to a PWM value
     *
     * PWM value is dependent on the minimum and maximum PWM values.
     *
     * \param angle: the angle in degrees (0-180)
     * \return: the PWM value
     */
    U16 angleToPWM(U8 angle);

    /**
     * \brief Sets the PWM value to the specified channel
     *
     * \param channel: the PWM channel (0-15)
     * \param pwm: the PWM value
     * \return: I2C status of transactions
     */
    Drv::I2cStatus setPWM(U8 channel, U16 pwm);

    /**
     * \brief Gets the PWM value of the specified channel
     *
     * \param channel: the PWM channel (0-15)
     * \param pwm: the PWM value to set
     * \return: I2C status of transactions
     */
    Drv::I2cStatus getPWM(U8 channel, U16* pwm);

    /**
     * \brief Sets up the device to know what register the next read should be from
     *
     * The device requires a write call with a register's address before a read will function correctly. This helper
     * sets up that read address by writing it to the device via the I2C write port.
     *
     * \param reg: device internal address to the first register to be read
     * \return: I2C status from the write call
     */
    Drv::I2cStatus setupReadRegister(U8 reg);

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
    Drv::I2cStatus sendByteCommand(U8 startRegister, U8 byte);

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
    Drv::I2cStatus readRegisterByte(U8 startRegister, U8* byte);

    // ----------------------------------------------------------------------
    // Member variables
    // ----------------------------------------------------------------------

    const U8 REG_MODE_REG1 = 0x00;
    const U8 REG_MODE_REG2 = 0x01;
    const U8 REG_CHANNEL0_ON_L = 0x06;
    const U8 REG_CHANNEL0_ON_H = 0x07;
    const U8 REG_CHANNEL0_OFF_L = 0x08;
    const U8 REG_CHANNEL0_OFF_H = 0x09;
    const U8 CHANNEL_MULTIPLYER = 4;
    const U8 REG_PRESCALE = 0xFE;

    U32 m_i2cDevAddress;

    U32 m_polyDb_offset;

    U8 m_channels;
    U64 m_referenceClockSpeed;
    U32 m_frequency;

    U8 m_defaultAngle;

    U64 m_minPWM;
    U64 m_maxPWM;

    I8 m_offsets[16] = {0};
    Components::PCA9685_ANGLES m_angles = {0};
};

}  // namespace Components

#endif
