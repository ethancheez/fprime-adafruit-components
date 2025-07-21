// ======================================================================
// \title  MPL3115A2.hpp
// \author ethancheez
// \brief  hpp file for MPL3115A2 component implementation class
// ======================================================================

#ifndef Sensors_MPL3115A2_HPP
#define Sensors_MPL3115A2_HPP

#include "Components/MPL3115A2/MPL3115A2ComponentAc.hpp"

namespace Sensors {

class MPL3115A2 : public MPL3115A2ComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct MPL3115A2 object
    MPL3115A2(const char* const compName  //!< The component name
    );

    //! Destroy MPL3115A2 object
    ~MPL3115A2();

    void config();

    /**
     * \brief Sets the offset index to store telemetry into PolyDB
     *
     * \param offset: the offset index value
     */
    void setPolyDbOffset(U32 offset);

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SET_SEA_LEVEL_PRESSURE
    //!
    //! Command to set the sea level pressure in hPa
    void SET_SEA_LEVEL_PRESSURE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                           U32 cmdSeq,           //!< The command sequence number
                                           F64 sea_level_pressure);

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    //!
    //! Port to receive calls from the rate group
    void run_handler(NATIVE_INT_TYPE portNum,  //!< The port number
                     NATIVE_UINT_TYPE context  //!< The call order
    );

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
    Drv::I2cStatus setupReadRegister(U8 reg);

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
    Drv::I2cStatus writeRegisterBlock(U8 startRegister, Fw::Buffer& buffer);

    /**
     * \brief Reads a block of data from the specified register of the barometer
     *
     * This function starts by writing the startRegister to the barometer by passing it to `setupReadRegister`. It then
     * calls the read port of the I2C bus to read data from the barometer. It will read `buffer.getSize()` bytes from
     * the I2C device and as such the caller must set this up.
     *
     * \param startRegister: register address to start reading from
     * \param buffer: buffer to read into. Determines size of read.
     * \return: I2C status of transactions
     */
    Drv::I2cStatus readRegisterBlock(U8 startRegister, Fw::Buffer& buffer);

    Drv::I2cStatus pollCtrlReg1Mask(U8 mask, U8 *regValue);

    Drv::I2cStatus pollStatus(U8 mask);

    Drv::I2cStatus setSeaLevelPressure(F64 pressure);

    Drv::I2cStatus getPressure(F64 *pressure);

    Drv::I2cStatus getAltitude(F64 *altitude);

    Drv::I2cStatus getTemperature(F64 *temperature);

    U32 m_i2cDevAddress;
    F64 m_seaLevelPressure;

    U32 m_polyDb_offset;

    Sensors::MPL3115A2_DataUnit_F64 m_pressure;
    Sensors::MPL3115A2_DataUnit_F64 m_altitude;
    Sensors::MPL3115A2_DataUnit_F64 m_temperature;
};

}  // namespace Sensors

#endif
