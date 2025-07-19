// ======================================================================
// \title  BMP390.hpp
// \author ethan
// \brief  hpp file for BMP390 component implementation class
// ======================================================================

#ifndef Adafruit_BMP390_HPP
#define Adafruit_BMP390_HPP

#include "BMP390_Registers.hpp"
#include "Components/BMP390/BMP390ComponentAc.hpp"

namespace Adafruit {

class BMP390 final : public BMP390ComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct BMP390 object
    BMP390(const char* const compName  //!< The component name
    );

    //! Destroy BMP390 object
    ~BMP390();

    void config();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    //!
    //! Port to receive calls from the rate group
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SET_SEA_LEVEL_PRESSURE
    void SET_SEA_LEVEL_PRESSURE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                           U32 cmdSeq,           //!< The command sequence number
                                           F64 sea_level_pressure) override;

  private:
    bool reset();

    Drv::I2cStatus readRegister(U8 startRegister, U8& byte);

    Drv::I2cStatus writeRegister(U8 startRegister, U8 byte);

    Drv::I2cStatus writeRegisterBlock(U8 startRegister, Fw::Buffer& buffer);

    Drv::I2cStatus readRegisterBlock(U8 startRegister, Fw::Buffer& buffer);

    Drv::I2cStatus setupReadRegister(U8 reg);

    I8 cal_crc(U8 seed, U8 data);

    void setTemperatureOversampling(U8 oversample);

    void setPressureOversampling(U8 oversample);

    bool setIIRFilterCoeff(U8 filtercoeff);

    bool setOutputDataRate(U8 odr);

    bool performReading(void);

    Drv::I2cStatus sleep();

    Drv::I2cStatus setSensorSettings(U32 desired_settings);

    Drv::I2cStatus set_pwr_ctrl_settings(U32 desired_settings);

    Drv::I2cStatus set_odr_filter_settings(U32 desired_settings);

    Drv::I2cStatus set_int_ctrl_settings(U32 desired_settings);

    Drv::I2cStatus set_advance_settings(U32 desired_settings);

    Drv::I2cStatus set_op_mode(U8 opmode);

    Drv::I2cStatus get_op_mode(U8& opmode);

    Drv::I2cStatus set_normal_mode();

    Drv::I2cStatus write_power_mode();

    Drv::I2cStatus validate_normal_mode_settings();

    Drv::I2cStatus get_odr_filter_settings();

    void parse_calib_data(Fw::Buffer& buffer);

    void fill_osr_data(U32 settings, U8* addr, Fw::Buffer& reg_data, U8& len);

    void fill_odr_data(U8* addr, Fw::Buffer& reg_data, U8& len);

    void fill_filter_data(U8* addr, Fw::Buffer& reg_data, U8& len);

    void validate_osr_and_odr_settings();

    F64 compensate_temperature(const struct bmp3_uncomp_data* uncomp_data);

    F64 compensate_pressure(const struct bmp3_uncomp_data* uncomp_data);

  private:
    U8 m_i2cDevAddress = 0x77;
    F64 m_seaLevelPressure = 1013.25;  //!< Sea level pressure in hPa

    /*! Sensor Settings */
    struct bmp3_settings settings;

    /*! Trim data */
    struct bmp3_calib_data calib_data;

    bool _filterEnabled = false;
    bool _tempOSEnabled = false;
    bool _presOSEnabled = false;
    bool _ODREnabled = false;

    // Pressure and temperature data
    F64 m_pressure = 0.0;
    F64 m_temperature = 0.0;
};

}  // namespace Adafruit

#endif
