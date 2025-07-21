// ======================================================================
// \title  PA1010D.hpp
// \author ethancheez
// \brief  hpp file for PA1010D component implementation class
// ======================================================================

#ifndef Sensors_PA1010D_HPP
#define Sensors_PA1010D_HPP

#include "Components/PA1010D/PA1010DComponentAc.hpp"

#include <vector>

namespace Sensors {

class PA1010D : public PA1010DComponentBase {

  struct RMCPacket {
    F32 utcTime;
    CHAR status;
    F32 latitude;
    CHAR lat_NS;
    F32 longitude;
    CHAR lng_EW;
    F32 speed;
    F32 course;
    U32 date;
  };

  struct GGAPacket {
    F32 utcTime;
    F32 latitude;
    CHAR lat_NS;
    F32 longitude;
    CHAR lng_EW;
    U32 fix;
    U32 satsUsed;
    F32 hdop;
    F32 msl_altitude;
  };

  public:
    /**
     * \brief Construct PA1010D object
     */
    PA1010D(const char* const compName  //!< The component name
    );

    /**
     * \brief Destroy PA1010D object
     */
    ~PA1010D();

    /**
     * \brief Configure the GPS
     *
     * Sets the default NMEA outputs to have RMC, VTG, and GGA
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
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context  //!< The call order
    );

    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SET_NMEA_OUTPUT
    void SET_NMEA_OUTPUT_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                    U32 cmdSeq,           //!< The command sequence number
                                    Sensors::GPS_NMEA_OUTPUTS nmea,
                                    Fw::On state);

    //! Handler implementation for command ENABLE_CONSTELLATIONS
    //!
    //! Command to enable/disable constellation use
    void ENABLE_CONSTELLATIONS_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                          U32 cmdSeq,           //!< The command sequence number
                                          bool GPS,
                                          bool GLONASS,
                                          bool GALILEO,
                                          bool BEIDOU);

    //! Handler implementation for command SET_UPDATE_RATE_MS
    void SET_UPDATE_RATE_MS_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                       U32 cmdSeq,           //!< The command sequence number
                                       U16 ms);

    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    /**
     * \brief Parses the NMEA frame for the specified protocol
     *
     * The function makes use of `get_delimiters()` to know where the correct NMEA
     * frame is located within `buf`. The parsed data is stored into private member
     * variables.
     *
     * \param buf: buffer contaning the entire byte stream read from the GPS
     * \param protocol: the protocol string to parse
     */
    void parse_nmea(Fw::Buffer buf, const CHAR* protocol);

    /**
     * \brief Sends a command to the GPS
     *
     * The command must follow the PMTK formats. First index must be '$', and a
     * checksum is calculated and appended to the end of the command.
     *
     * \param buf: buffer contaning the command bytes to send
     * \return: I2C status from the write call
     */
    Drv::I2cStatus sendCommand(Fw::Buffer cmd);

    // ----------------------------------------------------------------------
    // Member variables
    // ----------------------------------------------------------------------

    bool m_configured = false;

    U32 m_i2cDevAddress;

    U32 m_polyDb_offset;

    CHAR nmea_states[6];

    GPS_DataUnit gps_utc_date;
    GPS_DataUnit gps_utc_time;
    GPS_DataUnit_F64 gps_latitude;
    Fw::String gps_latitude_NS;
    GPS_DataUnit_F64 gps_longitude;
    Fw::String gps_longitude_EW;
    GPS_DataUnit_F64 gps_speed;
    GPS_DataUnit_F64 gps_altitude;
    I32 gps_sats_used;
    GPS_Constellations_NumSatellites gps_sats_used_constellations;
    F64 gps_hdop;
};

}  // namespace Sensors

#endif
