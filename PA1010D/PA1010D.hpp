// ======================================================================
// \title  PA1010D.hpp
// \author ethancheez
// \brief  hpp file for PA1010D component implementation class
// ======================================================================

#ifndef Adafruit_PA1010D_HPP
#define Adafruit_PA1010D_HPP

#include "Components/PA1010D/PA1010DComponentAc.hpp"

#include <vector>

namespace Adafruit {

class PA1010D : public PA1010DComponentBase {
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
    void run_handler(NATIVE_INT_TYPE portNum,  //!< The port number
                     NATIVE_UINT_TYPE context  //!< The call order
    );

    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SET_NMEA_OUTPUT
    void SET_NMEA_OUTPUT_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                    U32 cmdSeq,           //!< The command sequence number
                                    Adafruit::GPS_NMEA_OUTPUTS nmea,
                                    Fw::On state);

    //! Handler implementation for command SET_UPDATE_RATE_MS
    void SET_UPDATE_RATE_MS_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                       U32 cmdSeq,           //!< The command sequence number
                                       U16 ms);

    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    /**
     * \brief Get the indices of the delimiters (',') for the protocol frame
     *
     * The function searches for a match of the protocol and stores all indices of ','
     * within a vector until the next protocol is read (indicated by '$'), or when the
     * end of the buffer is reached (can also be indicated by a sequence of "\n\n").
     *
     * \param buf: buffer contaning the entire byte stream read from the GPS
     * \param protocol: the protocol string to look for
     * \return: a vector containing the indices of all the delimiters for that protocol
     */
    std::vector<U8> get_delimiters(Fw::Buffer buf, const CHAR* protocol);

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

    U32 m_i2cDevAddress;

    U32 m_polyDb_offset;

    CHAR nmea_states[6];

    GPS_DataUnit gps_utc_date;
    GPS_DataUnit gps_utc_time;
    GPS_DataUnit gps_latitude;
    Fw::String gps_latitude_NS;
    GPS_DataUnit gps_longitude;
    Fw::String gps_longitude_EW;
    GPS_DataUnit_F64 gps_speed;
    GPS_DataUnit_F64 gps_altitude;
};

}  // namespace Adafruit

#endif
