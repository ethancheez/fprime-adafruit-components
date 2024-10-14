// ======================================================================
// \title  PA1010D.cpp
// \author ethancheez
// \brief  cpp file for PA1010D component implementation class
// ======================================================================

#include "Components/PA1010D/PA1010D.hpp"
#include "FpConfig.hpp"
#include "Fw/Logger/Logger.hpp"

#include <string>

namespace Sensors {

/**
 * \brief Construct PA1010D object
 */
PA1010D ::PA1010D(const char* const compName)
    : PA1010DComponentBase(compName),
      m_polyDb_offset(0),
      m_i2cDevAddress(0x10),
      gps_utc_date("", "ddmmyy"),
      gps_utc_time("", "hhmmss.sss"),
      gps_latitude("", "ddmm.mmmm"),
      gps_latitude_NS("N"),
      gps_longitude("", "dddmm.mmmm"),
      gps_longitude_EW("W"),
      gps_speed(0.0, "km/hr"),
      gps_altitude(0.0, "m"),
      nmea_states{'0', '1', '1', '1', '0', '0'} {}

/**
 * \brief Destroy PA1010D object
 */
PA1010D ::~PA1010D() {}

/**
 * \brief Configure the GPS
 *
 * Sets the default NMEA outputs to have RMC, VTG, and GGA
 */
void PA1010D::config() {
    CHAR pmtk[] = "PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
    pmtk[8] = this->nmea_states[0];
    pmtk[10] = this->nmea_states[1];
    pmtk[12] = this->nmea_states[2];
    pmtk[14] = this->nmea_states[3];
    pmtk[16] = this->nmea_states[4];
    pmtk[18] = this->nmea_states[5];

    Fw::Buffer buf(reinterpret_cast<U8*>(pmtk), sizeof pmtk);
    this->sendCommand(buf);
}

/**
 * \brief Sets the offset index to store telemetry into PolyDB
 *
 * \param offset: the offset index value
 */
void PA1010D::setPolyDbOffset(U32 offset) {
    this->m_polyDb_offset = offset;
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

//! Handler implementation for run
void PA1010D ::run_handler(NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context) {
    // NMEA State Telemetry
    Sensors::GPS_NMEA_States states;
    states[0] = {Sensors::GPS_NMEA_OUTPUTS::GLL, (this->nmea_states[0] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[1] = {Sensors::GPS_NMEA_OUTPUTS::RMC, (this->nmea_states[1] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[2] = {Sensors::GPS_NMEA_OUTPUTS::VTG, (this->nmea_states[2] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[3] = {Sensors::GPS_NMEA_OUTPUTS::GGA, (this->nmea_states[3] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[4] = {Sensors::GPS_NMEA_OUTPUTS::GSA, (this->nmea_states[4] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[5] = {Sensors::GPS_NMEA_OUTPUTS::GSV, (this->nmea_states[5] == '1') ? Fw::On::ON : Fw::On::OFF};

    if (this->isConnected_tlmOut_OutputPort(0)) {
        this->tlmWrite_NMEA_States(states);
    }

    // Get GPS Data
    U8 data[256] = {'\0'};
    Fw::Buffer buf(data, sizeof data);
    Drv::I2cStatus stat = this->i2cRead_out(0, this->m_i2cDevAddress, buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return;
    }

    if (buf.getData() == nullptr || buf.getSize() < 6) {
        return;
    }

    parse_nmea(buf, "$GNRMC");
    parse_nmea(buf, "$GNVTG");
    parse_nmea(buf, "$GNGGA");

    if (this->isConnected_tlmOut_OutputPort(0)) {
        GPS_Time gps_time;
        gps_time.setutc_date(this->gps_utc_date);
        gps_time.setutc_time(this->gps_utc_time);
        this->tlmWrite_GPS_Time(gps_time);

        GPS_Location gps_location;
        gps_location.setlatitude(this->gps_latitude);
        gps_location.setlat_NS(this->gps_latitude_NS);
        gps_location.setlongitude(this->gps_longitude);
        gps_location.setlng_EW(this->gps_longitude_EW);
        this->tlmWrite_GPS_Location(gps_location);

        this->tlmWrite_GPS_Speed(this->gps_speed);

        this->tlmWrite_GPS_Altitude(this->gps_altitude);
    }

    if (this->isConnected_setPolyDb_OutputPort(0)) {
        Fw::PolyType vals[] = {
            static_cast<void*>(const_cast<char*>(this->gps_utc_date.getvalue().toChar())),
            static_cast<void*>(const_cast<char*>(this->gps_utc_time.getvalue().toChar())),
            static_cast<void*>(const_cast<char*>(this->gps_latitude.getvalue().toChar())),
            static_cast<void*>(const_cast<char*>(this->gps_latitude_NS.toChar())),
            static_cast<void*>(const_cast<char*>(this->gps_longitude.getvalue().toChar())),
            static_cast<void*>(const_cast<char*>(this->gps_longitude_EW.toChar())),
            this->gps_speed.getvalue(),
            this->gps_altitude.getvalue(),
        };
        Svc::MeasurementStatus mstat = Svc::MeasurementStatus::OK;
        Fw::Time ts(TB_NONE, 0, 0);

        for (U32 entry = 0; entry < 8; entry++) {
            this->setPolyDb_out(0, entry + m_polyDb_offset, mstat, ts, vals[entry]);

            if (mstat != Svc::MeasurementStatus::OK) {
                this->log_WARNING_HI_PolyDbSetError(mstat);
            }
        }
    }
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

//! Handler implementation for command SET_NMEA_OUTPUT
void PA1010D ::SET_NMEA_OUTPUT_cmdHandler(FwOpcodeType opCode,
                                          U32 cmdSeq,
                                          Sensors::GPS_NMEA_OUTPUTS nmea,
                                          Fw::On state) {
    U8 index = 0;
    switch (nmea) {
        case Sensors::GPS_NMEA_OUTPUTS::GLL:
            index = 0;
            break;
        case Sensors::GPS_NMEA_OUTPUTS::RMC:
            index = 1;
            break;
        case Sensors::GPS_NMEA_OUTPUTS::VTG:
            index = 2;
            break;
        case Sensors::GPS_NMEA_OUTPUTS::GGA:
            index = 3;
            break;
        case Sensors::GPS_NMEA_OUTPUTS::GSA:
            index = 4;
            break;
        case Sensors::GPS_NMEA_OUTPUTS::GSV:
            index = 5;
    }

    this->nmea_states[index] = (state == Fw::On::ON) ? '1' : '0';

    CHAR pmtk[] = "PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
    pmtk[8] = this->nmea_states[0];
    pmtk[10] = this->nmea_states[1];
    pmtk[12] = this->nmea_states[2];
    pmtk[14] = this->nmea_states[3];
    pmtk[16] = this->nmea_states[4];
    pmtk[18] = this->nmea_states[5];

    Fw::Buffer buf(reinterpret_cast<U8*>(pmtk), sizeof pmtk);
    Drv::I2cStatus stat = this->sendCommand(buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

//! Handler implementation for command SET_UPDATE_RATE_MS
void PA1010D ::SET_UPDATE_RATE_MS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U16 ms) {
    std::string ms_str = std::to_string(ms);
    CHAR pmtk_header[] = "PMTK220,";

    CHAR pmtk[sizeof pmtk_header + ms_str.length()];
    memcpy(pmtk, pmtk_header, sizeof pmtk_header);
    memcpy(&pmtk[sizeof pmtk_header], &ms_str[0], ms_str.length());

    Fw::Buffer buf(reinterpret_cast<U8*>(pmtk), sizeof pmtk);
    Drv::I2cStatus stat = this->sendCommand(buf);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

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
std::vector<U8> PA1010D::get_delimiters(Fw::Buffer buf, const CHAR* protocol) {
    U32 start = 0;
    U32 end = 0;
    bool found = false;
    for (U32 i = 0; i < buf.getSize(); i++) {
        if (strncmp((const char*)&buf.getData()[i], protocol, 6) == 0) {
            start = i;
            found = true;
        } else if (found && (buf.getData()[i] == '$' || (buf.getData()[i] == '\n' && buf.getData()[i + 1] == '\n'))) {
            break;
        }
        end = i;
    }

    std::vector<U8> delimiters;
    for (U32 i = start; i <= end; i++) {
        if (buf.getData()[i] == ',') {
            delimiters.push_back(i);
        }
    }

    return delimiters;
}

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
void PA1010D::parse_nmea(Fw::Buffer buf, const CHAR* protocol) {
    std::vector<U8> delimiters = this->get_delimiters(buf, protocol);

    if (delimiters.size() == 0) {
        return;
    }

    for (U32 i = 0; i < delimiters.size() - 1; i++) {
        if (delimiters[i] > delimiters[i + 1]) {
            return;
        }
    }

    // for (U32 i = delimiters[0] - 6; i <= delimiters[delimiters.size() - 1] + 4; i++) {
    //     Fw::Logger::logMsg("%c", buf.getData()[i]);
    // }
    // Fw::Logger::logMsg("\n");

    if (protocol == "$GNRMC" && delimiters.size() == 12 && buf.getData()[delimiters[1] + 1] == 'A') {
        CHAR utc_date[15] = {'\0'};
        CHAR utc_time[15] = {'\0'};
        CHAR lat[15] = {'\0'};
        CHAR lng[15] = {'\0'};
        CHAR lat_NS[15] = {'\0'};
        CHAR lng_EW[15] = {'\0'};

        if (delimiters[1] - delimiters[0] <= 1 || delimiters[3] - delimiters[2] <= 1 ||
            delimiters[4] - delimiters[3] <= 1 || delimiters[5] - delimiters[4] <= 1 ||
            delimiters[6] - delimiters[5] <= 1 || delimiters[9] - delimiters[8] <= 1) {
            return;
        }

        memcpy(utc_time, &buf.getData()[delimiters[0] + 1], delimiters[1] - delimiters[0] - 1);
        memcpy(lat, &buf.getData()[delimiters[2] + 1], delimiters[3] - delimiters[2] - 1);
        memcpy(lat_NS, &buf.getData()[delimiters[3] + 1], delimiters[4] - delimiters[3] - 1);
        memcpy(lng, &buf.getData()[delimiters[4] + 1], delimiters[5] - delimiters[4] - 1);
        memcpy(lng_EW, &buf.getData()[delimiters[5] + 1], delimiters[6] - delimiters[5] - 1);
        memcpy(utc_date, &buf.getData()[delimiters[8] + 1], delimiters[9] - delimiters[8] - 1);

        this->gps_utc_date.setvalue(utc_date);
        this->gps_utc_time.setvalue(utc_time);
        this->gps_latitude.setvalue(lat);
        this->gps_latitude_NS = Fw::String(lat_NS);
        this->gps_longitude.setvalue(lng);
        this->gps_longitude_EW = Fw::String(lng_EW);
    } else if (protocol == "$GNVTG" && delimiters.size() == 9 && delimiters[7] - delimiters[6] - 1 > 0) {
        CHAR speed[15] = {'\0'};
        memset(speed, 0, sizeof(speed));

        memcpy(speed, &buf.getData()[delimiters[6] + 1], delimiters[7] - delimiters[6] - 1);

        try {
            F64 speed_val = std::stof(speed);
            this->gps_speed.setvalue(speed_val);
        } catch (const std::exception& e) {
            // Invalid float value
            return;
        }

    } else if (protocol == "$GNGGA" && delimiters.size() == 14 && buf.getData()[delimiters[5] + 1] == '1') {
        CHAR altitude[15] = {'\0'};
        memset(altitude, 0, sizeof(altitude));

        memcpy(altitude, &buf.getData()[delimiters[8] + 1], delimiters[9] - delimiters[8] - 1);

        try {
            F64 altitude_val = std::stof(altitude);
            this->gps_altitude.setvalue(altitude_val);
        } catch (const std::exception& e) {
            // Invalid float value
            return;
        }

    } else {
        return;
    }
}

/**
 * \brief Sends a command to the GPS
 *
 * The command must follow the PMTK formats. First index must be '$', and a
 * checksum is calculated and appended to the end of the command.
 *
 * \param buf: buffer contaning the command bytes to send
 * \return: I2C status from the write call
 */
Drv::I2cStatus PA1010D::sendCommand(Fw::Buffer buf) {
    U8 cmd[buf.getSize() + 1 + 3 + 2];

    cmd[0] = '$';
    memcpy(&cmd[1], buf.getData(), buf.getSize());

    cmd[sizeof cmd - 5] = '*';

    U8 checksum = 0;
    for (U8 i = 0; i < buf.getSize(); i++) {
        checksum ^= buf.getData()[i];
    }

    CHAR hexDigits[] = "0123456789ABCDEF";
    cmd[sizeof cmd - 4] = static_cast<U8>(hexDigits[(checksum >> 4) & 0x0F]);
    cmd[sizeof cmd - 3] = static_cast<U8>(hexDigits[checksum & 0x0F]);

    cmd[sizeof cmd - 2] = '\r';
    cmd[sizeof cmd - 1] = '\n';

    Fw::Buffer cmd_buf(cmd, sizeof cmd);
    return this->i2cWrite_out(0, this->m_i2cDevAddress, cmd_buf);
}

}  // namespace Sensors
