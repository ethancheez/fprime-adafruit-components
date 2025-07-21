// ======================================================================
// \title  PA1010D.cpp
// \author ethancheez
// \brief  cpp file for PA1010D component implementation class
// ======================================================================

#include "Components/PA1010D/PA1010D.hpp"
#include <config/FpConfig.hpp>
#include "Fw/Logger/Logger.hpp"

#include <cmath>
#include <string>

namespace Adafruit {

/**
 * \brief Construct PA1010D object
 */
PA1010D ::PA1010D(const char* const compName)
    : PA1010DComponentBase(compName),
      m_polyDb_offset(0),
      m_i2cDevAddress(0x10),
      gps_utc_date(Fw::String(""), Fw::String("ddmmyy")),
      gps_utc_time(Fw::String(""), Fw::String("hhmmss.sss")),
      gps_latitude(0.0, Fw::String("degrees")),
      gps_latitude_NS(Fw::String((std::string() + 'N').c_str())),
      gps_longitude(0.0, Fw::String("degrees")),
      gps_longitude_EW(Fw::String((std::string() + 'W').c_str())),
      gps_speed(0.0, Fw::String("m/s")),
      gps_altitude(0.0, Fw::String("m")),
      nmea_states{'0', '1', '0', '1', '0', '0'},  // [GLL, RMC, VTG, GGA, GSA, GSV]
      gps_sats_used(0),
      gps_hdop(0.0) {
    this->gps_sats_used_constellations[0] = {Fw::String("GPS"), 0};
    this->gps_sats_used_constellations[1] = {Fw::String("GLONASS"), 0};
    this->gps_sats_used_constellations[2] = {Fw::String("GALILEO"), 0};
    this->gps_sats_used_constellations[3] = {Fw::String("BEIDOU"), 0};
}

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
    // Set default NMEA Outputs
    {
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
    // Enable all constellations
    {
        CHAR pmtk[] = "PMTK353,1,1,1,0,1";
        Fw::Buffer buf(reinterpret_cast<U8*>(pmtk), sizeof pmtk);
        this->sendCommand(buf);
    }
    // Set update rate
    {
        CHAR pmtk[] = "PMTK220,100";
        Fw::Buffer buf(reinterpret_cast<U8*>(pmtk), sizeof pmtk);
        this->sendCommand(buf);

        CHAR pmtk2[] = "PMTK300,200,0,0,0,0";
        Fw::Buffer buf2(reinterpret_cast<U8*>(pmtk2), sizeof pmtk2);
        this->sendCommand(buf2);
    }

    this->m_configured = true;
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
void PA1010D ::run_handler(FwIndexType portNum, U32 context) {
    if (!this->m_configured) {
        return;
    }

    Os::RawTime now;
    now.now();
    U32 dt_us;  // [us]
    now.getDiffUsec(this->last_ts, dt_us);
    this->last_ts.now();
    F64 freq = 1 / (dt_us / 1000000.0);  // [Hz]

    // NMEA State Telemetry
    Adafruit::GPS_NMEA_States states;
    states[0] = {Adafruit::GPS_NMEA_OUTPUTS::GLL, (this->nmea_states[0] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[1] = {Adafruit::GPS_NMEA_OUTPUTS::RMC, (this->nmea_states[1] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[2] = {Adafruit::GPS_NMEA_OUTPUTS::VTG, (this->nmea_states[2] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[3] = {Adafruit::GPS_NMEA_OUTPUTS::GGA, (this->nmea_states[3] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[4] = {Adafruit::GPS_NMEA_OUTPUTS::GSA, (this->nmea_states[4] == '1') ? Fw::On::ON : Fw::On::OFF};
    states[5] = {Adafruit::GPS_NMEA_OUTPUTS::GSV, (this->nmea_states[5] == '1') ? Fw::On::ON : Fw::On::OFF};

    if (this->isConnected_tlmOut_OutputPort(0)) {
        this->tlmWrite_NMEA_States(states);
        this->tlmWrite_freq(freq);
    }

    // Get GPS Data
    U8 reg_data[256];
    Fw::Buffer buffer(reg_data, 256);
    Drv::I2cStatus stat = this->i2cRead_out(0, this->m_i2cDevAddress, buffer);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
        return;
    }

    if (buffer.getData() == nullptr || buffer.getSize() < 6 || buffer.getData()[0] != '$') {
        return;
    }

    if (nmea_states[0] == '1') {
        // parse_nmea(buffer, "GLL"); // GLL is not used in the current implementation
    }
    if (nmea_states[1] == '1') {
        parse_nmea(buffer, "RMC");
    }
    if (nmea_states[2] == '1') {
        // parse_nmea(buffer, "VTG"); // VTG is not used in the current implementation
    }
    if (nmea_states[3] == '1') {
        parse_nmea(buffer, "GGA");
    }
    if (nmea_states[4] == '1') {
        parse_nmea(buffer, "GSA");
    }
    if (nmea_states[5] == '1') {
        // parse_nmea(buffer, "GSV"); // GSV is not used in the current implementation
    }

    if (this->isConnected_setPolyDb_OutputPort(0)) {
        Fw::PolyType vals[] = {
            static_cast<void*>(const_cast<char*>(this->gps_utc_date.getvalue().toChar())),
            static_cast<void*>(const_cast<char*>(this->gps_utc_time.getvalue().toChar())),
            this->gps_latitude.getvalue(),
            static_cast<void*>(const_cast<char*>(this->gps_latitude_NS.toChar())),
            this->gps_longitude.getvalue(),
            static_cast<void*>(const_cast<char*>(this->gps_longitude_EW.toChar())),
            this->gps_speed.getvalue(),
            this->gps_altitude.getvalue(),
            this->gps_sats_used,
            this->gps_hdop,
        };
        Svc::MeasurementStatus mstat = Svc::MeasurementStatus::OK;
        Fw::Time ts(TB_NONE, 0, 0);

        for (U32 entry = 0; entry < 10; entry++) {
            Svc::PolyDbCfg::PolyDbEntry dbEntry = static_cast<Svc::PolyDbCfg::PolyDbEntry::T>(entry + m_polyDb_offset);
            this->setPolyDb_out(0, dbEntry, mstat, ts, vals[entry]);

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
                                          Adafruit::GPS_NMEA_OUTPUTS nmea,
                                          Fw::On state) {
    U8 index = 0;
    switch (nmea) {
        case Adafruit::GPS_NMEA_OUTPUTS::GLL:
            index = 0;
            break;
        case Adafruit::GPS_NMEA_OUTPUTS::RMC:
            index = 1;
            break;
        case Adafruit::GPS_NMEA_OUTPUTS::VTG:
            index = 2;
            break;
        case Adafruit::GPS_NMEA_OUTPUTS::GGA:
            index = 3;
            break;
        case Adafruit::GPS_NMEA_OUTPUTS::GSA:
            index = 4;
            break;
        case Adafruit::GPS_NMEA_OUTPUTS::GSV:
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

    Fw::Buffer buffer(reinterpret_cast<U8*>(pmtk), sizeof pmtk);
    Drv::I2cStatus stat = this->sendCommand(buffer);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

//! Handler implementation for command ENABLE_CONSTELLATIONS
void PA1010D ::ENABLE_CONSTELLATIONS_cmdHandler(FwOpcodeType opCode,
                                                U32 cmdSeq,
                                                bool GPS,
                                                bool GLONASS,
                                                bool GALILEO,
                                                bool BEIDOU) {
    CHAR pmtk[] = "PMTK353,0,0,0,0,0";

    if (GPS) {
        pmtk[8] = '1';
    }
    if (GLONASS) {
        pmtk[10] = '1';
    }
    if (GALILEO) {
        pmtk[12] = '1';
    }
    if (BEIDOU) {
        pmtk[16] = '1';
    }

    Fw::Buffer buffer(reinterpret_cast<U8*>(pmtk), sizeof pmtk);
    Drv::I2cStatus stat = this->sendCommand(buffer);
    if (stat != Drv::I2cStatus::I2C_OK) {
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

    Fw::Buffer buffer(reinterpret_cast<U8*>(pmtk), sizeof pmtk);
    Drv::I2cStatus stat = this->sendCommand(buffer);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

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
void PA1010D::parse_nmea(Fw::Buffer buf, const CHAR* protocol) {
    // Keep looping buffer because there can be multiple of the same packet within the buffer
    U32 protocolIndex = 0;
    while (protocolIndex < buf.getSize()) {
        // Find protocol in buffer
        for (; protocolIndex < buf.getSize(); protocolIndex++) {
            if (strncmp((const char*)&buf.getData()[protocolIndex], protocol, 3) == 0) {
                break;
            }
        }
        if (protocolIndex == buf.getSize()) {
            return;
        }

        // Debug print
        // for (U32 i = protocolIndex; i < buf.getSize() && buf.getData()[i] != '\n'; i++) {
        //     printf("%c", buf.getData()[i]);
        // }
        // printf("\n");

        CHAR* pointer = reinterpret_cast<CHAR*>(&buf.getData()[protocolIndex]);
        if (strcmp(protocol, "RMC") == 0) {
            RMCPacket packet;
            FwIndexType ret =
                sscanf(pointer, "RMC,%f,%c,%f,%c,%f,%c,%f,%f,%u", &packet.utcTime, &packet.status, &packet.latitude,
                       &packet.lat_NS, &packet.longitude, &packet.lng_EW, &packet.speed, &packet.course, &packet.date);
            // printf("RMC: %f %c %f %c %f %c %f %f %u\n", packet.utcTime, packet.status, packet.latitude,
            // packet.lat_NS,
            //        packet.longitude, packet.lng_EW, packet.speed, packet.course, packet.date);

            // Not all items were found, skip
            if (ret < 9) {
                protocolIndex += 1;
                continue;
            }

            // Lat conversion to ##.####
            U32 lat_dd = packet.latitude / 100;
            F32 lat_mm = std::fmod(packet.latitude, 100);
            F32 lat_deg = lat_dd + lat_mm / 60;
            if (packet.lat_NS == 'S') {
                lat_deg *= -1;
            }
            // Lng conversion to ##.####
            U32 lng_ddd = packet.longitude / 100;
            F32 lng_mm = std::fmod(packet.longitude, 100);
            F32 lng_deg = lng_ddd + lng_mm / 60;
            if (packet.lng_EW == 'W') {
                lng_deg *= -1;
            }

            F32 speed_mps = packet.speed * 0.514444;  // [m/s]

            CHAR date[7] = {'\0'};
            CHAR time[11] = {'\0'};
            snprintf(date, sizeof(date), "%06u", packet.date);
            snprintf(time, sizeof(time), "%010.3f", packet.utcTime);
            this->gps_utc_date.setvalue(Fw::String(date));
            this->gps_utc_time.setvalue(Fw::String(time));
            this->gps_latitude.setvalue(lat_deg);
            this->gps_latitude_NS = (std::string() + packet.lat_NS).c_str();
            this->gps_longitude.setvalue(lng_deg);
            this->gps_longitude_EW = (std::string() + packet.lng_EW).c_str();

            this->gps_speed.setvalue(speed_mps);
            this->tlmWrite_GPS_Speed(this->gps_speed);

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

            // Found all items! No need to look for another instance
            return;
        } else if (strcmp(protocol, "GGA") == 0) {
            GGAPacket packet;
            FwIndexType ret = sscanf(pointer, "GGA,%f,%f,%c,%f,%c,%u,%u,%f,%f", &packet.utcTime, &packet.latitude,
                                         &packet.lat_NS, &packet.longitude, &packet.lng_EW, &packet.fix,
                                         &packet.satsUsed, &packet.hdop, &packet.msl_altitude);
            // printf("GGA: %f %f %c %f %c %u %u %f %f\n", packet.utcTime, packet.latitude, packet.lat_NS,
            //        packet.longitude, packet.lng_EW, packet.fix, packet.satsUsed, packet.hdop, packet.msl_altitude);

            // Not all items were found, skip
            if (ret < 9) {
                protocolIndex += 1;
                continue;
            }

            this->gps_altitude.setvalue(packet.msl_altitude);
            this->tlmWrite_GPS_Altitude(this->gps_altitude);

            this->gps_sats_used = packet.satsUsed;
            this->tlmWrite_GPS_NumSatellites(this->gps_sats_used);

            this->gps_hdop = packet.hdop;
            this->tlmWrite_GPS_HDOP(this->gps_hdop);

            // Found all items! No need to look for another instance
            return;
        } else if (strcmp(protocol, "GSA") == 0) {
            U8 satCount = 0;
            for (U8 i = protocolIndex + 7; i < buf.getSize() && i + 1 < buf.getSize(); i += 3) {
                if (buf.getData()[i + 1] == ',') {
                    break;
                }
                satCount++;
                if (satCount == 12) {  // 12 is the max for this GPS
                    break;
                }
            }

            if (buf.getData()[protocolIndex - 1] == 'P') {
                this->gps_sats_used_constellations[0].setnum_satellites(satCount);
            } else if (buf.getData()[protocolIndex - 1] == 'L') {
                this->gps_sats_used_constellations[1].setnum_satellites(satCount);
            } else if (buf.getData()[protocolIndex - 1] == 'A') {
                this->gps_sats_used_constellations[2].setnum_satellites(satCount);
            } else if (buf.getData()[protocolIndex - 1] == 'B') {
                this->gps_sats_used_constellations[3].setnum_satellites(satCount);
            }
            this->tlmWrite_GPS_Constellations(this->gps_sats_used_constellations);
        }

        protocolIndex += 1;
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
    U8 reg_data[buf.getSize() + 1 + 3 + 2];
    Fw::Buffer cmdBuffer(reg_data, buf.getSize() + 1 + 3 + 2);

    cmdBuffer.getData()[0] = '$';
    memcpy(&cmdBuffer.getData()[1], buf.getData(), buf.getSize());

    cmdBuffer.getData()[cmdBuffer.getSize() - 5] = '*';

    U8 checksum = 0;
    for (U8 i = 0; i < buf.getSize(); i++) {
        checksum ^= buf.getData()[i];
    }

    CHAR hexDigits[] = "0123456789ABCDEF";
    cmdBuffer.getData()[cmdBuffer.getSize() - 4] = static_cast<U8>(hexDigits[(checksum >> 4) & 0x0F]);
    cmdBuffer.getData()[cmdBuffer.getSize() - 3] = static_cast<U8>(hexDigits[checksum & 0x0F]);

    cmdBuffer.getData()[cmdBuffer.getSize() - 2] = '\r';
    cmdBuffer.getData()[cmdBuffer.getSize() - 1] = '\n';

    Drv::I2cStatus stat = this->i2cWrite_out(0, this->m_i2cDevAddress, cmdBuffer);
    if (stat != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(stat);
    }
    return stat;
}

}  // namespace Adafruit
