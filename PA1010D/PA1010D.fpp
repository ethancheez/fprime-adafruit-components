module Sensors {

    struct GPS_DataUnit {
        value: string,
        unit: string,
    }

    struct GPS_DataUnit_F64 {
        value: F64,
        unit: string,
    }

    struct GPS_Time {
        utc_date: GPS_DataUnit,
        utc_time: GPS_DataUnit,
    }

    struct GPS_Location {
        latitude: GPS_DataUnit_F64,
        lat_NS: string,
        longitude: GPS_DataUnit_F64,
        lng_EW: string,
    }

    struct GPS_Constellations_Sats {
        constellation: string,
        num_satellites: U8,
    }

    enum GPS_NMEA_OUTPUTS: U8 {
        GLL = 0,
        RMC = 1,
        VTG = 2,
        GGA = 3,
        GSA = 4,
        GSV = 5,
    }

    struct GPS_NMEA_State {
        protocol: GPS_NMEA_OUTPUTS,
        $state: Fw.On,
    }

    array GPS_NMEA_States = [6] GPS_NMEA_State

    array GPS_Constellations_NumSatellites = [4] GPS_Constellations_Sats

    @ Component for PA1010D GPS Module
    active component PA1010D {

        # ----------------------------------------------------------------------
        # I/O ports
        # ----------------------------------------------------------------------

        @ Port to receive calls from the rate group
        async input port run: Svc.Sched

        @ Port to read from I2C port
        output port i2cRead: Drv.I2c

        @ Port to write to I2C port
        output port i2cWrite: Drv.I2c

        @ Port to write to PolyDB
        output port setPolyDb: Svc.Poly

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Telemetry for number of satellites
        telemetry GPS_NumSatellites: I32

        @ Telemetry for the number of satellites used for each constellation
        telemetry GPS_Constellations: GPS_Constellations_NumSatellites

        @ Telemetry for Horizontal Dilution of Precision
        telemetry GPS_HDOP: F64

        @ Telemetry for latitude and longitude
        telemetry GPS_Location: Sensors.GPS_Location

        @ Telemetry for date and time in UTC
        telemetry GPS_Time: Sensors.GPS_Time

        @ Telemetry for ground speed
        telemetry GPS_Speed: Sensors.GPS_DataUnit_F64

        @ Telemetry for altitude
        telemetry GPS_Altitude: Sensors.GPS_DataUnit_F64

        @ Telemetry for the enabled/disabled NMEA states
        telemetry NMEA_States: Sensors.GPS_NMEA_States

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Command to enable/disable NMEA outputs
        sync command SET_NMEA_OUTPUT(nmea: Sensors.GPS_NMEA_OUTPUTS, $state: Fw.On)

        @ Command to enable/disable constellation use
        sync command ENABLE_CONSTELLATIONS(
            GPS: bool,
            GLONASS: bool,
            GALILEO: bool,
            BEIDOU: bool
        )

        @ Command to change the update rate of the GPS
        sync command SET_UPDATE_RATE_MS(ms: U16)

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Encountered an I2C error in reading or writing
        event I2cError(
            status: Drv.I2cStatus   @< the status value returned
        ) \
        severity warning high \
        format "PA1010D I2C error status {}"

        @ Error occurred when saving value to PolyDB
        event PolyDbSetError(
            status: U32   @< the status value returned
        ) \
        severity warning high \
        format "PolyDB set failed with status {}"

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

    }
}