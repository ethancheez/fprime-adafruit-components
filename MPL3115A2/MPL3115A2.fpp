module Sensors {

    struct MPL3115A2_DataUnit_F64 {
        value: F64,
        unit: string,
    }

    @ Component for the MPL3115A2 Altimeter
    passive component MPL3115A2 {

        # ----------------------------------------------------------------------
        # I/O ports
        # ----------------------------------------------------------------------

        @ Port to receive calls from the rate group
        sync input port run: Svc.Sched

        @ Port to read from I2C port
        output port i2cRead: Drv.I2c

        @ Port to write to I2C port
        output port i2cWrite: Drv.I2c

        @ Port to write to I2C port
        output port i2cWriteRead: Drv.I2cWriteRead

        @ Port to write to PolyDB
        output port setPolyDb: Svc.Poly

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Telemetry for the current altitude in meters
        telemetry Altitude: Sensors.MPL3115A2_DataUnit_F64

        @ Telemetry for the current temperature in C
        telemetry TemperatureC: Sensors.MPL3115A2_DataUnit_F64

        @ Telemetry for the current air pressure in kPa
        telemetry Pressure: Sensors.MPL3115A2_DataUnit_F64

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Command to set the sea level pressure in hPa
        sync command SET_SEA_LEVEL_PRESSURE(sea_level_pressure: F64)

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Encountered an I2C error in reading or writing
        event I2cError(
            status: Drv.I2cStatus   @< the status value returned
        ) \
        severity warning high \
        format "MPL3115A2 I2C error status {}"

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

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

    }
}