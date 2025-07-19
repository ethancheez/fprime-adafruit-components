module Adafruit {

    struct BMP390_DataUnit_F64 {
        value: F64,
        unit: string,
    }

    @ Component for BMP390 Barometric Altimeter
    active component BMP390 {

        # ----------------------------------------------------------------------
        # I/O ports
        # ----------------------------------------------------------------------

        @ Port to read from I2C port
        output port i2cRead: Drv.I2c

        @ Port to write to I2C port
        output port i2cWrite: Drv.I2c

        @ Port to write-read from I2C
        output port i2cWriteRead: Drv.I2cWriteRead

        @ Port to receive calls from the rate group
        async input port run: Svc.Sched

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        telemetry Altitude: Adafruit.BMP390_DataUnit_F64

        telemetry TemperatureC: Adafruit.BMP390_DataUnit_F64
        
        telemetry Pressure: Adafruit.BMP390_DataUnit_F64

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        sync command SET_SEA_LEVEL_PRESSURE(sea_level_pressure: F64)

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        event I2cError(
            status: Drv.I2cStatus
        ) \
        severity warning high \
        format "BMP390 I2C error status {}"

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

    }
}