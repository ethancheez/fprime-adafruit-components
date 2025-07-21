module Sensors {

    array PCA9685_ANGLES = [16] U8

    port PcaAngle(
                    channel: U8 @< Channel number to set 
                    angle: U8 @< Angle to set channel
                 )

    @ Component for the PCA9685 PWM Servo Driver
    active component PCA9685 {

        # ----------------------------------------------------------------------
        # I/O ports
        # ----------------------------------------------------------------------

        @ Port to receive calls from the rate group
        sync input port run: Svc.Sched

        @ Port to read from I2C port
        output port i2cRead: Drv.I2c

        @ Port to write to I2C port
        output port i2cWrite: Drv.I2c

        @ Port to write to PolyDB
        output port setPolyDb: Svc.Poly

        @ Port to set a channel angle called by other components
        sync input port setAngle: Sensors.PcaAngle

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Telemetry for the servo angles in degrees for all 16 channels
        telemetry Servo_Angles: Sensors.PCA9685_ANGLES

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Command to set the angle offset
        sync command SET_ANGLE_OFFSET(channel: U8, angle_offset: I8)

        @ Command to set the angle, with consideration of the offset
        sync command SET_ANGLE(channel: U8, angle: U8)

        @ Command to directly set a PWM value to the channel
        sync command SET_PWM_RAW(channel: U8, pwm: U16)

        @ Command to set the minimum and maximum values of the PWM value
        sync command SET_PWM_RANGE(channel: U8, min_pwm: U16, max_pwm: U16)

        @ Reset all channel angles
        sync command RESET_ALL_ANGLES()

        @ Reset all channel angle offsets
        sync command RESET_ALL_ANGLE_OFFSETS()

        @ Performs a series of motions to verify servos
        async command CALIBRATE()

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Invalid value when setting angle of channel
        event SetAngleInvalidRange(
            angle: U8   @< the angle attempted to be set
        ) \
        severity warning high \
        format "Angle of {} is not within the range [0, 180]"

        @ Invalid value when setting angle offset of channel
        event SetAngleOffsetInvalidRange(
            angle: I8   @< the angle offset attempted to be set
        ) \
        severity warning high \
        format "Angle offset of {} is not within the range [-20, 20]"

        @ Encountered an I2C error in reading or writing
        event I2cError(
            status: Drv.I2cStatus   @< the status value returned
        ) \
        severity warning high \
        format "PCA9685 I2C error status {}"

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