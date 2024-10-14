module Adafruit {

    struct BNO055_DataUnit_F64 {
        value: F64,
        unit: string,
    }

    struct IMU_XYZ_F64 {
        x: BNO055_DataUnit_F64,
        y: BNO055_DataUnit_F64,
        z: BNO055_DataUnit_F64,
    }

    struct IMU_WXYZ_F64 {
        w: BNO055_DataUnit_F64,
        x: BNO055_DataUnit_F64,
        y: BNO055_DataUnit_F64,
        z: BNO055_DataUnit_F64,
    }

    @ Component for the BNO055 IMU Module
    passive component BNO055 {

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

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Accelerometer XYZ Telemetry
        telemetry Accel: Adafruit.IMU_XYZ_F64

        @ Gyroscope XYZ Telemetry
        telemetry Gyro: Adafruit.IMU_XYZ_F64
        
        @ Magnetometer XYZ Telemetry
        telemetry Mag: Adafruit.IMU_XYZ_F64

        @ Euler XYZ Angles Telemetry
        telemetry Euler: Adafruit.IMU_XYZ_F64

        @ Quaternion WXYZ Constants Telemetry
        telemetry Quat: Adafruit.IMU_WXYZ_F64

        @ Linear Acceleration XYZ Telemetry
        telemetry LinearAccel: Adafruit.IMU_XYZ_F64

        @ Gravitational Acceleration XYZ Telemetry
        telemetry Gravity: Adafruit.IMU_XYZ_F64

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Error occurred when reading IMU data
        event ImuUpdateError(
            data: string            @< the data field that failed to update
            status: Drv.I2cStatus   @< the status value returned
        ) \
        severity warning high \
        format "{} update failed with status {}"

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

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

    }
}