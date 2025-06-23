module Adafruit {

    struct BNO085_DataUnit_F64 {
        value: F64,
        unit: string,
    }

    struct BNO085_IMU_XYZ_F64 {
        x: BNO085_DataUnit_F64,
        y: BNO085_DataUnit_F64,
        z: BNO085_DataUnit_F64,
    }

    struct BNO085_IMU_WXYZ_F64 {
        w: BNO085_DataUnit_F64,
        x: BNO085_DataUnit_F64,
        y: BNO085_DataUnit_F64,
        z: BNO085_DataUnit_F64,
    }

    @ Component for F Prime FSW framework.
    passive component BNO085 {

        # ----------------------------------------------------------------------
        # I/O ports
        # ----------------------------------------------------------------------

        @ Port to read from I2C port
        output port i2cRead: Drv.I2c

        @ Port to write to I2C port
        output port i2cWrite: Drv.I2c

        @ Port to receive calls from the rate group
        sync input port run: Svc.Sched

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Accelerometer XYZ Telemetry
        telemetry Accel: Adafruit.BNO085_IMU_XYZ_F64

        @ Gyroscope XYZ Telemetry
        telemetry Gyro: Adafruit.BNO085_IMU_XYZ_F64
        
        @ Magnetometer XYZ Telemetry
        telemetry Mag: Adafruit.BNO085_IMU_XYZ_F64

        @ Euler Angles XYZ Telemetry
        telemetry Euler: Adafruit.BNO085_IMU_XYZ_F64

        @ Quaternion WXYZ Constants Telemetry
        telemetry Quat: Adafruit.BNO085_IMU_WXYZ_F64

        @ Linear Acceleration XYZ Telemetry
        telemetry LinearAccel: Adafruit.BNO085_IMU_XYZ_F64

        @ Gravitational Acceleration XYZ Telemetry
        telemetry Gravity: Adafruit.BNO085_IMU_XYZ_F64

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