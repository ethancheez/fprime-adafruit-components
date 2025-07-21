module Radios {
    @ Component for RFM95 Radio Module
    active component RFM95 {

        import Drv.ByteStreamDriver

        # ----------------------------------------------------------------------
        # Implementation ports
        # ----------------------------------------------------------------------

        @ Allows for deallocation of radio command communications
        output port deallocate: Fw.BufferSend

        @ Allows for allocation of buffers
        output port allocate: Fw.BufferGet

        # ----------------------------------------------------------------------
        # I/O Ports
        # ----------------------------------------------------------------------

        @ Port receiving calls from the rate group
        async input port run: Svc.Sched

        @ SPI port for sending and receiving data
        output port spiReadWrite: Drv.SpiReadWrite

        @ Port sending calls to the GPIO driver
        output port radioReset: Drv.GpioWrite

        @ Port sending calls to the GPIO driver
        output port radioChipSelect: Drv.GpioWrite

        @ Port sending calls to the GPIO driver
        sync input port radioInterrupt: Svc.Cycle

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Telemetry packets sent for RFM69
        telemetry PacketsSent: U32

        @ Telemetry packets received for RFM69
        telemetry PacketsReceived: U32

        @ Telemetry for number of bad packets received
        telemetry BadPackets: U32

        @ Telemetry for transmitting power level
        telemetry TxPowerLevel: I8

        @ Telemetry for last RSSI value
        telemetry RSSI: I16

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Event for timeout waiting for packet sent
        event WaitPacketSentTimeout() \
        severity warning low \
        format "Timeout waiting for packet sent"

        @ Event for a bad packet received
        event BadPacketReceived() \
        severity warning low \
        format "Bad packet received"

        @ Event for a origin and destination ID report
        event OriginDestinationIdReport(
            originId: U8,
            destinationId: U8
        ) \
        severity activity high \
        format "Origin ID: {}, Destination ID: {}"

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Command to set the radio transmit power
        sync command SET_TX_POWER(power: I8)

        @ Command to report the radio's origin and destination IDs
        sync command REPORT_ORIGIN_DESTINATION_ID() \

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------

        @ Parameter for the radio's header origin ID
        param ORIGIN_ID: U8 default 0xFF

        @ Parameter for the radio's header destination ID
        param DESTINATION_ID: U8 default 0xFF

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

        @ Port to return the value of a parameter
        param get port prmGetOut

        @Port to set the value of a parameter
        param set port prmSetOut

    }
}