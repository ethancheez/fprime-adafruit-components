#ifndef RF95_REGISTERS_HPP
#define RF95_REGISTERS_HPP

// This is the address that indicates a broadcast
#define RF95_BROADCAST_ADDRESS 0xFF

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RF95_NUM_INTERRUPTS 3

// This is the bit in the SPI address that marks it as a write
#define RF95_SPI_WRITE_MASK 0x80

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RF95_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define RF95_MAX_PAYLOAD_LEN RF95_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RF95_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RF95_MAX_MESSAGE_LEN
 #define RF95_MAX_MESSAGE_LEN (RF95_MAX_PAYLOAD_LEN - RF95_HEADER_LEN)
#endif

// The crystal oscillator frequency of the module
#define RF95_FXOSC 32000000.0

// The Frequency Synthesizer step = RF95_FXOSC / 2^^19
#define RF95_FSTEP  (RF95_FXOSC / 524288)


// Register names (LoRa Mode, from table 85)
#define RF95_REG_00_FIFO                                0x00
#define RF95_REG_01_OP_MODE                             0x01
#define RF95_REG_02_RESERVED                            0x02
#define RF95_REG_03_RESERVED                            0x03
#define RF95_REG_04_RESERVED                            0x04
#define RF95_REG_05_RESERVED                            0x05
#define RF95_REG_06_FRF_MSB                             0x06
#define RF95_REG_07_FRF_MID                             0x07
#define RF95_REG_08_FRF_LSB                             0x08
#define RF95_REG_09_PA_CONFIG                           0x09
#define RF95_REG_0A_PA_RAMP                             0x0a
#define RF95_REG_0B_OCP                                 0x0b
#define RF95_REG_0C_LNA                                 0x0c
#define RF95_REG_0D_FIFO_ADDR_PTR                       0x0d
#define RF95_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RF95_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define RF95_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RF95_REG_12_IRQ_FLAGS                           0x12
#define RF95_REG_13_RX_NB_BYTES                         0x13
#define RF95_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define RF95_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define RF95_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define RF95_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define RF95_REG_18_MODEM_STAT                          0x18
#define RF95_REG_19_PKT_SNR_VALUE                       0x19
#define RF95_REG_1A_PKT_RSSI_VALUE                      0x1a
#define RF95_REG_1B_RSSI_VALUE                          0x1b
#define RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RF95_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define RF95_REG_20_PREAMBLE_MSB                        0x20
#define RF95_REG_21_PREAMBLE_LSB                        0x21
#define RF95_REG_22_PAYLOAD_LENGTH                      0x22
#define RF95_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define RF95_REG_24_HOP_PERIOD                          0x24
#define RF95_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define RF95_REG_26_MODEM_CONFIG3                       0x26

#define RF95_REG_27_PPM_CORRECTION                      0x27
#define RF95_REG_28_FEI_MSB                             0x28
#define RF95_REG_29_FEI_MID                             0x29
#define RF95_REG_2A_FEI_LSB                             0x2a
#define RF95_REG_2C_RSSI_WIDEBAND                       0x2c
#define RF95_REG_31_DETECT_OPTIMIZE                     0x31
#define RF95_REG_33_INVERT_IQ                           0x33
#define RF95_REG_37_DETECTION_THRESHOLD                 0x37
#define RF95_REG_39_SYNC_WORD                           0x39

#define RF95_REG_40_DIO_MAPPING1                        0x40
#define RF95_REG_41_DIO_MAPPING2                        0x41
#define RF95_REG_42_VERSION                             0x42

#define RF95_REG_4B_TCXO                                0x4b
#define RF95_REG_4D_PA_DAC                              0x4d
#define RF95_REG_5B_FORMER_TEMP                         0x5b
#define RF95_REG_61_AGC_REF                             0x61
#define RF95_REG_62_AGC_THRESH1                         0x62
#define RF95_REG_63_AGC_THRESH2                         0x63
#define RF95_REG_64_AGC_THRESH3                         0x64

// RF95_REG_01_OP_MODE                             0x01
#define RF95_LONG_RANGE_MODE                       0x80
#define RF95_ACCESS_SHARED_REG                     0x40
#define RF95_LOW_FREQUENCY_MODE                    0x08
#define RF95_MODE                                  0x07
#define RF95_MODE_SLEEP                            0x00
#define RF95_MODE_STDBY                            0x01
#define RF95_MODE_FSTX                             0x02
#define RF95_MODE_TX                               0x03
#define RF95_MODE_FSRX                             0x04
#define RF95_MODE_RXCONTINUOUS                     0x05
#define RF95_MODE_RXSINGLE                         0x06
#define RF95_MODE_CAD                              0x07

// RF95_REG_09_PA_CONFIG                           0x09
#define RF95_PA_SELECT                             0x80
#define RF95_MAX_POWER                             0x70
#define RF95_OUTPUT_POWER                          0x0f

// RF95_REG_0A_PA_RAMP                             0x0a
#define RF95_LOW_PN_TX_PLL_OFF                     0x10
#define RF95_PA_RAMP                               0x0f
#define RF95_PA_RAMP_3_4MS                         0x00
#define RF95_PA_RAMP_2MS                           0x01
#define RF95_PA_RAMP_1MS                           0x02
#define RF95_PA_RAMP_500US                         0x03
#define RF95_PA_RAMP_250US                         0x04
#define RF95_PA_RAMP_125US                         0x05
#define RF95_PA_RAMP_100US                         0x06
#define RF95_PA_RAMP_62US                          0x07
#define RF95_PA_RAMP_50US                          0x08
#define RF95_PA_RAMP_40US                          0x09
#define RF95_PA_RAMP_31US                          0x0a
#define RF95_PA_RAMP_25US                          0x0b
#define RF95_PA_RAMP_20US                          0x0c
#define RF95_PA_RAMP_15US                          0x0d
#define RF95_PA_RAMP_12US                          0x0e
#define RF95_PA_RAMP_10US                          0x0f

// RF95_REG_0B_OCP                                 0x0b
#define RF95_OCP_ON                                0x20
#define RF95_OCP_TRIM                              0x1f

// RF95_REG_0C_LNA                                 0x0c
#define RF95_LNA_GAIN                              0xe0
#define RF95_LNA_GAIN_G1                           0x20
#define RF95_LNA_GAIN_G2                           0x40
#define RF95_LNA_GAIN_G3                           0x60                
#define RF95_LNA_GAIN_G4                           0x80
#define RF95_LNA_GAIN_G5                           0xa0
#define RF95_LNA_GAIN_G6                           0xc0
#define RF95_LNA_BOOST_LF                          0x18
#define RF95_LNA_BOOST_LF_DEFAULT                  0x00
#define RF95_LNA_BOOST_HF                          0x03
#define RF95_LNA_BOOST_HF_DEFAULT                  0x00
#define RF95_LNA_BOOST_HF_150PC                    0x03

// RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RF95_RX_TIMEOUT_MASK                       0x80
#define RF95_RX_DONE_MASK                          0x40
#define RF95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RF95_VALID_HEADER_MASK                     0x10
#define RF95_TX_DONE_MASK                          0x08
#define RF95_CAD_DONE_MASK                         0x04
#define RF95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RF95_CAD_DETECTED_MASK                     0x01

// RF95_REG_12_IRQ_FLAGS                           0x12
#define RF95_RX_TIMEOUT                            0x80
#define RF95_RX_DONE                               0x40
#define RF95_PAYLOAD_CRC_ERROR                     0x20
#define RF95_VALID_HEADER                          0x10
#define RF95_TX_DONE                               0x08
#define RF95_CAD_DONE                              0x04
#define RF95_FHSS_CHANGE_CHANNEL                   0x02
#define RF95_CAD_DETECTED                          0x01

// RF95_REG_18_MODEM_STAT                          0x18
#define RF95_RX_CODING_RATE                        0xe0
#define RF95_MODEM_STATUS_CLEAR                    0x10
#define RF95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RF95_MODEM_STATUS_RX_ONGOING               0x04
#define RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RF95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RF95_PLL_TIMEOUT                           0x80
#define RF95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RF95_FHSS_PRESENT_CHANNEL                  0x3f

// RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RF95_BW                                    0xf0

#define RF95_BW_7_8KHZ                             0x00
#define RF95_BW_10_4KHZ                            0x10
#define RF95_BW_15_6KHZ                            0x20
#define RF95_BW_20_8KHZ                            0x30
#define RF95_BW_31_25KHZ                           0x40
#define RF95_BW_41_7KHZ                            0x50
#define RF95_BW_62_5KHZ                            0x60
#define RF95_BW_125KHZ                             0x70
#define RF95_BW_250KHZ                             0x80
#define RF95_BW_500KHZ                             0x90
#define RF95_CODING_RATE                           0x0e
#define RF95_CODING_RATE_4_5                       0x02
#define RF95_CODING_RATE_4_6                       0x04
#define RF95_CODING_RATE_4_7                       0x06
#define RF95_CODING_RATE_4_8                       0x08
#define RF95_IMPLICIT_HEADER_MODE_ON               0x01

// RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RF95_SPREADING_FACTOR                      0xf0
#define RF95_SPREADING_FACTOR_64CPS                0x60
#define RF95_SPREADING_FACTOR_128CPS               0x70
#define RF95_SPREADING_FACTOR_256CPS               0x80
#define RF95_SPREADING_FACTOR_512CPS               0x90
#define RF95_SPREADING_FACTOR_1024CPS              0xa0
#define RF95_SPREADING_FACTOR_2048CPS              0xb0
#define RF95_SPREADING_FACTOR_4096CPS              0xc0
#define RF95_TX_CONTINUOUS_MODE                    0x08

#define RF95_PAYLOAD_CRC_ON                        0x04
#define RF95_SYM_TIMEOUT_MSB                       0x03

// RF95_REG_26_MODEM_CONFIG3
#define RF95_MOBILE_NODE                           0x08 // HopeRF term
#define RF95_LOW_DATA_RATE_OPTIMIZE                0x08 // Semtechs term
#define RF95_AGC_AUTO_ON                           0x04

// RF95_REG_4B_TCXO                                0x4b
#define RF95_TCXO_TCXO_INPUT_ON                    0x10

// RF95_REG_4D_PA_DAC                              0x4d
#define RF95_PA_DAC_DISABLE                        0x04
#define RF95_PA_DAC_ENABLE                         0x07
 
#endif // RF95_REGISTERS_HPP
