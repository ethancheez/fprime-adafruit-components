#ifndef Adafruit_MPL3115A2_REGISTERS_HPP
#define Adafruit_MPL3115A2_REGISTERS_HPP

#define MPL3115A2_STATUS            0x00
#define MPL3115A2_OUT_P_MSB         0x01
#define MPL3115A2_OUT_P_CSB         0x02
#define MPL3115A2_OUT_P_LSB         0x03
#define MPL3115A2_OUT_T_MSB         0x04
#define MPL3115A2_OUT_T_LSB         0x05
#define MPL3115A2_DR_STATUS         0x06
#define MPL3115A2_OUT_P_DELTA_MSB   0x07
#define MPL3115A2_OUT_P_DELTA_CSB   0x08
#define MPL3115A2_OUT_P_DELTA_LSB   0x09
#define MPL3115A2_OUT_T_DELTA_MSB   0x0A
#define MPL3115A2_OUT_T_DELTA_LSB   0x0B
#define MPL3115A2_WHO_AM_I          0x0C
#define MPL3115A2_F_STATUS          0x0D
#define MPL3115A2_F_DATA            0x0E
#define MPL3115A2_F_SETUP           0x0F
#define MPL3115A2_TIME_DLY          0x10
#define MPL3115A2_SYSMOD            0x11
#define MPL3115A2_INT_SOURCE        0x12
#define MPL3115A2_PT_DATA_CFG       0x13
#define MPL3115A2_BAR_IN_MSB        0x14
#define MPL3115A2_BAR_IN_LSB        0x15
#define MPL3115A2_P_TGT_MSB         0x16
#define MPL3115A2_P_TGT_LSB         0x17
#define MPL3115A2_T_TGT             0x18
#define MPL3115A2_P_WND_MSB         0x19
#define MPL3115A2_P_WND_LSB         0x1A
#define MPL3115A2_T_WND             0x1B
#define MPL3115A2_P_MIN_MSB         0x1C
#define MPL3115A2_P_MIN_CSB         0x1D
#define MPL3115A2_P_MIN_LSB         0x1E
#define MPL3115A2_T_MIN_MSB         0x1F
#define MPL3115A2_T_MIN_LSB         0x20
#define MPL3115A2_P_MAX_MSB         0x21
#define MPL3115A2_P_MAX_CSB         0x22
#define MPL3115A2_P_MAX_LSB         0x23
#define MPL3115A2_T_MAX_MSB         0x24
#define MPL3115A2_T_MAX_LSB         0x25
#define MPL3115A2_CTRL_REG1         0x26
#define MPL3115A2_CTRL_REG2         0x27
#define MPL3115A2_CTRL_REG3         0x28
#define MPL3115A2_CTRL_REG4         0x29
#define MPL3115A2_CTRL_REG5         0x2A
#define MPL3115A2_OFF_P             0x2B
#define MPL3115A2_OFF_T             0x2C
#define MPL3115A2_OFF_H             0x2D

// PT DATA register bits
#define MPL3115A2_PT_DATA_CFG_TDEFE 0x01
#define MPL3115A2_PT_DATA_CFG_PDEFE 0x02
#define MPL3115A2_PT_DATA_CFG_DREM  0x04

//  DR_STATUS flags
#define MPL3115A2_TDR_F             0x02    // temp data ready
#define MPL3115A2_PDR_F             0x04    // pressure/altitude data ready
#define MPL3115A2_PTDR_F            0x08    // P/A *or* Temp data ready
#define MPL3115A2_TOW_F             0x20    // Temp data was overwritten
#define MPL3115A2_POW_F             0x40    // Press/Altitude data was overwritten
#define MPL3115A2_PTOW_F            0x80    // P/A *or* temp data was overwritten

//  Control Register 1 flags
#define MPL3115A2_ACTV_F            0x01    // 0==>standby, 1==>active
#define MPL3115A2_OST_F             0x02    // -->1 triggers sampling
#define MPL3115A2_RST_F             0x04    // -->1 triggers reset; 0==> reset complete
#define MPL3115A2_OS_F              0x38    // 3-bit field for OverSample rate
#define MPL3115A2_RAW_F             0x40    // sample in raw mode
#define MPL3115A2_ALT_F             0x80    // 1==>altimeter mode; 0==>barometer mode

#endif  // Adafruit_MPL3115A2_REGISTERS_HPP
