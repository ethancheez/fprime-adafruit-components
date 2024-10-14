// ======================================================================
// \title  PCA9685.cpp
// \author ethan
// \brief  cpp file for PCA9685 component implementation class
// ======================================================================

#include "Components/PCA9685/PCA9685.hpp"
#include "FpConfig.hpp"

namespace Components {

  // ----------------------------------------------------------------------
  // Component construction and destruction
  // ----------------------------------------------------------------------

  PCA9685 ::
    PCA9685(const char* const compName) :
      PCA9685ComponentBase(compName)
  {

  }

  PCA9685 ::
    ~PCA9685()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void PCA9685 ::
    run_handler(
        NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void PCA9685 ::
    setAngle_handler(
        NATIVE_INT_TYPE portNum,
        U8 channel,
        U8 angle
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Handler implementations for commands
  // ----------------------------------------------------------------------

  void PCA9685 ::
    SET_ANGLE_OFFSET_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        U8 channel,
        I8 angle_offset
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void PCA9685 ::
    SET_ANGLE_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        U8 channel,
        U8 angle
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void PCA9685 ::
    SET_PWM_RAW_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        U8 channel,
        U16 pwm
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void PCA9685 ::
    SET_PWM_RANGE_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        U8 channel,
        U16 min_pwm,
        U16 max_pwm
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void PCA9685 ::
    RESET_ALL_ANGLES_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void PCA9685 ::
    RESET_ALL_ANGLE_OFFSETS_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void PCA9685 ::
    CALIBRATE_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

}
