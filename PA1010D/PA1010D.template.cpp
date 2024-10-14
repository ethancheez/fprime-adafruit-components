// ======================================================================
// \title  PA1010D.cpp
// \author ethan
// \brief  cpp file for PA1010D component implementation class
// ======================================================================

#include "Components/PA1010D/PA1010D.hpp"
#include "FpConfig.hpp"

namespace Sensors {

  // ----------------------------------------------------------------------
  // Component construction and destruction
  // ----------------------------------------------------------------------

  PA1010D ::
    PA1010D(const char* const compName) :
      PA1010DComponentBase(compName)
  {

  }

  PA1010D ::
    ~PA1010D()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void PA1010D ::
    run_handler(
        NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Handler implementations for commands
  // ----------------------------------------------------------------------

  void PA1010D ::
    SET_NMEA_OUTPUT_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        Sensors::GPS_NMEA_OUTPUTS nmea,
        Fw::On state
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void PA1010D ::
    SET_UPDATE_RATE_MS_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        U16 ms
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

}
