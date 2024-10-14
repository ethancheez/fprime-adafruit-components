// ======================================================================
// \title  PA1010D.hpp
// \author ethan
// \brief  hpp file for PA1010D component implementation class
// ======================================================================

#ifndef Sensors_PA1010D_HPP
#define Sensors_PA1010D_HPP

#include "Components/PA1010D/PA1010DComponentAc.hpp"

namespace Sensors {

  class PA1010D :
    public PA1010DComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Component construction and destruction
      // ----------------------------------------------------------------------

      //! Construct PA1010D object
      PA1010D(
          const char* const compName //!< The component name
      );

      //! Destroy PA1010D object
      ~PA1010D();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for run
      //!
      //! Port to receive calls from the rate group
      void run_handler(
          NATIVE_INT_TYPE portNum, //!< The port number
          NATIVE_UINT_TYPE context //!< The call order
      ) override;

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for commands
      // ----------------------------------------------------------------------

      //! Handler implementation for command SET_NMEA_OUTPUT
      //!
      //! Command to enable/disable NMEA outputs
      void SET_NMEA_OUTPUT_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq, //!< The command sequence number
          Sensors::GPS_NMEA_OUTPUTS nmea,
          Fw::On state
      ) override;

      //! Handler implementation for command SET_UPDATE_RATE_MS
      //!
      //! Command to change the update rate of the GPS
      void SET_UPDATE_RATE_MS_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq, //!< The command sequence number
          U16 ms
      ) override;

  };

}

#endif
