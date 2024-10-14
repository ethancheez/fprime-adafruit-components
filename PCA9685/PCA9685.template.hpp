// ======================================================================
// \title  PCA9685.hpp
// \author ethan
// \brief  hpp file for PCA9685 component implementation class
// ======================================================================

#ifndef Components_PCA9685_HPP
#define Components_PCA9685_HPP

#include "Components/PCA9685/PCA9685ComponentAc.hpp"

namespace Components {

  class PCA9685 :
    public PCA9685ComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Component construction and destruction
      // ----------------------------------------------------------------------

      //! Construct PCA9685 object
      PCA9685(
          const char* const compName //!< The component name
      );

      //! Destroy PCA9685 object
      ~PCA9685();

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

      //! Handler implementation for setAngle
      //!
      //! Port to set a channel angle called by other components
      void setAngle_handler(
          NATIVE_INT_TYPE portNum, //!< The port number
          U8 channel, //!< Channel number to set
          U8 angle //!< Angle to set channel
      ) override;

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for commands
      // ----------------------------------------------------------------------

      //! Handler implementation for command SET_ANGLE_OFFSET
      //!
      //! Command to set the angle offset
      void SET_ANGLE_OFFSET_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq, //!< The command sequence number
          U8 channel,
          I8 angle_offset
      ) override;

      //! Handler implementation for command SET_ANGLE
      //!
      //! Command to set the angle, with consideration of the offset
      void SET_ANGLE_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq, //!< The command sequence number
          U8 channel,
          U8 angle
      ) override;

      //! Handler implementation for command SET_PWM_RAW
      //!
      //! Command to directly set a PWM value to the channel
      void SET_PWM_RAW_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq, //!< The command sequence number
          U8 channel,
          U16 pwm
      ) override;

      //! Handler implementation for command SET_PWM_RANGE
      //!
      //! Command to set the minimum and maximum values of the PWM value
      void SET_PWM_RANGE_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq, //!< The command sequence number
          U8 channel,
          U16 min_pwm,
          U16 max_pwm
      ) override;

      //! Handler implementation for command RESET_ALL_ANGLES
      //!
      //! Reset all channel angles
      void RESET_ALL_ANGLES_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq //!< The command sequence number
      ) override;

      //! Handler implementation for command RESET_ALL_ANGLE_OFFSETS
      //!
      //! Reset all channel angle offsets
      void RESET_ALL_ANGLE_OFFSETS_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq //!< The command sequence number
      ) override;

      //! Handler implementation for command CALIBRATE
      //!
      //! Performs a series of motions to verify servos
      void CALIBRATE_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq //!< The command sequence number
      ) override;

  };

}

#endif
