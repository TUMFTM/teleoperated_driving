// Copyright
/****************************************************************************
 *                                      ###################################
 *                                     #########                    ########
 *                                      ##################          ########
 *                                                #########         ########
 *                                     ###################   ##############
 *                                                www.sensodrive.de
 *
 *  File:           sensowheel.h
 *  Project:        SENSO-Wheel
 *  Originator:     Renate Rueckel
 *  Copyright:      SENSODRIVE GmbH, Wessling
 *
 *      $LastChangedDate: 2012-09-07 16:39:05 +0200 (Fr, 07 Sep 2012) $
 *        $LastChangedBy: geist $
 *  $LastChangedRevision: 58 $
 *
 ****************************************************************************
 *  $Id: sensowheel.h 58 2012-09-07 14:39:05Z geist $
 ****************************************************************************
 *
 *  SENSO-Wheel constants to be included to your simulator software
 *
 *  This file contains code for firmware version 3.06 and later!
 *  For a detailed description of functionality of firmware please refer to
 *  its release notes.
 *
 ***************************************************************************/

#ifndef SENSOWHEEL_H_
#define SENSOWHEEL_H_

/* DEFINES */
/***********/

/* SENSO-Wheel PARAMETERS */
// initial values
#define SWH_INIT_ENDSTOPPOS             540
#define SWH_INIT_POSOFFSET              0
#define SWH_INIT_TORQUE                 0
#define SWH_INIT_DAMPING                0
#define SWH_INIT_FRICTION               0
#define SWH_INIT_STIFFNESS              0
#define SWH_INIT_TORQUE_LIMITATION      100
#define SWH_INIT_PEAKTORQUE_LIMITATION  100
#define SWH_INIT_SYSPAR_NOMINALTORQUE   7500    // initialized for systems before firmware version 3.x
#define SWH_INIT_SYSPAR_MAXIMUMTORQUE   16580   // initialized for systems before firmware version 3.x
#define SWH_INIT_SYSPAR_INCSPERREV      8000    // initialized for systems before firmware version 1.34

// limit values
// Control Module
#define SWH_ENDSTOPPOS_MIN              10      // <10 to switch off
#define SWH_ENDSTOPPOS_MAX              1440
#define SWH_POSOFFSET_LIMIT             359     // +/-
#define SWH_TORQUE_LIMITATION_MIN       0       // torque limitation in [%]
#define SWH_TORQUE_LIMITATION_MAX       255
#define SWH_PEAKTORQUE_LIMITATION_MIN   0       // peak torque limitation in [%]
#define SWH_PEAKTORQUE_LIMITATION_MAX   100
// I/O, 3 digital outputs
#define SWH_DIGOUT_MIN                  0
#define SWH_DIGOUT_MAX                  7
// Normal Mode
#define SWH_FRICTION_MIN                0
#define SWH_FRICTION_MAX                5000
#define SWH_DAMPING_MIN                 0
#define SWH_DAMPING_MAX                 500
#define SWH_STIFFNESS_MIN               0
#define SWH_STIFFNESS_MAX               2500


/* SENSO-Wheel CAN identifier */
// The direction is defined as follows:
// "receive": The host software sends, the SENSO-Wheel receives
// "send": The SENSO-Wheel sends, the host software receives
#define SWH_CAN_ID_RECV_UPDATE          0x010   // reserved for Firmware Update
#define SWH_CAN_ID_RECV_CONTROL         0x200   // SENSO-Wheel receive message for control module
#define SWH_CAN_ID_SEND_CONTROL         0x210   // SENSO-Wheel send message with status information
#define SWH_CAN_ID_RECV_MODENORMAL      0x201   // receive message for normal mode
#define SWH_CAN_ID_SEND_MODENORMAL      0x211   // send message with actual values in normal mode
#define SWH_CAN_ID_RECV_MODEBASIC       0x202   // receive message for basic mode
#define SWH_CAN_ID_SEND_MODEBASIC       0x212   // send message with actual position in basic mode
#define SWH_CAN_ID_RECV_MODEDEMO        0x204   // reserved for Demonstration Mode
#define SWH_CAN_ID_SEND_MODEDEMO        0x214   // reserved for Demonstration Mode
#define SWH_CAN_ID_RECV_ADIO            0x20A   // receive message for dig. outputs, dig. inputs
#define SWH_CAN_ID_SEND_ADIO            0x21A   // send message with ana. and dig. outputs
#define SWH_CAN_ID_RECV_ABSPOS          0x20B   // receive message for absolute position
#define SWH_CAN_ID_SEND_ABSPOS          0x21B   // send message with confirmation of absolute position
#define SWH_CAN_ID_RECV_PEDAL           0x20C   // send Pedals
#define SWH_CAN_ID_SEND_PEDAL           0x21C   // receive Pedals
#define SWH_CAN_ID_RECV_ROVALUES        0x20D   // receive message for read-only values
#define SWH_CAN_ID_SEND_ROVALUES        0x21D   // send message with read-only values
#define SWH_CAN_ID_RECV_EXTVERSION      0x20E   // receive message for extended version
#define SWH_CAN_ID_SEND_EXTVERSION      0x21E   // send message with extended version
#define SWH_CAN_ID_RECV_FVERSION        0x20F   // receive message for version of firmware (deprecated)
#define SWH_CAN_ID_SEND_FVERSION        0x21F   // send message with version of firmware (deprecated)
#define SWH_CAN_ID_RECV_USERPARAMS      0x221   // receive message for user parameters
#define SWH_CAN_ID_SEND_USERPARAMS      0x231   // send message with user parameters
#define SWH_CAN_ID_RECV_EFFPARAMS       0x22A   // receive message for optional parameters for SENSO-Wheel effects
#define SWH_CAN_ID_SEND_EFFPARAMS       0x23A   // send message with confirmation of optional parameters
#define SWH_CAN_ID_RECV_SYSPARAMS       0x22D   // receive message for system parameters
#define SWH_CAN_ID_SEND_SYSPARAMS       0x23D   // send message with system parameters

/* SENSO-Wheel length of CAN message */
#define SWH_CAN_DLC_RECV_CONTROL        8
#define SWH_CAN_DLC_SEND_CONTROL        8
#define SWH_CAN_DLC_RECV_MODENORMAL     8
#define SWH_CAN_DLC_SEND_MODENORMAL     8
#define SWH_CAN_DLC_RECV_MODEBASIC      2
#define SWH_CAN_DLC_SEND_PEDALS         1
#define SWH_CAN_DLC_RECV_PEDALS         1
#define SWH_CAN_DLC_SEND_MODEBASIC      4
#define SWH_CAN_DLC_RECV_MODEDEMO       8 // reserved for Demonstration Mode
#define SWH_CAN_DLC_SEND_MODEDEMO       8 // reserved for Demonstration Mode
#define SWH_CAN_DLC_RECV_ADIO           2
#define SWH_CAN_DLC_SEND_ADIO           6
#define SWH_CAN_DLC_RECV_ABSPOS         2
#define SWH_CAN_DLC_SEND_ABSPOS         2
#define SWH_CAN_DLC_RECV_ROVALUES       1
#define SWH_CAN_DLC_SEND_ROVALUES       8
#define SWH_CAN_DLC_RECV_EXTVERSION     1
#define SWH_CAN_DLC_SEND_EXTVERSION     8
#define SWH_CAN_DLC_RECV_FVERSION       0
#define SWH_CAN_DLC_SEND_FVERSION       8
#define SWH_CAN_DLC_RECV_USERPARAMS_W   8 // to write user parameters
#define SWH_CAN_DLC_RECV_USERPARAMS_R   1 // to read  user parameters
#define SWH_CAN_DLC_SEND_USERPARAMS     6
#define SWH_CAN_DLC_RECV_EFFPARAMS_W    8 // to write optional parameters for SENSO-Wheel effects
#define SWH_CAN_DLC_RECV_EFFPARAMS_R    1 // to read optional parameters for SENSO-Wheel effects
#define SWH_CAN_DLC_SEND_EFFPARAMS      8
#define SWH_CAN_DLC_RECV_SYSPARAMS      1
#define SWH_CAN_DLC_SEND_SYSPARAMS      8

/* SENSO-Wheel control word */
// transitions of state machine
#define SWH_CTRL_SWITCH_OFF             0x0000  // Switch to STATE OFF
#define SWH_CTRL_SWITCH_READY           0x0002  // Switch to STATE READY
#define SWH_CTRL_SWITCH_ON              0x0004  // Switch to STATE ON
#define SWH_CTRL_SWITCH_QUITERROR       0x000F  // Quit ERROR
#define SWH_CTRL_SWITCH_MASK            0x000F  // Mask
// modes of operation
#define SWH_CTRL_MODE_NORMAL            0x0010  // NORMAL MODE
#define SWH_CTRL_MODE_BASIC             0x0020  // BASIC MODE
#define SWH_CTRL_MODE_REF               0x0040  // REFERENCE TOUR
#define SWH_CTRL_MODE_DEMO              0x0080  // DEMONSTRATION
#define SWH_CTRL_MODE_MASK              0x00F0  // Mask
// auxiliary functions
#define SWH_CTRL_AUX_CAN_WATCHDOG_OFF   0x0100  // Watchdog OFF (SAVETY WARNING: FORBIDDEN DURING NORMAL OPERATION!)
#define SWH_CTRL_AUX_VIBRATION_ON       0x0200  // Vibration beyond end stops ON (Normal Mode only)
#define SWH_CTRL_AUX_FRICTION_2         0x0400  // Switch to version-2-friction (Normal Mode only)
#define SWH_CTRL_AUX_MASK               0xFF00  // Mask

/* SENSO-Wheel status codes */
// states
#define SWH_STATUS_STATE_OFF            0x0000  // STATE OFF
#define SWH_STATUS_STATE_READY          0x0002  // STATE READY
#define SWH_STATUS_STATE_ON             0x0004  // STATE ON -> Motor activated
#define SWH_STATUS_STATE_ERROR          0x0008  // STATE ERROR
#define SWH_STATUS_STATE_MASK           0x000F  // Mask
// operating modes
#define SWH_STATUS_MODE_NORMAL          0x0010  // Steering Normal Mode
#define SWH_STATUS_MODE_BASIC           0x0020  // Steering Basic Functions
#define SWH_STATUS_MODE_REF             0x0040  // Reference Tour
#define SWH_STATUS_MODE_DEMO            0x0080  // Demonstration Mode
#define SWH_STATUS_MODE_MASK            0x00F0  // Mask
// other status bits
#define SWH_STATUS_CAN_WATCHDOG_OFF     0x0100  // CAN Watchdog OFF
#define SWH_STATUS_OVERLOAD_ACTIVE      0x0200  // Overload Protection Active
#define SWH_STATUS_ENDSTOPS_ACTIVE      0x0400  // End Stops Active
#define SWH_STATUS_ENC_INDEX            0x0800  // Encoder Index found
#define SWH_STATUS_REFERENCING_COMPLETE 0x2000  // Reference Tour completed
#define SWH_STATUS_CPU_WATCHDOG_OK      0x4000  // Test CPU Watchdog Success

/* Set numbers for read only values */
#define SWH_NUM_ROVALUES_ENTRIES        0
#define SWH_NUM_ROVALUES_TEMPERATURES   1
#define SWH_NUM_ROVALUES_ABS_TORQUE     2
#define SWH_NUM_ROVALUES_ERROR          3

/* Set numbers for version information */
#define SWH_NUM_EXTVERSION_ENTRIES      0
#define SWH_NUM_EXTVERSION_SOFTWARE     1
#define SWH_NUM_EXTVERSION_PRODUCT      2

/* Set numbers for optional parameters for SENSO-Wheel effects */
#define SWH_NUM_EFFPARAMS_ENTRIES       0
#define SWH_NUM_EFFPARAMS_ENDSTOP       1
#define SWH_NUM_EFFPARAMS_REFERENCING   2
#define SWH_NUM_EFFPARAMS_FRICTION      3
#define SWH_NUM_EFFPARAMS_ENDSTOPVIBR   4
#define SWH_NUM_EFFPARAMS_FRICTION_2    5
#define SWH_NUM_EFFPARAMS_DAMPING       6
#define SWH_NUM_EFFPARAMS_TORQUESCALING 7

/* Set numbers for user parameters */
#define SWH_NUM_USERPARAMS_ENTRIES      0
#define SWH_NUM_USERPARAMS_CANBAUDRATE  1

/* Set numbers for system parameters */
#define SWH_NUM_SYSPARAMS_ENTRIES       0
#define SWH_NUM_SYSPARAMS_INCS_PER_REV  1
#define SWH_NUM_SYSPARAMS_TORQUES       2

/* List of product codes */
#define SWH_PRODUCTCODE_SDLC            1   // SENSO-Wheel SD-LC
#define SWH_PRODUCTCODE_VEM             2   // SENSO-Wheel VEM
#define SWH_PRODUCTCODE_ECO             3   // SENSO-Wheel ECO
#define SWH_PRODUCTCODE_HT              4   // SENSO-Wheel HT

/* SENSO-Wheel error codes */
#define SWH_ERR_OK                              0x0000  // No error
#define SWH_ERR_OVERCURRENT_POWERSTAGE          0x0001  // Overcurrent in output stage
#define SWH_ERR_SUPPLY_VOLTAGE_TOO_HIGH         0x0002  // Supply voltage too high
#define SWH_ERR_SUPPLY_VOLTAGE_TOO_LOW          0x0004  // Supply voltage too low
#define SWH_ERR_OVERTEMPERATURE_POWERSTAGE      0x0008  // Overtemperature in output stage
#define SWH_ERR_HALLSENSOR                      0x0010  // Hall sensor fault
#define SWH_ERR_CAN                             0x0020  // CAN bus error
#define SWH_ERR_ENCODER_HALL                    0x0040  // Hall sensors swapped or encoder error
#define SWH_ERR_CAN_WATCHDOG                    0x0080  // CAN Watchdog
#define SWH_ERR_ILLEGAL_STEER_POSITION          0x0100  // Invalid position (Abs(Position) > Maximum)
#define SWH_ERR_SWITCH_ON_WHILE_OUTSIDE_ENDSTOP 0x0200  // Power-Up while |Position|> End stop position
#define SWH_ERR_REFERENCING_TIMEOUT             0x0400  // Position Referencing time limit exceeded
#define SWH_ERR_OVERTEMPERATURE_BREAKCHOPPER    0x0800  // Overtemperature in Brake Chopper
#define SWH_ERR_HARDWARE_SPECIFIC_ERROR         0x2000  // Hardware specific error.
#define SWH_ERR_UPDATE_WRONG_MESSAGE_NUMBER     0x4000  // Wrong number of CAN firmware update messages
#define SWH_ERR_UPDATE_CRC_ERROR                0x8000  // Wrong checksum of CAN firmware update


#endif /* SENSOWHEEL_H_ */

