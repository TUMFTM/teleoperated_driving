/**
 * @file senso_controller.hpp
 * @brief Header file for the SensoController class, managing a SensoWheel device through CAN communication.
 * @copyright TUMFTM 2024
 *
 * This file defines the `senso_controller` class, which interfaces with the SensoWheel device over the CAN bus.
 * It provides functionalities for initializing the wheel, handling steering and pedal inputs, and managing
 * device-specific operations such as error handling and feedback.
 *
 * Key features:
 * - Initialization and operation of the SensoWheel device.
 * - Reading and writing steering, acceleration, brake, and clutch positions.
 * - Handling and reporting device errors and statuses.
 * - Support for cyclic events and force feedback.
 */


// Copyright 20xx FTM
#pragma once
#include "can_interface.h"
#include <functional>
#include <cstring>

typedef void(*MessageSendFunction)(uint64_t Identifier, unsigned char Length, BYTE* Data);
typedef void(*HandleErrorsFunction)(uint16_t uiError, bool bIsError);
typedef void(*HandleStatusFunction)(uint16_t uiStatus);

#define NU_POSITION_INCREMENTS 40000 ///< Number of position increments for steering wheel range.
#define MIN_POSITION_INCREMENTS (-1.25*NU_POSITION_INCREMENTS) ///< Minimum position increment for the steering wheel.
#define MAX_POSITION_INCREMENTS (1.25*NU_POSITION_INCREMENTS) ///< Maximum position increment for the steering wheel.
#define MIN_ACC_PEDAL_INCREMENTS 1980 ///< Minimum increment for acceleration pedal.
#define MAX_ACC_PEDAL_INCREMENTS 2630 ///< Maximum increment for acceleration pedal.
#define MIN_BRAKE_PEDAL_INCREMENTS 900 ///< Minimum increment for brake pedal.
#define MAX_BRAKE_PEDAL_INCREMENTS 3350 ///< Maximum increment for brake pedal.
#define MIN_CLUTCH_PEDAL_INCREMENTS 1990 ///< Minimum increment for clutch pedal.
#define MAX_CLUTCH_PEDAL_INCREMENTS 2600 ///< Maximum increment for clutch pedal.

/**
 * @class SensoController
 * @brief Controls a SensoWheel device via the CAN bus.
 *
 * The `SensoController` class handles communication with the SensoWheel device, processing steering and pedal inputs,
 * managing initialization stages, and sending feedback values. It supports custom callbacks for steering and pedal input.
 */
class SensoController : public CanInterface{
private:
    /**
     * @brief Sends a control message to the SensoWheel device.
     */
    void SendControlMessage();

    /**
     * @brief Sends a generic CAN message.
     * @param Identifier The message identifier.
     * @param Length The length of the message data.
     * @param Data Pointer to the message data.
     */
    void SendMessage(unsigned int Identifier, unsigned char Length, BYTE* Data);

    /**
     * @brief Sends I/O-related data to the SensoWheel device.
     * @param uiDigOut Digital output data to send.
     */
    void SendIO(uint16_t uiDigOut);

    /**
     * @brief Checks the SensoWheel's status and updates the provided buffer with a human-readable status.
     * @param status The device status code.
     * @param pTxtBuf Buffer to store the status description.
     */
    void CheckSensoWheelStatus(int status, char* pTxtBuf);

    /**
     * @brief Checks the SensoWheel's error state and updates the provided buffer with a human-readable error description.
     * @param status The error code.
     * @param pTxtBuf Buffer to store the error description.
     */
    void CheckSensoWheelError(int status, char* pTxtBuf);

    // Internal member variables for state and feedback handling
    uint16_t _uiCurrentError;
    uint16_t _uiInitStage;
    uint16_t _uiControlWord;
    uint16_t _uiCurrentState;
    uint16_t _uiAuxiliaryFunctions;
    uint16_t _uiDigitalInputs;
    uint16_t _uiAnalogInput1;
    uint16_t _uiAnalogInput2;
    BYTE _uiControlTorqueLimitation;
    BYTE _uiControlPeakTorqueLimitation;
    int _nEndStopPosition;
    int _nPositionOffset;
    int _nAnalogWheelPosition;
    int _nEncoderIndexPositionReceived;
    int _nActualSteeringPosition;
    int _nActualSteeringVelocity;
    int _nActualSteeringTorque;
    int _nActualAccPedalPos;
    int _nActualBrakePedalPos;
    int _nActualClutchPedalPos;
    int _nDemandedTorque;
    int _nDemandedFriction;
    int _nDemandedDamping;
    int _nDemandedStiffness;
    bool _bInputsReceived;
    bool _bAbsolutePositionSet;

    std::function<void(const int)> _steering_callback; ///< Callback for steering wheel position updates.
    std::function<void(const int, const int, const int)> _pedal_callback; ///< Callback for pedal position updates.

    HandleErrorsFunction _cbHandleErrors; ///< Callback for handling errors.
    HandleStatusFunction _cbHandleStatus; ///< Callback for handling status updates.
    MessageSendFunction _cbMessageSend; ///< Callback for sending messages.

public:
    /**
     * @brief Constructs a SensoController instance and initializes member variables.
     */
    SensoController();

    /**
     * @brief Destructor for the SensoController class.
     *
     * Cleans up resources and switches off the SensoWheel device.
     */
    ~SensoController();

    /**
     * @brief Handles incoming CAN messages.
     * @param Identifier The CAN message identifier.
     * @param Length The length of the CAN message data.
     * @param Data Pointer to the CAN message data.
     *
     * This method processes messages related to the SensoWheel's control, normal mode, pedals, and more.
     */
    void HandleReply(unsigned int Identifier, unsigned char Length, const BYTE* Data) override;
    
    /**
     * @brief Switches off the SensoWheel device.
     */
    void SwitchOff();

    /**
     * @brief Handles error codes from the SensoWheel device.
     */
    void HandleErrors();

    /**
     * @brief Handles status updates from the SensoWheel device.
     */
    void HandleStatus();

    /**
     * @brief Calculates the absolute position of the steering wheel based on potentiometer input.
     * @param _uiAnalogInput The potentiometer value to calculate the position from.
     */
    void CalculateAbsoluteWheelPosition(uint16_t _uiAnalogInput);
    
    /**
     * @brief Performs cyclic events, including sending feedback and requesting pedal values.
     */
    void DoCyclicEvents();

    /**
     * @brief Initializes the SensoWheel device.
     * @return `true` if initialization is successful, `false` otherwise.
     */
    bool InitWheel();

    /**
     * @brief Sets the callback for steering wheel position updates.
     * @param f Callback function to handle steering updates.
     */
    void set_steering_callback(std::function<void(const int)> f);

    /**
     * @brief Sets the callback for pedal position updates.
     * @param f Callback function to handle pedal updates.
     */
    void set_pedal_callback(std::function<void(const int, const int, const int)> f);

    /**
     * @brief Resets internal state and feedback values.
     */
    void reset_values();
};

