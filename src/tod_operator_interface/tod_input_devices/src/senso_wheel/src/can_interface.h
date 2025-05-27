

/**
 * @file can_interface.hpp
 * @brief Header file for the CanInterface class, which provides an abstraction for interacting with CAN bus devices.
 * @copyright TUMFTM 2024
 * 
 * This file defines the `CanInterface` class, which serves as a base class for CAN bus communication.
 * It provides methods for initialization, reading, and writing CAN messages and includes virtual functions
 * for custom behavior in derived classes.
 *
 * Key features:
 * - CAN bus initialization and teardown.
 * - Reading and writing CAN messages.
 * - Virtual methods for custom CAN message handling and sending.
 */

#pragma once

#include <iostream>
#include "PCANBasic.h"
#include <unistd.h>

#define MAX_PATH 260    ///< Maximum path length for event handling structures.
typedef void *HANDLE;

/**
 * @class CanInterface
 * @brief Provides a base class for CAN bus communication.
 * 
 * The `CanInterface` class initializes a CAN bus channel, handles incoming and outgoing messages, 
 * and provides virtual functions for derived classes to implement specific behavior.
 */
class CanInterface{
protected:
    unsigned int pcan_device;       ///< CAN device identifier.
    BYTE _channel;                  ///< Channel number for the CAN interface.
    WORD _baudrate;                 ///< Baud rate for the CAN communication.
    BYTE _channelNr;                ///< Channel number assigned by the CAN interface.
    BYTE _senderID;                 ///< Identifier for messages sent by the CAN interface.
    HANDLE _hMessageReceived;       ///< Handle for message reception events.
    wchar_t _szEventName[MAX_PATH]; ///< Event name for message reception.
    
    /**
     * @brief Virtual function to handle incoming CAN messages.
     * @param Identifier The CAN message identifier.
     * @param Length The length of the CAN message data.
     * @param Data Pointer to the CAN message data.
     *
     * This method must be implemented by derived classes to define how incoming CAN messages are processed.
     */
    virtual void HandleReply(unsigned int Identifier, unsigned char Length, const BYTE* Data) = 0;
    
    /**
     * @brief Sends a CAN message.
     * @param Sender The ID of the sender.
     * @param IdentifierType The type of identifier (standard or extended).
     * @param Identifier The CAN message identifier.
     * @param Length The length of the CAN message data.
     * @param Data Pointer to the CAN message data.
     */
    virtual void SendCANMessage(unsigned char Sender, unsigned char IdentifierType,
        unsigned int Identifier, unsigned char Length, const BYTE* Data);

public:
    /**
     * @brief Constructs a CanInterface instance.
     * @param channel The CAN channel to use.
     * @param baudrate The baud rate for CAN communication.
     *
     * Initializes the CAN interface with the specified channel and baud rate.
     */
    CanInterface(BYTE channel, WORD baudrate);

    /**
     * @brief Destructor for the CanInterface class.
     *
     * Cleans up by uninitializing the CAN interface.
     */
    ~CanInterface();

    bool _bInitSuccessfull;     ///< Indicates whether the CAN interface was initialized successfully.
    bool _bGetData;             ///< Indicates whether the interface is actively receiving data.

    /**
     * @brief Runs the main loop for reading CAN messages.
     *
     * Continuously reads messages from the CAN channel and invokes the `HandleReply` method for each message.
     */
    void run();

    /**
     * @brief Closes the CAN interface.
     *
     * Uninitializes the CAN channel and cleans up resources.
     */
    void close();

    /**
     * @brief Initializes the CAN interface.
     *
     * Configures the CAN channel, sets up the baud rate, and prepares the interface for communication.
     */
    void Init();
};
