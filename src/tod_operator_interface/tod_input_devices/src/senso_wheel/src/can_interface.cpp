// Copyright TUM-FTM
#include "can_interface.h"
#include <signal.h>
#include <sys/eventfd.h>

CanInterface::CanInterface(BYTE channel, WORD baudrate) {
    _channel = channel;
    _bInitSuccessfull = false;
    _bGetData = false;
    _baudrate = baudrate;
}

CanInterface::~CanInterface() {
    CAN_Uninitialize(_channel);
}

void CanInterface::close() {
    CAN_Uninitialize(_channel);
}

//todo: evtl close Methode -> siehe urspr. Implem
void CanInterface::Init() {
    TPCANStatus Status;
    //signal(SIGINT, signal_handler); //Todo: required?
    Status = CAN_Initialize(_channel, _baudrate, 0, 0, 0);
    
    char errorString[256];
    TPCANStatus statusResult = CAN_GetErrorText(Status, 0, errorString);
    
    printf("CAN_Initialize(%xh): Status=0x%x\n", _channel, (int)Status);

    if (Status == PCAN_ERROR_OK) {
        std::cout << "Can Initialized successfully"<< std::endl;
        _bInitSuccessfull = true;
        //CAN_SetValue(_channel,PCAN_RECEIVE_EVENT, &_hMessageReceived, sizeof(_hMessageREceived));
        _bGetData = true;
        //Todo: set some booleans to true (getData, InitSucessful)
    } else {
        // Todo: Add some try again logic (Reset, Uninitialize Can)
        std::cout << "Status: " << errorString << std::endl;
        //std::cout << "Cant Initialize CAN-Interface," <<
        //   " please restart SensoWheel by reconnecting power supply" << std::endl;

    }
}

void CanInterface::run() {
    TPCANMsg msg;
    TPCANTimestamp time;
    TPCANStatus Status;
    //__int64 lTimeStamp;
    if (!_bInitSuccessfull) {
        return;
    }
    while (_bGetData) {
        Status = CAN_Read(_channel, &msg, NULL);
        while (Status == PCAN_ERROR_OK) {
            //QByteArray Data((const char*)msg.DATA, sizeof(msg.DATA));
            HandleReply(msg.ID, sizeof(msg.DATA), (BYTE*)msg.DATA);
            Status = CAN_Read(_channel, &msg, NULL);
        }
        if (Status != PCAN_ERROR_QRCVEMPTY) {
            //printf("CAN_Read(%xh) failure 0x%x\n", pcan_device, (int)Status);
            std::cout << "Error occured while reading CAN messages: " << Status << std::endl;
        }
    }
}

void CanInterface::SendCANMessage(unsigned char Sender,
        unsigned char IdentifierType, unsigned int Identifier, unsigned char Length, const BYTE* Data) {
    TPCANStatus Status;
    TPCANMsg msgSend;
    switch (IdentifierType) {
    case 1: // Standard Message
        msgSend.MSGTYPE = PCAN_MESSAGE_STANDARD;
        break;
    case 2: // Extended Message
        msgSend.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        break;
    }

    msgSend.ID = Identifier;
    msgSend.LEN = Length;
    for (int i = 0; i < (int)msgSend.LEN; i++) {
        msgSend.DATA[i] = Data[i];
    }
    Status = CAN_Write(_channel, &msgSend);
}
