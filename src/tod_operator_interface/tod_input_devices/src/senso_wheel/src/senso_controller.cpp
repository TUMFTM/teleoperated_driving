// Copyright TUM-FTM
#include "senso_controller.h"
#include "sensowheel.h"

#include <unistd.h>

#define INDEX_POSITION_OFFSET   21
#define STAGE_CLEAR_ERROR   1
#define STAGE_START 2
#define STAGE_REFERENCING 3
#define STAGE_RUNNING   4
#define min(a, b) (((a) < (b))?(a):(b))
#define max(a, b) (((a) > (b))?(a):(b))


SensoController::SensoController() : CanInterface(PCAN_USBBUS1, PCAN_BAUD_1M) {
    reset_values();
}

void SensoController::set_steering_callback(std::function<void(const int)> f) {
    _steering_callback = f;
}

void SensoController::set_pedal_callback(std::function<void(const int, const int, const int)> f) {
    _pedal_callback = f;
}

void SensoController::reset_values() {
    _uiCurrentState = 0;
    _uiCurrentError = 0;
    _uiControlWord = 0;

    _nEndStopPosition = SWH_INIT_ENDSTOPPOS;
    _nPositionOffset = INDEX_POSITION_OFFSET;
    _nEncoderIndexPositionReceived = 0;
    _uiControlTorqueLimitation = SWH_INIT_TORQUE_LIMITATION;
    _uiControlPeakTorqueLimitation = SWH_INIT_PEAKTORQUE_LIMITATION;

    //CAN-message: mode normal and basic
    _nDemandedTorque = 0;
    _nDemandedFriction = 300;
    _nDemandedDamping = 31;
    _nDemandedStiffness = 20;
    _nActualSteeringPosition = 0;
    _nActualSteeringVelocity = 0;
    _nActualSteeringTorque = 0;
    _nActualAccPedalPos = 0;
    _nActualBrakePedalPos = 0;
    _nActualClutchPedalPos = 0;

    //CAN-message: I/O values
    _uiDigitalInputs = 0;
    _uiAnalogInput1 = 0;
    _uiAnalogInput2 = 0;

    ///////////////////
    _nAnalogWheelPosition = 0;
    _uiAuxiliaryFunctions = SWH_CTRL_AUX_FRICTION_2 | SWH_CTRL_AUX_CAN_WATCHDOG_OFF;
    _cbMessageSend = NULL;
    _cbHandleErrors = NULL;
    _cbHandleStatus = NULL;

    _bInputsReceived = false;
    _bAbsolutePositionSet = false;

    _uiInitStage = STAGE_START;
}

SensoController::~SensoController() {
    SwitchOff();
}

void SensoController::SwitchOff() {
    // Switch off solenoid to fix steering wheel
    for (int i = 0; i < 100; i++) {
        SendIO(0);
        usleep(1*1000);
    }
    // set status to OFF
    _uiControlWord = SWH_CTRL_MODE_NORMAL | SWH_CTRL_SWITCH_OFF;
    SendControlMessage();
}

bool SensoController::InitWheel() {
    if (_uiCurrentError != SWH_ERR_OK) { // always go back to the first stage if an error was signalled
        _uiInitStage = STAGE_CLEAR_ERROR;
    } else if (_uiCurrentState == 0) { // no statusword received so far, so start with STAGE_START
        _uiInitStage = STAGE_START;
    }

    switch (_uiInitStage) {
    case STAGE_CLEAR_ERROR:
        // clear any error flags and preconfigure SensoWheel
        _uiControlWord = SWH_CTRL_MODE_NORMAL | SWH_CTRL_SWITCH_QUITERROR | _uiAuxiliaryFunctions;
        SendControlMessage();

        _uiInitStage++;
        break;
    case STAGE_START:
        // set all digital outputs to enabled and request the potentiometer value
        // make sure the solenoid is open so that the steering wheel can move
        SendIO(SWH_DIGOUT_MAX);
        // clear any error flags and preconfigure SensoWheel
        _uiControlWord = SWH_CTRL_MODE_NORMAL | SWH_CTRL_SWITCH_OFF | _uiAuxiliaryFunctions;
        SendControlMessage();

        _uiInitStage++;
        break;
    case STAGE_REFERENCING:

        if ((_uiCurrentState&SWH_STATUS_REFERENCING_COMPLETE) == 0) {
            if ((_uiCurrentState&SWH_STATUS_MODE_MASK) != SWH_STATUS_MODE_REF) {
                // switch to ref mode first
                _uiControlWord = SWH_CTRL_MODE_REF | SWH_CTRL_SWITCH_READY | _uiAuxiliaryFunctions;
                SendControlMessage();
            } else {
                //alway send the message until the ref is done
                //enable motor in ref mode
                _uiControlWord = SWH_CTRL_MODE_REF | SWH_CTRL_SWITCH_ON | _uiAuxiliaryFunctions;
                SendControlMessage();
            }
        } else { // the index was found, so switch to normal mode
            // switch to normal mode first
            _uiControlWord = SWH_CTRL_MODE_NORMAL | SWH_CTRL_SWITCH_READY | _uiAuxiliaryFunctions;
            SendControlMessage();

            // go to next stage
            _uiInitStage++;
        }
        break;
    case STAGE_RUNNING:
        if ((_uiCurrentState&SWH_STATUS_MODE_MASK) != SWH_STATUS_MODE_NORMAL) {
            // switch to normal mode first
            _uiControlWord = SWH_CTRL_MODE_NORMAL | SWH_CTRL_SWITCH_READY | _uiAuxiliaryFunctions;
            SendControlMessage();
        } else if ((_uiCurrentState&SWH_STATUS_STATE_MASK) == SWH_STATUS_STATE_OFF) {
            // switch ready in mode normal
            _uiControlWord = SWH_CTRL_MODE_NORMAL | SWH_CTRL_SWITCH_READY | _uiAuxiliaryFunctions;
            SendControlMessage();
        } else if ((_uiCurrentState&SWH_STATUS_STATE_MASK) == SWH_STATUS_STATE_READY) {
            // switch on in mode normal
            _uiControlWord = SWH_CTRL_MODE_NORMAL | SWH_CTRL_SWITCH_ON | _uiAuxiliaryFunctions;
            SendControlMessage();
        } else { // we are in normal mode and switched o
            return true;
        }
        break;
    }
    return false;
}

void SensoController::HandleReply(unsigned int Identifier, unsigned char Length, const BYTE* Data) {
    switch (Identifier) {
    case SWH_CAN_ID_SEND_CONTROL:       // control module
    {
        _uiCurrentState = ((uint16_t)Data[1] << 8) | Data[0];
        _uiCurrentError = ((uint16_t)Data[3] << 8) | Data[2];

        // end stop position
        int nActEndStopPosition = ((char)Data[5] << 8) | Data[4];
        // encoder index position
        _nEncoderIndexPositionReceived = ((char)Data[7] << 8) | Data[6];
        // check received values for errors
        HandleErrors();
        HandleStatus();
        break;
    }
    case SWH_CAN_ID_SEND_MODENORMAL: // Normal Mode actual values
        // position (bytes: 0-3 range: -2^31...(2^31-1) unit:increments)
        _nActualSteeringPosition = ((char)Data[3] << 24) |
            ((unsigned char)Data[2] << 16) | ((unsigned char)Data[1] << 8) | (unsigned char)Data[0];
        _steering_callback(_nActualSteeringPosition);

        // steering velocity (bytes: 4-5 range: -32768...32767 unit:U/min)
        _nActualSteeringVelocity = ((char)Data[5] << 8) | (unsigned char)Data[4];

        // steering torque (bytes: 6-7 range: -32768...32767 unit:mNm)
        _nActualSteeringTorque = ((char)Data[7] << 8) | (unsigned char)Data[6];

        break;

    case SWH_CAN_ID_SEND_PEDAL: // Pedal Values
        // acceleration pedal (byte 2-3, Range 0...4095)
        _nActualAccPedalPos = ((unsigned char)Data[3] << 8) | (unsigned char)Data[2];

        // brake pedal (byte 4-5, Range 0...4095)
        _nActualBrakePedalPos = ((unsigned char)Data[5] << 8) | (unsigned char)Data[4];

        // clutch pedal (byte 6-7, Range 0...4095)
        _nActualClutchPedalPos = ((unsigned char)Data[7] << 8) | (unsigned char)Data[6];

        _pedal_callback(_nActualAccPedalPos, _nActualBrakePedalPos, _nActualClutchPedalPos);

        break;

    case SWH_CAN_ID_SEND_MODEBASIC: // Basic Mode actual values
        // Position
        _nActualSteeringPosition = (((char)Data[3] << 24) |
            ((unsigned char)Data[2] << 16) | ((unsigned char)Data[1] << 8) | (unsigned char)Data[0]);

        _steering_callback(_nActualSteeringPosition);
        break;

    case SWH_CAN_ID_SEND_ADIO: // I/O connector values, analog and digital
        // digital input values
        _uiDigitalInputs = ((unsigned char)Data[1] << 8) | (unsigned char)Data[0];

        // analog input values
        _uiAnalogInput1 = ((unsigned char)Data[3] << 8) | (unsigned char)Data[2];
        _uiAnalogInput2 = ((unsigned char)Data[5] << 8) | (unsigned char)Data[4];

        CalculateAbsoluteWheelPosition(_uiAnalogInput1);

        _bInputsReceived = true;

    case SWH_CAN_ID_SEND_ABSPOS:
        //("\n  Reference DONE\n");
        break;
    }
}

void SensoController::SendMessage(unsigned int Identifier, unsigned char Length, BYTE* Data) {
    SendCANMessage(0, 1, Identifier, Length, Data); // Todo: resolve identifier type
}

void SensoController::SendControlMessage() {
    BYTE Data[SWH_CAN_DLC_RECV_CONTROL];
    // Control Word
    Data[0] = 0x00ff & _uiControlWord;
    Data[1] = _uiControlWord >> 8;
    // End Stop Position
    // End Stop Position
    Data[2] = 0x00ff & (int)(_nEndStopPosition);
    Data[3] = (int)(_nEndStopPosition) >> 8;
    // Position Offset
    Data[4] = 0x00ff & _nPositionOffset;
    Data[5] = _nPositionOffset >> 8;
    // Torque Limits
    Data[6] = _uiControlTorqueLimitation;       // limit in percent
    Data[7] = _uiControlPeakTorqueLimitation;
    SendMessage(SWH_CAN_ID_RECV_CONTROL, SWH_CAN_DLC_RECV_CONTROL, Data);
}

void SensoController::SendIO(uint16_t uiDigOut) {
    BYTE Data[SWH_CAN_DLC_RECV_ADIO];
    // digital output
    Data[0] = 0x00ff & uiDigOut;
    Data[1] = uiDigOut >> 8;
    // transmit message
    SendMessage(SWH_CAN_ID_RECV_ADIO, SWH_CAN_DLC_RECV_ADIO, Data);
}

void SensoController::HandleErrors() {
        char txtErr[200] = "";
        CheckSensoWheelError(_uiCurrentError, txtErr);
        //qDebug() << txtErr;
    if (_cbHandleErrors)
        _cbHandleErrors(_uiCurrentError, _uiCurrentError != SWH_ERR_OK);
}

void SensoController::HandleStatus() {
    char txtStat[200] = "";
    CheckSensoWheelStatus(_uiCurrentState, txtStat);
    if (_cbHandleStatus)
        _cbHandleStatus(_uiCurrentState);
}

void SensoController::CalculateAbsoluteWheelPosition(uint16_t uiPotentiometerValue) {
    _nAnalogWheelPosition = uiPotentiometerValue * 360 * 10 / 1009;
}

void SensoController::DoCyclicEvents() {
    static int pedalRequestFlag = 0;
    if ((_uiCurrentState & SWH_STATUS_MODE_MASK) == SWH_STATUS_MODE_NORMAL) { // we are in normal mode
        // send feedback values
        BYTE Data[SWH_CAN_DLC_RECV_MODENORMAL];

        // request pedal values
        // Request flag
        pedalRequestFlag++;

        if (pedalRequestFlag == 10) {
            pedalRequestFlag = 0;
            Data[0] = 0x1;
            // transmit message
            SendMessage(SWH_CAN_ID_RECV_PEDAL, SWH_CAN_DLC_RECV_PEDALS, Data);
        }
        // Torque
        Data[0] = 0x00ff & (int)(_nDemandedTorque);
        Data[1] = (int)(_nDemandedTorque) >> 8;

        // Friction
        Data[2] = 0x00ff & (int)(_nDemandedFriction);
        Data[3] = (int)(_nDemandedFriction) >> 8;

        // Damping
        Data[4] = 0x00ff & (int)(_nDemandedDamping);
        Data[5] = (int)(_nDemandedDamping) >> 8;

        // Spring Stiffnes
        Data[6] = 0x00ff & (int)_nDemandedStiffness;
        Data[7] = (int)_nDemandedStiffness >> 8;

        // transmit message
        SendMessage(SWH_CAN_ID_RECV_MODENORMAL, SWH_CAN_DLC_RECV_MODENORMAL, Data);
    } else if ((_uiCurrentState & SWH_STATUS_MODE_MASK) == SWH_STATUS_MODE_BASIC) {
        BYTE Data[SWH_CAN_DLC_RECV_MODEBASIC];
        // send torque
        // Torque
        Data[0] = 0x00ff & (int)(_nDemandedTorque);
        Data[1] = (int)(_nDemandedTorque) >> 8;
        // transmit message
        SendMessage(SWH_CAN_ID_RECV_MODEBASIC, SWH_CAN_DLC_RECV_MODEBASIC, Data);

        // request pedal values
        // Request flag
        Data[0] = 0x1;
        // transmit message
        SendMessage(SWH_CAN_ID_RECV_PEDAL, SWH_CAN_DLC_RECV_PEDALS, Data);
    }
}

void SensoController::CheckSensoWheelError(int err, char* pTxtBuf) {
    if (err == SWH_ERR_OK)
        strcpy(pTxtBuf, "OK\n");
    else
        strcpy(pTxtBuf, "\n");

    if (err & SWH_ERR_OVERCURRENT_POWERSTAGE)
        strcat(pTxtBuf, "   Overcurrent in output stage\n");
    if (err & SWH_ERR_SUPPLY_VOLTAGE_TOO_HIGH)
        strcat(pTxtBuf, "   Supply voltage too high\n");
    if (err & SWH_ERR_SUPPLY_VOLTAGE_TOO_LOW)
        strcat(pTxtBuf, "   Supply voltage too low\n");
    if (err & SWH_ERR_OVERTEMPERATURE_POWERSTAGE)
        strcat(pTxtBuf, "   Overtemperature in output stage\n");
    if (err & SWH_ERR_HALLSENSOR)
        strcat(pTxtBuf, "   Hall sensor fault\n");
    if (err & SWH_ERR_CAN)
        strcat(pTxtBuf, "   CAN bus error\n");
    if (err & SWH_ERR_ENCODER_HALL)
        strcat(pTxtBuf, "   Hall sensors swapped or encoder error\n");
    if (err & SWH_ERR_CAN_WATCHDOG)
        strcat(pTxtBuf, "   CAN Watchdog\n");
    if (err & SWH_ERR_ILLEGAL_STEER_POSITION)
        strcat(pTxtBuf, "   Invalid position (Position > Maximum position)\n");
    if (err & SWH_ERR_SWITCH_ON_WHILE_OUTSIDE_ENDSTOP)
        strcat(pTxtBuf, "   Power-Up while Position > End stop position\n");
    if (err & SWH_ERR_REFERENCING_TIMEOUT)
        strcat(pTxtBuf, "   Position Referencing time limit exceeded\n");
    if (err & SWH_ERR_OVERTEMPERATURE_BREAKCHOPPER)
        strcat(pTxtBuf, "   Overtemperature in Brake Chopper\n");
    if (err & SWH_ERR_UPDATE_WRONG_MESSAGE_NUMBER)
        strcat(pTxtBuf, "   Wrong number of CAN firmware update messages\n");
    if (err & SWH_ERR_UPDATE_CRC_ERROR)
        strcat(pTxtBuf, "   Wrong checksum of CAN firmware update\n");
    return;
}

void SensoController::CheckSensoWheelStatus(int status, char* pTxtBuf) {
    strcpy(pTxtBuf, "\n");
    if ((status & 0xF) == 0)
        strcat(pTxtBuf, "   State:  OFF\n");
    if (status & SWH_STATUS_STATE_READY)
        strcat(pTxtBuf, "   State:  READY\n");
    if (status & SWH_STATUS_STATE_ON)
        strcat(pTxtBuf, "   State:  ON -> motor activated\n");
    if (status & SWH_STATUS_STATE_ERROR)
        strcat(pTxtBuf, "   State:  ERROR\n");
    if (status & SWH_STATUS_MODE_NORMAL)
        strcat(pTxtBuf, "   steering wheel in NORMAL MODE\n");
    if (status & SWH_STATUS_MODE_BASIC)
        strcat(pTxtBuf, "   steering wheel in BASIC MODE\n");
    if (status & SWH_STATUS_MODE_REF)
        strcat(pTxtBuf, "   steering wheel in REFERENCING MODE\n");
    if (status & SWH_STATUS_MODE_DEMO)
        strcat(pTxtBuf, "   steering wheel in DEMONSTRATION MODE\n");
    if (status & SWH_STATUS_CAN_WATCHDOG_OFF)
        strcat(pTxtBuf, "   CAN Watchdog Off\n");
    if (status & SWH_STATUS_OVERLOAD_ACTIVE)
        strcat(pTxtBuf, "   Overload protection active\n");
    if (status & SWH_STATUS_ENDSTOPS_ACTIVE)
        strcat(pTxtBuf, "   End stops active\n");
    if (status & SWH_STATUS_ENC_INDEX)
        strcat(pTxtBuf, "   Encoder index\n");
    if (status & SWH_STATUS_REFERENCING_COMPLETE)
        strcat(pTxtBuf, "   Position referencing completed\n");
    if (status & SWH_STATUS_CPU_WATCHDOG_OK)
        strcat(pTxtBuf, "   Test CPU-Watchdog successful\n");
    return;
}
