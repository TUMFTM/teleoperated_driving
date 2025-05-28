// Copyright TUM-FTM
#include "audi_steering_wheel_controller.h"

#define BITVAL(byte, bit)   ((Data[byte]&(1 << bit)) > 0)
#define BITVAL2(byte, bit)  ((Origin[byte]&(1 << bit)) > 0)

AudiSteeringWheelController::AudiSteeringWheelController() :CanInterface(PCAN_USBBUS2, PCAN_BAUD_100K) {
    buttonState.assign(31, false);
}

AudiSteeringWheelController::~AudiSteeringWheelController() {
}

void AudiSteeringWheelController::set_button_callback(std::function<void(const std::vector<bool>)> f) {
    _button_callback = f;
}

void AudiSteeringWheelController::HandleReply(unsigned int Identifier, unsigned char Length, const BYTE * Data) {
    switch (Identifier) {
    case 0x2c1:     // blinker
    {
        buttonState.at(0) = BITVAL(0, 0); //Blinker Left
        buttonState.at(1) = BITVAL(0, 1); //BlinkerRight
        buttonState.at(2) = BITVAL(0, 2); //fernlicht Flash
        buttonState.at(3) = BITVAL(0, 3); //Fernlicht an
        buttonState.at(4) = BITVAL(0, 7); //
        //_jsn.pos1 = (Data[1] >> 1) & 0x07; //Wiper state
        //_jsn.pos2 = (Data[2] >> 2) & 0x03; //Wiper speed
        buttonState.at(10) = BITVAL(1, 5); //Wiper Pull
        buttonState.at(5)= BITVAL(2, 6); // ResbuttonState.at)
        buttonState.at(11) = BITVAL(1, 0); // Wiper Tip down
        buttonState.at(12) = BITVAL(2, 4); // Wiper button Down
        buttonState.at(13) = BITVAL(2, 5); // Wiper button up

        _button_callback(buttonState);
    }
    break;
    case 0x289:
    {
        const BYTE* Origin = Data;
        // paddle left
        buttonState.at(22) = BITVAL(3, 0);
        //paddle right
        buttonState.at(23) = BITVAL(3, 1);
        // XX 01 XX 00 -> normal
        // XX 05 XX XX -> lever down
        buttonState.at(24) = BITVAL(1, 2);
        // XX 15 XX XX -> lever long down
        buttonState.at(25) = BITVAL(1, 4);
        // XX 09 XX XX -> lever up
        buttonState.at(26) = BITVAL(1, 3);
        // XX 29 XX XX -> lever long up
        buttonState.at(27) = BITVAL(1, 5);
        // XX 03 XX XX -> lever push
        buttonState.at(28) = BITVAL(1, 1);
        // XX 00 XX XX -> lever pushed to back
        buttonState.at(29) = !BITVAL(1, 0);
        // acc lever pull
        buttonState.at(30) = BITVAL(2, 1);

        _button_callback(buttonState);
    }
    break;
    case 0x5C3:
    {
        // Button right not pressed:
        // 0x39 00 => 0b0011 1001 0000 0000
        // Button right pressed:
        // 0x3C 2A => 0b0011 1100 0010 1010
        // 0x3C 00 => 0b0011 1100 0000 0000 for 2s after press
        //if (Data[0]==0x3C && Data[1]==0x2A)

        if (Data[1] == 0x2A) // with this we also detect fast klicks
            buttonState.at(14) = true;
        else
            buttonState.at(14) = false;

        // Button left pressed:
        // 0x3A 00
        // 0x3A 1C => 0b0011 1010 0001 1100
        // 0x3A 00 => 0b0011 1010 0000 0000 for 2 s after press
        // 0x39 01 => 0b0011 1001 0000 0001 long press
        //if (Data[0]==0x3A && Data[1]==0x1C)
        if (Data[1] == 0x1C || Data[1] == 0x01) // with this we also detect fast klicks
            buttonState.at(15) = true;
        else
            buttonState.at(15) = false;

        // right wheel up:
        // 0xXX 06  => 0b0110
        if (Data[1] == 0x06)
            buttonState.at(16) = true;
        else
            buttonState.at(16) = false;
        // right wheel down:
        // 0xXX 07  => 0b0111
        if (Data[1] == 0x07)
            buttonState.at(17) = true;
        else
            buttonState.at(17) = false;

        // left wheel up:
        // 0xXX 0B  => 0b1011
        if (Data[1] == 0x0B) {
            buttonState.at(18) = true;
            }
        else
            buttonState.at(18) = false;
        // left wheel down:
        // 0xXX 0C  => 0b1100
        if (Data[1] == 0x0C) {
            buttonState.at(19) = true;
        }
        else
            buttonState.at(19) = false;

        // wheel left pressed:
        // 0x39 08
        if (Data[1] == 0x08)
            buttonState.at(20) = true;
        else
            buttonState.at(20) = false;

        // wheel right pressed:
        // 0x3B A7
        if (Data[1] == 0xA7)
            buttonState.at(21) = true;
        else
            buttonState.at(21) = false;

        _button_callback(buttonState);
    }
    break;
    default:
        break;
    }
}


