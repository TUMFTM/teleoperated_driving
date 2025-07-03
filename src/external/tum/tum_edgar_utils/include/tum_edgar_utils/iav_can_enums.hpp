#pragma once

// IMPORTANT: CANIDs are dummy values for open-source release.
namespace CANIDs
{
    enum
    {
        Motion = 0,
        Steering = 1,
        PassengerPresence = 2,
        Powertrain_1 = 3,
        Misc_1 = 4,
        Assistance_1 = 5,
        GPSData_2 = 6,
        GPSData_1 = 7,
        GPSData_3 = 8,
        Wheel_1 = 9,
        Wheel_2 = 10, 
        Assistance_2 = 11,
        Misc_2 = 12,
        SteeringWheelButtons = 13,
        FrontObject_1 = 14,
        FrontObject_2 = 15,
        FrontObject_3 = 16,
        FrontObject_4 = 17,
        DriverOperations = 18,
        Acceleration_Interface = 19,
        SteeringCurv_Interface = 20,
        Gear_Interface = 21,
        Vehicle_Interface_01 = 22,
        Vehicle_Interface_02 = 23,
        Vehicle_ControlModes = 24,
        Gateway_States = 25,
        AI_CancelConditions = 26,
        SI_CancelConditions = 27,
        GI_CancelConditions = 28,
        VI_CancelConditions = 29,
    };
}

namespace SteeringWheelButtons
{
    enum
    {
        None = 0,
        Phone = 1,
        MenuUp = 2,
        MenuDown = 3,
        VoiceInput = 4,
        MenuLeft = 5,
        MenuRight = 6,
        Ok = 7,
        NextTitle = 8,
        PreviousTitle = 9,
        VolumeUp = 10,
        VolumeDown = 11,
        DriverAssist = 12,
        View = 13,
        ConfigurableButton = 14,
        MenueList = 15,
        LeftThumbwheelUp = 16,
        LeftThumbwheelDown = 17,
        LeftThumbwheelPress = 18,
        RightThumbwheelUp = 19,
        RightThumbwheelDown = 20,
        RightThumbwheelPress = 21,
        Return = 22,
    };
}

namespace GatewayStates
{
    enum
    {
        Ready = 4,
        STIActive = 3,
        Locked = 3,
        SAIActive = 2,
        ExternalError = 15,
        InternalError = 14,
        EPBEngaged = 13,
        Active = 2,
        Standby = 1,
        Off = 0,
    };
}

namespace IAVGatewayStates
{
    enum
    {
        Error = 9,
        Critical = 8,
        EPBClosed = 7,
        CustGatewayOn = 6,
        ACCMainOn = 5,
        ACCMainOff = 4,
        EngineOn = 3,
        EngineOff = 2,
        IgnitionOn = 1,
        IgnitionOff = 0,
    };
}

namespace EdgarCustomerGears
{
    enum
    {
        Neutral = 6,
        Park = 1,
        Sport = 9,
        Reverse = 7,
        Drive = 5,
        Error = 15,
        NoInformation = 0,
    };
}

namespace EdgarVehicleGears
{
    enum
    {
        Intermediate = 0,
        Init = 1,
        Park = 5,
        Reverse = 6,
        Neutral = 7,
        Drive = 8,
        Sport = 9,
        Efficient = 10,
        TipInS = 13,
        TipInD = 14,
    };
}

namespace EdgarTurnIndicator
{
    enum
    {
        NoRequest = 0,
        Left = 1,
        Right = 2,
        Hazard = 3,
    };
}

namespace EdgarHorn
{
    enum
    {
        Off = 0,
        On = 1,
    };
}

namespace EdgarFrontWiperRequest
{
    enum
    {
        Off = 0,
        OneTap = 1,
        WindshieldWash = 2,
        Intermittent = 3,
        Low = 4,
        High = 5,
    };
}

namespace EdgarRearWiperRequest
{
    enum
    {
        Off = 0,
        Intermittent = 1,
        WindshieldWash = 2,
    };
}

namespace EdgarFrontWiperIntervalRequest
{
    enum
    {
        Off = 0,
        Level1 = 1,
        Level2 = 2,
        Level3 = 3,
        Level4 = 4,
    };
}

namespace EdgarHeadlightRequest
{
    enum
    {
        Off = 0,
        LowBeam = 1,
        Parking = 2,
        Auto = 3,
    };
}

namespace EdgarHighBeamRequest
{
    enum
    {
        Off = 0,
        Flasher = 1,
        HighBeam = 2,
    };
}

namespace FrontAssistIntervention
{
    enum
    {
        NoIntervention = 0,
        PrefillRequest = 1,
        BrakeJerkRequest = 2,
        EmergencyBrakerequested = 3,
    };
}