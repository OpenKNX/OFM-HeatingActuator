#include "HeatingActuatorModule.h"
#include "HeatingActuatorChannel.h"
#include "ClimateDeviceHeatingActuator.h"
#include "ClimateControlModule.h"
#include "RoomChannel.h"
#include "PIController.h"
#include "PWMController.h"
#include "TargetTemperatureManipulationController.h"

#define DeviceKoOffset (CLI_KoCDev2Power - CLI_KoCDev1Power)
#undef CLI_KoCalcNumber
#define CLI_KoCalcNumber(index) (index + CLI_KoBlockOffset + _channelIndex * CLI_KoBlockSize + _deviceIndex * DeviceKoOffset)
#undef CLI_KoCalcIndex
#define CLI_KoCalcIndex(number) ((number >= CLI_KoCalcNumber(0) && number < CLI_KoCalcNumber(CLI_KoBlockSize)) ? (number - CLI_KoBlockOffset) % CLI_KoBlockSize : -1)
#undef CLI_KoCalcChannel
#define CLI_KoCalcChannel(number) ((number >= CLI_KoBlockOffset && number < CLI_KoBlockOffset + CLI_ChannelCount * CLI_KoBlockSize) ? (number - CLI_KoBlockOffset) / CLI_KoBlockSize : -1)

#define DeviceParameterOffset (CLI_CHControlMode2 - CLI_CHControlMode1)
#undef CLI_ParamCalcIndex
#define CLI_ParamCalcIndex(index) (index + CLI_ParamBlockOffset + _channelIndex * CLI_ParamBlockSize + _deviceIndex * DeviceParameterOffset)

#define KoCLI_CHAVCOut KoCLI_CKo13
#define CLI_KoCHVACOut (CLI_KoCKo13 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHAVCOutFeedback KoCLI_CKo14
#define CLI_KoCHVACOutFeedback (CLI_KoCKo14 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHPowerOut KoCLI_CKo15
#define CLI_KoCHPowerOut (CLI_KoCKo15 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHPowerOutFeedback KoCLI_CKo16
#define CLI_KoCHPowerOutFeedback (CLI_KoCKo16 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHCoolingOut KoCLI_CKo13
#define CLI_KoCHCoolingOut (CLI_KoCKo13 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHCoolingOutFeedback KoCLI_CKo14
#define CLI_KoCHCoolingOutFeedback (CLI_KoCKo14 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHHeatingOut KoCLI_CKo15
#define CLI_KoCHHeatingOut (CLI_KoCKo15 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHHeatingOutFeedback KoCLI_CKo16
#define CLI_KoCHHeatingOutFeedback (CLI_KoCKo16 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHDehumificationOut KoCLI_CKo17
#define CLI_KoCHDehumificationOut (CLI_KoCKo17 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHDehumificationOutFeedback KoCLI_CKo18
#define CLI_KoCHDehumificationOutFeedback (CLI_KoCKo18 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHFanOut KoCLI_CKo19
#define CLI_KoCHFanOut (CLI_KoCKo19 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CHFanOutFeedback KoCLI_CKo20
#define CLI_KoCHFanOutFeedback (CLI_KoCKo20 + _deviceIndex * DeviceKoOffset)

#define KoCLI_CDevPower KoCLI_CDev1Power
#define CLI_KoCDevPower (CLI_KoCDev1Power + _deviceIndex * DeviceKoOffset)

#define KoCLI_CDevSet KoCLI_CDev1Set
#define CLI_KoCDevSet (CLI_KoCDev1Set + _deviceIndex * DeviceKoOffset)

#define KoCLI_CDevPWM KoCLI_CDev1PWM
#define CLI_KoCDevPWM (CLI_KoCDev1PWM + _deviceIndex * DeviceKoOffset)

#define KoCLI_CDevSetFb KoCLI_CDev1SetFb
#define CLI_KoCDevSetFb (CLI_KoCDev1SetFb + _deviceIndex * DeviceKoOffset)

#define KoCLI_CDevRoomTemp KoCLI_CDev1RoomTemp
#define CLI_KoCDevRoomTemp (CLI_KoCDev1RoomTemp + _deviceIndex * DeviceKoOffset)

#define KoCLI_CDevIsActive KoCLI_CDev1IsActive
#define CLI_KoCDevIsActive (CLI_KoCDev1IsActive + _deviceIndex * DeviceKoOffset)


ClimateDeviceHeatingActuator::ClimateDeviceHeatingActuator(
    int channelIndex, 
    int deviceIndex, 
    RoomChannel& roomChannel) : 
    ClimateDevice(channelIndex, deviceIndex, roomChannel)
{
    switch (ParamCLI_CHPIPreset1)
    {
        case PT_CLIPIPreset::FloorHeating:
            _piController = new PIController(5.0f, 160.0f * 60.0f);
            break;
        case PT_CLIPIPreset::Radiator:
            _piController = new PIController(3.0f, 80.0f * 60.0f);
            break;
        case PT_CLIPIPreset::AirHeating:
            _piController = new PIController(2.0f, 30.0f * 60.0f);
            break;
        case PT_CLIPIPreset::Custom:
            _piController = new PIController(ParamCLI_CHPII1, ParamCLI_CHPID1 * 60.0f);
            break;
        default:
            // never leave the controller unset, everything else dereferences it unconditionally
            logErrorP("Unknown PI preset %u, falling back to radiator", (uint8_t)ParamCLI_CHPIPreset1);
            _piController = new PIController(3.0f, 80.0f * 60.0f);
            break;
    }
}

ClimateModeSelection ClimateDeviceHeatingActuator::currentMode()
{
    return _mode;
}

void ClimateDeviceHeatingActuator::processInputKo(GroupObject& ko)
{
    //int koNr = CLI_KoCalcIndex(ko.asap());
}

// void ClimateDeviceHeatingActuator::calculateIsActive()
// {
//     if (ParamCLI_CHIsActive1 == PT_CLIIsActive::Calculated && _piController == nullptr && _roomTemperatureRawKnx != std::numeric_limits<uint16_t>::max() && _targetTemperatureRawKnx != std::numeric_limits<uint16_t>::max())
//     {
//         float targetTemperature = _roomChannel.getTemperatureFromRawKnx(_targetTemperatureRawKnx);
//         float roomTemperature = _roomChannel.getTemperatureFromRawKnx(_roomTemperatureRawKnx);
//         switch (_mode)
//         {
//             case ClimateModeSelection::Heating:
//                 setIsActive(targetTemperature - roomTemperature > 0.01f);
//                 break;
//             case ClimateModeSelection::Cooling:
//                 setIsActive(targetTemperature - roomTemperature < -0.01f);
//                 break;
//             case ClimateModeSelection::Auto:
//                 setIsActive(abs(targetTemperature - roomTemperature) > 0.01f);
//                 break;
//             case ClimateModeSelection::Fan:
//             case ClimateModeSelection::Dehumification:
//                 setIsActive(true);
//                 break;
//             default:
//                 setIsActive(false);
//         }
//         logDebugP("Calculate isActive to %s with room %0.1f °C and target %0.1f °C for mode %d", _isActive ? "active" : "inactive", roomTemperature, targetTemperature, (int)_mode);
//     }
// }

void ClimateDeviceHeatingActuator::setTargetTemperature(uint16_t targetTemperatureRawKnx)
{
    _targetTemperatureRawKnx = targetTemperatureRawKnx;
    auto targetTemperature = _roomChannel.getTemperatureFromRawKnx(targetTemperatureRawKnx);   
    logInfoP("Set target temperature to %0.1f °C", targetTemperature);
    // calculateIsActive();

    _piController->setTargetTemperature(targetTemperature);
}

void ClimateDeviceHeatingActuator::setIsActive(bool active)
{
    if (_isActive != active)
    {
        _isActive = active;
        logInfoP("Device %d is now %s", deviceNumber(), active ? "active" : "inactive");
        _roomChannel.isActiveChangedFromDevice();
    }
}

void ClimateDeviceHeatingActuator::loop()
{
    if (_roomTemperatureRawKnx != std::numeric_limits<uint16_t>::max() && _targetTemperatureRawKnx != std::numeric_limits<uint16_t>::max())
    {
        if (_piController->loop())
        {
            float positionValue = _piController->getPositionValue();

            HeatingActuatorChannel* channel = openknxHeatingActuatorModule.getChannel(_channelIndex);
            if (channel == nullptr)
            {
                logErrorP("No heating actuator channel for room channel %d", _channelIndex + 1);
                return;
            }

            channel->moveValveToPosition(positionValue / 100.0f);

            bool isActive = positionValue > 0.001;
            setIsActive(isActive);
        }
    }
}

void ClimateDeviceHeatingActuator::setMode(ClimateModeSelection mode)
{
    logInfoP("Mode: %s", ClimateModeSelectionHelper::toString(mode));
    _mode = mode;
    _piController->setOperationMode(mode);
    // calculateIsActive();
}

void ClimateDeviceHeatingActuator::setRoomTemperature(uint16_t roomTemperatureRawKnx)
{
    _roomTemperatureRawKnx = roomTemperatureRawKnx;
    auto roomTemperature = _roomChannel.getTemperatureFromRawKnx(roomTemperatureRawKnx);
    logInfoP("Set room temperature: %0.1f °C", roomTemperature);
    // calculateIsActive();
    _piController->setCurrentTemperature(roomTemperature);
}

bool ClimateDeviceHeatingActuator::isActive()
{
    return _isActive;
}

void ClimateDeviceHeatingActuator::logStatus()
{
    logInfoP("Mode %s", ClimateModeSelectionHelper::toString(_mode));
    logInfoP("Is active: %s", _isActive ? "yes" : "no");
    if (_targetTemperatureRawKnx != std::numeric_limits<uint16_t>::max())
        logInfoP("Target temperature: %0.1f°C", _roomChannel.getTemperatureFromRawKnx(_targetTemperatureRawKnx));
    if (_roomTemperatureRawKnx != std::numeric_limits<uint16_t>::max())
        logInfoP("Room temperature: %0.1f°C", _roomChannel.getTemperatureFromRawKnx(_roomTemperatureRawKnx));
    _piController->logStatus(logPrefix());
}

uint8_t ClimateDeviceHeatingActuator::getRoundingParameter()
{
    return 0;
}