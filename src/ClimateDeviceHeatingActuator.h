#pragma once
#include "ClimateDevice.h"

class RoomChannel;
class PIController;
class PWMController;
class TargetTemperatureManipulationController;

class ClimateDeviceHeatingActuator : public ClimateDevice
{
    private:
        uint16_t _targetTemperatureRawKnx = std::numeric_limits<uint16_t>::max();
        uint16_t _roomTemperatureRawKnx = std::numeric_limits<uint16_t>::max();
        ClimateModeSelection _mode = ClimateModeSelection::Undefined;
        bool _isActive = false;
        PIController* _piController = nullptr;
        void setIsActive(bool active);
    public:
        ClimateDeviceHeatingActuator(int channelIndex, int deviceIndex, RoomChannel& roomChannel);
        void loop() override;
        void setMode(ClimateModeSelection mode) override;
        void setTargetTemperature(uint16_t targetTemperatureRawKnx) override;
        void setRoomTemperature(uint16_t roomTemperatureRawKnx) override;
        void logStatus() override;
        void processInputKo(GroupObject &ko) override;
        uint8_t getRoundingParameter() override;
        bool isActive() override;
        ClimateModeSelection currentMode() override;
};

