#include "HeatingActuatorModule.h"
#include "OpenKNX.h"
#include "ModuleVersionCheck.h"

HeatingActuatorModule openknxHeatingActuatorModule;

HeatingActuatorModule::HeatingActuatorModule()
{
}

HeatingActuatorModule::~HeatingActuatorModule()
{
}

const std::string HeatingActuatorModule::name()
{
    return "Heating";
}

const std::string HeatingActuatorModule::version()
{
    return MODULE_HeatingActuator_Version;
}

void HeatingActuatorModule::processInputKo(GroupObject &ko)
{
    if (ko.asap() != HTA_KoCentralFunction &&
        (ko.asap() < HTA_KoBlockOffset ||
         ko.asap() > HTA_KoBlockOffset + ParamCLI_VisibleChannels * HTA_KoBlockSize - 1))
        return;

    logDebugP("processInputKo");
    logIndentUp();

    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
        _channel[i]->processInputKo(ko);

    logIndentDown();
}

void HeatingActuatorModule::setup(bool configured)
{
#ifdef OPENKNX_GPIO_NUM
    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
    {
        openknx.gpio.pinMode(0x0100 + i, OUTPUT);
        openknx.gpio.digitalWrite(0x0100 + i, LOW);

        openknx.gpio.pinMode(0x0200 + i, INPUT);
    }
#else
    // Wire is initialized by GPIO module, when used
    OPENKNX_GPIO_WIRE.setSDA(OPENKNX_GPIO_SDA);
    OPENKNX_GPIO_WIRE.setSCL(OPENKNX_GPIO_SCL);
    OPENKNX_GPIO_WIRE.begin();
    OPENKNX_GPIO_WIRE.setClock(OPENKNX_GPIO_CLOCK);
#endif

#ifdef OPENKNX_GPIO_WIRE
    if (ina.begin())
    {
        logDebugP("INA219 setup done with address %u", ina.getAddress());

        ina.setBusVoltageRange(16);
        ina.setGain(1);
        ina.setMaxCurrentShunt(0.5, 0.1);
        ina.setModeShuntContinuous();
        delay(1000);

        logDebugP("getBusVoltageRange %u", ina.getBusVoltageRange());
        logDebugP("getGain %u", ina.getGain());
        logDebugP("getBusADC %u", ina.getBusADC());
        logDebugP("getShuntADC %u", ina.getShuntADC());
        logDebugP("getMode %u", ina.getMode());

        logDebugP("isCalibrated %u", ina.isCalibrated());
        logDebugP("getCurrentLSB %.4f", ina.getCurrentLSB());
        logDebugP("getShunt %.4f", ina.getShunt());
        logDebugP("getMaxCurrent %.4f", ina.getMaxCurrent());
    }
    else
        logDebugP("INA219 not found at address %u", ina.getAddress());
#endif

    pinMode(OPENKNX_HTA_MOT_PWR_PIN, OUTPUT);
    digitalWrite(OPENKNX_HTA_MOT_PWR_PIN, MOT_PWR_OFF);

    pinMode(OPENKNX_HTA_MOT_HIGH1_PIN, OUTPUT);
    digitalWrite(OPENKNX_HTA_MOT_HIGH1_PIN, MOT_HIGH1_OFF);
    pinMode(OPENKNX_HTA_MOT_HIGH2_PIN, OUTPUT);
    digitalWrite(OPENKNX_HTA_MOT_HIGH2_PIN, MOT_HIGH2_OFF);
    pinMode(OPENKNX_HTA_MOT_LOW1_PIN, OUTPUT);
    digitalWrite(OPENKNX_HTA_MOT_LOW1_PIN, MOT_LOW1_OFF);
    pinMode(OPENKNX_HTA_MOT_LOW2_PIN, OUTPUT);
    digitalWrite(OPENKNX_HTA_MOT_LOW2_PIN, MOT_LOW2_OFF);

    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
    {
        _channel[i] = new HeatingActuatorChannel(i);
        _channel[i]->setup(configured);
    }
}

void HeatingActuatorModule::loop(bool configured)
{
    if (_motorPower)
    {
        _currentAvg -= _currentAvg / 10;
        _currentAvg += ina.getCurrent_mA() / 10;
        _currentCount++;

        if (_currentCount >= 10) 
        {
            if (_currentAvg > OPENKNX_HTA_CURRENT_MOT_MAX_LIMIT)
            {
                logDebugP("STOP MAX (current: %.2f)", _currentAvg);
                stopMotor();
            }

            if (_currentCount % 10 == 0)
            {
                if (delayCheck(_debugOutputTimer, 1000)) 
                {
                    logDebugP("current: %.2f, last: %.2f", _currentAvg, _currentAvgLast);
                    _debugOutputTimer = delayTimerInit();
                }

                _currentAvgLast = _currentAvg;
            }
        }
    }

    if (!configured)
        return;

    for (uint8_t i = 0; i < MIN(ParamCLI_VisibleChannels, OPENKNX_HTA_CHANNEL_COUNT); i++)
        _channel[i]->loop(_motorPower, _currentCount, _currentAvg, _currentAvgLast);

    processMaxSetValuesAndRequests();
}

void HeatingActuatorModule::processMaxSetValuesAndRequests()
{
    uint8_t maxSetValueHeating = 0;
    uint8_t maxSetValueCooling = 0;
    uint8_t maxSetValueCombined = 0;

    bool requestHeating = false;
    bool requestCooling = false;
    bool requestCombined = false;

    if (ParamHTA_ObjectsMaxSetValueHeating)
        maxSetValueHeating = KoHTA_MaxSetValueHeating.value(DPT_Scaling);

    if (ParamHTA_ObjectsMaxSetValueCooling)
        maxSetValueCooling = KoHTA_MaxSetValueCooling.value(DPT_Scaling);
    
    if (ParamHTA_ObjectsMaxSetValueCombined)
        maxSetValueCombined = KoHTA_MaxSetValueCombined.value(DPT_Scaling);
    
    for (uint8_t i = 0; i < MIN(ParamCLI_VisibleChannels, OPENKNX_HTA_CHANNEL_COUNT); i++)
    {
        if (!_channel[i]->considerForRequestAndMaxSetValue())
            continue;

        if (_channel[i]->isOperationModeHeating())
        {
            if (ParamHTA_ObjectsMaxSetValueHeating)
                maxSetValueHeating = max(maxSetValueHeating, _channel[i]->getSetValueTarget());
            
            if (ParamHTA_ObjectsHeatingCoolingRequest)
                requestHeating = requestHeating || (_channel[i]->getSetValueTarget() > 0);
        }
        else
        {
            if (ParamHTA_ObjectsMaxSetValueCooling)
                maxSetValueCooling = max(maxSetValueHeating, _channel[i]->getSetValueTarget());
            
            if (ParamHTA_ObjectsHeatingCoolingRequest)
                requestCooling = requestCooling || (_channel[i]->getSetValueTarget() > 0);
        }
        
        if (ParamHTA_ObjectsMaxSetValueCombined)
            maxSetValueCombined = max(maxSetValueCombined, _channel[i]->getSetValueTarget());
            
        if (ParamHTA_ObjectsHeatingCoolingRequest)
            requestCombined = requestCombined || (_channel[i]->getSetValueTarget() > 0);
    }

    if (ParamHTA_ObjectsMaxSetValueHeating)
    {
        if (maxSetValueHeating != (uint8_t)KoHTA_MaxSetValueHeatingStatus.value(DPT_Scaling))
            KoHTA_MaxSetValueHeatingStatus.value(maxSetValueHeating, DPT_Scaling);
        
        if (delayCheck(ParamHTA_ObjectsMaxSetValueHeatingCyclicTimeMS, _maxValueHeatingCyclicSendTimer))
        {
            KoHTA_MaxSetValueHeatingStatus.value(maxSetValueHeating, DPT_Scaling);
            _maxValueHeatingCyclicSendTimer = delayTimerInit();
        }
    }

    if (ParamHTA_ObjectsMaxSetValueCooling)
    {
        if (maxSetValueCooling != (uint8_t)KoHTA_MaxSetValueCoolingStatus.value(DPT_Scaling))
            KoHTA_MaxSetValueCoolingStatus.value(maxSetValueCooling, DPT_Scaling);
        
        if (delayCheck(ParamHTA_ObjectsMaxSetValueCoolingCyclicTimeMS, _maxValueCoolingCyclicSendTimer))
        {
            KoHTA_MaxSetValueCoolingStatus.value(maxSetValueCooling, DPT_Scaling);
            _maxValueCoolingCyclicSendTimer = delayTimerInit();
        }
    }

    if (ParamHTA_ObjectsMaxSetValueCombined)
    {
        if (maxSetValueCombined != (uint8_t)KoHTA_MaxSetValueCombinedStatus.value(DPT_Scaling))
            KoHTA_MaxSetValueCombinedStatus.value(maxSetValueCombined, DPT_Scaling);
        
        if (delayCheck(ParamHTA_ObjectsMaxSetValueCombinedCyclicTimeMS, _maxValueCombinedCyclicSendTimer))
        {
            KoHTA_MaxSetValueCombinedStatus.value(maxSetValueCombined, DPT_Scaling);
            _maxValueCombinedCyclicSendTimer = delayTimerInit();
        }
    }

    if (ParamHTA_ObjectsHeatingCoolingRequest)
    {
        if (requestHeating != (bool)KoHTA_RequestHeating.value(DPT_Switch) &&
            (ParamHTA_OperationMode == HTA_OPERATION_MODE_HEATING || ParamHTA_OperationMode == HTA_OPERATION_MODE_HEATCOOLING))
            KoHTA_RequestHeating.value(requestHeating, DPT_Switch);

        if (requestCooling != (bool)KoHTA_RequestCooling.value(DPT_Switch) &&
            (ParamHTA_OperationMode == HTA_OPERATION_MODE_COOLING || ParamHTA_OperationMode == HTA_OPERATION_MODE_HEATCOOLING))
            KoHTA_RequestCooling.value(requestCooling, DPT_Switch);

        if (requestCombined != (bool)KoHTA_RequestCombined.value(DPT_Switch) &&
            ParamHTA_OperationMode == HTA_OPERATION_MODE_HEATCOOLING)
            KoHTA_RequestCombined.value(requestCombined, DPT_Switch);
    }
}

HeatingActuatorChannel* HeatingActuatorModule::getChannel(uint8_t channelIndex)
{
    return _channel[channelIndex];
}

void HeatingActuatorModule::runMotor(uint8_t channelIndex, bool open)
{
    stopMotor();

    if (open)
    {
        digitalWrite(OPENKNX_HTA_MOT_HIGH1_PIN, MOT_HIGH1_OFF);
        digitalWrite(OPENKNX_HTA_MOT_HIGH2_PIN, MOT_HIGH2_ON);
        digitalWrite(OPENKNX_HTA_MOT_LOW1_PIN, MOT_LOW1_ON);
        digitalWrite(OPENKNX_HTA_MOT_LOW2_PIN, MOT_LOW2_OFF);
    }
    else
    {
        digitalWrite(OPENKNX_HTA_MOT_HIGH1_PIN, MOT_HIGH1_ON);
        digitalWrite(OPENKNX_HTA_MOT_HIGH2_PIN, MOT_HIGH2_OFF);
        digitalWrite(OPENKNX_HTA_MOT_LOW1_PIN, MOT_LOW1_OFF);
        digitalWrite(OPENKNX_HTA_MOT_LOW2_PIN, MOT_LOW2_ON);
    }

    _channel[channelIndex]->runMotor(open);
    digitalWrite(OPENKNX_HTA_MOT_PWR_PIN, MOT_PWR_ON);

    _currentCount = 0;
    _currentAvg = 0;
    _currentAvgLast = 0;
    _motorDirectionOpen = open;
    _motorChannelActive = channelIndex;
    _motorPower = true;
}

// there is always only one more running at the same time
void HeatingActuatorModule::stopMotor()
{
    logDebugP("Stop motor");

    digitalWrite(OPENKNX_HTA_MOT_PWR_PIN, MOT_PWR_OFF);
    _motorPower = false;

    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
        _channel[i]->stopMotor();
}

void HeatingActuatorModule::writeFlash()
{
    openknx.flash.writeByte(OPENKNX_HTA_FLASH_VERSION);
    openknx.flash.writeInt(OPENKNX_HTA_FLASH_MAGIC_WORD);

    openknx.flash.writeByte(OPENKNX_HTA_CHANNEL_COUNT);
    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
        _channel[i]->writeChannelData();

    logDebugP("State written to flash");
}

void HeatingActuatorModule::readFlash(const uint8_t *data, const uint16_t size)
{
    if (size == 0)
        return;

    logDebugP("Reading state from flash");
    logIndentUp();

    uint8_t version = openknx.flash.readByte();
    if (version != OPENKNX_HTA_FLASH_VERSION)
    {
        logDebugP("Invalid flash version %u", version);
        return;
    }

    uint32_t magicWord = openknx.flash.readInt();
    if (magicWord != OPENKNX_HTA_FLASH_MAGIC_WORD)
    {
        logDebugP("Flash content invalid");
        return;
    }

    uint8_t channelsStored = openknx.flash.readByte();
    if (channelsStored != OPENKNX_HTA_CHANNEL_COUNT)
    {
        logDebugP("Incompatbile channel count; %u != %u", channelsStored, OPENKNX_HTA_CHANNEL_COUNT);
        return;
    }

    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
        _channel[i]->readChannelData();
    
    logIndentDown();
}

uint16_t HeatingActuatorModule::flashSize()
{
    return 6 + OPENKNX_HTA_CHANNEL_COUNT * 13;
}

void HeatingActuatorModule::savePower()
{
    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
        _channel[i]->savePower();
}

bool HeatingActuatorModule::restorePower()
{
    bool success = true;
    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
        success &= _channel[i]->restorePower();
    
    return true;
}

void HeatingActuatorModule::showHelp()
{
    openknx.console.printHelpLine("hta ch NN opn", "Open valve (run counter-clockwise) of channel index NN (zero-based).");
    openknx.console.printHelpLine("hta ch NN cls", "Close valve (run clockwise) of channel index NN (zero-based).");
    openknx.console.printHelpLine("hta ch NN cal", "Start motor calibration for channel index NN (zero-based).");
    openknx.console.printHelpLine("hta ch NN MMM", "Move valve of channel index NN (zero-based) to position MMM (0-100 %).");
    openknx.console.printHelpLine("hta ch NN info", "Information and state of channel index NN (zero-based).");
    openknx.console.printHelpLine("hta stop", "Stop all motors (only one running at the same time).");
}

bool HeatingActuatorModule::processCommand(const std::string cmd, bool diagnoseKo)
{
    bool result = false;

    if (cmd.substr(0, 3) != "hta" || cmd.length() < 5)
        return result;

    if (cmd.length() == 5 && cmd.substr(4, 1) == "h")
    {
        openknx.console.writeDiagenoseKo("-> ch NN cal");
        openknx.console.writeDiagenoseKo("");
        openknx.console.writeDiagenoseKo("-> ch NN opn");
        openknx.console.writeDiagenoseKo("");
        openknx.console.writeDiagenoseKo("-> ch NN cls");
        openknx.console.writeDiagenoseKo("");
        openknx.console.writeDiagenoseKo("-> ch NN MMM");
        openknx.console.writeDiagenoseKo("");
        openknx.console.writeDiagenoseKo("-> ch NN info");
        openknx.console.writeDiagenoseKo("");
        openknx.console.writeDiagenoseKo("-> stop");
        openknx.console.writeDiagenoseKo("");
    }
    else if (cmd.length() == 8 && cmd.substr(4, 4) == "stop")
    {
        stopMotor();
        result = true;
    }
    else if (cmd.length() > 7 && cmd.substr(4, 2) == "ch")
    {
        if (cmd.length() == 13 && cmd.substr(10, 3) == "opn")
        {
            uint8_t channelIndex = stoi(cmd.substr(7, 2));
            _channel[channelIndex]->setTargetPosition(HTA_POSITION_FULLY_OPEN);
            runMotor(channelIndex, true);
            result = true;
        }
        else if (cmd.length() == 13 && cmd.substr(10, 3) == "cls")
        {
            uint8_t channelIndex = stoi(cmd.substr(7, 2));
            _channel[channelIndex]->setTargetPosition(HTA_POSITION_FULLY_CLOSED);
            runMotor(channelIndex, false);
            result = true;
        }
        else if (cmd.length() == 13 && cmd.substr(10, 3) == "cal")
        {
            uint8_t channelIndex = stoi(cmd.substr(7, 2));
            _channel[channelIndex]->startCalibration();
            result = true;
        }
        else if (cmd.length() == 14 && cmd.substr(10, 4) == "info")
        {
            uint8_t channelIndex = stoi(cmd.substr(7, 2));
            _channel[channelIndex]->logChannelInfo(diagnoseKo);
            result = true;
        }
        else if (cmd.length() > 10)
        {
            uint8_t channelIndex = stoi(cmd.substr(7, 2));
            uint8_t targetPercent = stoi(cmd.substr(10));
            _channel[channelIndex]->moveValveToPosition(targetPercent / 100.0);
            result = true;
        }
    }

    return result;
}