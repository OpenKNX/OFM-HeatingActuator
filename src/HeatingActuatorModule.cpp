#include "HeatingActuatorModule.h"
#include "OpenKNX.h"
#include "ModuleVersionCheck.h"

HeatingActuatorModule openknxHeatingActuatorModule;

// parses a decimal number without throwing, std::stoi is not usable with disabled exceptions
static bool parseNumber(const std::string text, uint16_t &value)
{
    if (text.empty())
        return false;

    uint32_t parsed = 0;
    for (const char digit : text)
    {
        if (digit < '0' || digit > '9')
            return false;

        parsed = parsed * 10 + (digit - '0');
        if (parsed > UINT16_MAX)
            return false;
    }

    value = (uint16_t)parsed;
    return true;
}

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
    const uint16_t asap = ko.asap();

    // module wide objects (central function, heating/cooling change over, max set values, requests)
    const bool isModuleKo = asap >= HTA_KoCentralFunction && asap <= HTA_KoRequestCombined;
    const bool isChannelKo = asap >= HTA_KoBlockOffset &&
                             asap < HTA_KoBlockOffset + OPENKNX_HTA_CHANNEL_COUNT * HTA_KoBlockSize;

    if (!isModuleKo && !isChannelKo)
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

#ifdef OPENKNX_HTA_CURRENT_INA_ADDR
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
        logErrorP("INA219 not found at address %u, motor current monitoring is not available", ina.getAddress());
#else
    logErrorP("No current sensor configured, motor current monitoring is not available");
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
    // motor protection has to work even if the device is not configured yet
    processCurrentMeasurement();

    if (!configured)
        return;

    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
        _channel[i]->loop(_motorPower, _currentCount, _currentAvg, _currentAvgLast);

    processMaxSetValuesAndRequests();
}

void HeatingActuatorModule::processCurrentMeasurement()
{
#ifdef OPENKNX_HTA_CURRENT_INA_ADDR
    if (!_motorPower)
        return;

    _currentAvg -= _currentAvg / 10;
    _currentAvg += ina.getCurrent_mA() / 10;
    _currentCount++;

    // the inrush current directly after motor start is far above the operating current
    // and must not be mistaken for a blocked motor
    if (_currentCount >= HTA_MOT_CURRENT_SETTLE_COUNT &&
        _currentAvg > OPENKNX_HTA_CURRENT_MOT_MAX_LIMIT)
    {
        logErrorP("STOP: motor current above hardware limit (current: %.2f mA, limit: %.2f mA)",
                  _currentAvg, (float)OPENKNX_HTA_CURRENT_MOT_MAX_LIMIT);
        stopMotor(MotorStopReason::Overcurrent);
        return;
    }

    // keep a slightly delayed copy of the average, the channels use it to detect a rising current
    if (_currentCount % 10 == 0)
    {
        if (delayCheck(_debugOutputTimer, 1000))
        {
            logDebugP("current: %.2f, last: %.2f", _currentAvg, _currentAvgLast);
            _debugOutputTimer = delayTimerInit();
        }

        _currentAvgLast = _currentAvg;
    }
#endif
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

    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
    {
        if (!_channel[i]->considerForRequestAndMaxSetValue())
            continue;

        const uint8_t setValueTarget = _channel[i]->getSetValueTarget();

        if (_channel[i]->isOperationModeHeating())
        {
            if (ParamHTA_ObjectsMaxSetValueHeating)
                maxSetValueHeating = MAX(maxSetValueHeating, setValueTarget);

            if (ParamHTA_ObjectsHeatingCoolingRequest)
                requestHeating = requestHeating || setValueTarget > 0;
        }
        else
        {
            if (ParamHTA_ObjectsMaxSetValueCooling)
                maxSetValueCooling = MAX(maxSetValueCooling, setValueTarget);

            if (ParamHTA_ObjectsHeatingCoolingRequest)
                requestCooling = requestCooling || setValueTarget > 0;
        }

        if (ParamHTA_ObjectsMaxSetValueCombined)
            maxSetValueCombined = MAX(maxSetValueCombined, setValueTarget);

        if (ParamHTA_ObjectsHeatingCoolingRequest)
            requestCombined = requestCombined || setValueTarget > 0;
    }

    if (ParamHTA_ObjectsMaxSetValueHeating)
    {
        if (maxSetValueHeating != (uint8_t)KoHTA_MaxSetValueHeatingStatus.value(DPT_Scaling))
            KoHTA_MaxSetValueHeatingStatus.value(maxSetValueHeating, DPT_Scaling);

        if (ParamHTA_ObjectsMaxSetValueHeatingCyclicTimeMS > 0 &&
            delayCheck(_maxValueHeatingCyclicSendTimer, ParamHTA_ObjectsMaxSetValueHeatingCyclicTimeMS))
        {
            KoHTA_MaxSetValueHeatingStatus.value(maxSetValueHeating, DPT_Scaling);
            _maxValueHeatingCyclicSendTimer = delayTimerInit();
        }
    }

    if (ParamHTA_ObjectsMaxSetValueCooling)
    {
        if (maxSetValueCooling != (uint8_t)KoHTA_MaxSetValueCoolingStatus.value(DPT_Scaling))
            KoHTA_MaxSetValueCoolingStatus.value(maxSetValueCooling, DPT_Scaling);

        if (ParamHTA_ObjectsMaxSetValueCoolingCyclicTimeMS > 0 &&
            delayCheck(_maxValueCoolingCyclicSendTimer, ParamHTA_ObjectsMaxSetValueCoolingCyclicTimeMS))
        {
            KoHTA_MaxSetValueCoolingStatus.value(maxSetValueCooling, DPT_Scaling);
            _maxValueCoolingCyclicSendTimer = delayTimerInit();
        }
    }

    if (ParamHTA_ObjectsMaxSetValueCombined)
    {
        if (maxSetValueCombined != (uint8_t)KoHTA_MaxSetValueCombinedStatus.value(DPT_Scaling))
            KoHTA_MaxSetValueCombinedStatus.value(maxSetValueCombined, DPT_Scaling);

        if (ParamHTA_ObjectsMaxSetValueCombinedCyclicTimeMS > 0 &&
            delayCheck(_maxValueCombinedCyclicSendTimer, ParamHTA_ObjectsMaxSetValueCombinedCyclicTimeMS))
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
    if (channelIndex >= OPENKNX_HTA_CHANNEL_COUNT)
        return nullptr;

    return _channel[channelIndex];
}

bool HeatingActuatorModule::runMotor(uint8_t channelIndex, bool open)
{
    if (channelIndex >= OPENKNX_HTA_CHANNEL_COUNT)
        return false;

    // the H-bridge and its power supply are shared, only one motor can run at a time
    if (_motorPower)
        return false;

    // let the supply settle before the next start, especially when changing motor direction
    if (!delayCheck(_motorStoppedAt, HTA_MOT_RESTART_DELAY))
        return false;

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

    _currentCount = 0;
    _currentAvg = 0;
    _currentAvgLast = 0;
    _motorChannelActive = channelIndex;
    _motorPower = true;

    _channel[channelIndex]->runMotor(open);
    digitalWrite(OPENKNX_HTA_MOT_PWR_PIN, MOT_PWR_ON);

    return true;
}

// there is always only one motor running at the same time
void HeatingActuatorModule::stopMotor(MotorStopReason reason)
{
    digitalWrite(OPENKNX_HTA_MOT_PWR_PIN, MOT_PWR_OFF);

    const bool wasRunning = _motorPower;
    _motorPower = false;

    if (wasRunning)
    {
        _motorStoppedAt = delayTimerInit();
        _channel[_motorChannelActive]->stopMotor(reason);
    }

    // make sure no channel output stays active
    for (uint8_t i = 0; i < OPENKNX_HTA_CHANNEL_COUNT; i++)
        if (_channel[i] != nullptr)
            _channel[i]->motorOutputOff();
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

    const uint8_t version = openknx.flash.readByte();
    const uint32_t magicWord = openknx.flash.readInt();
    const uint8_t channelsStored = openknx.flash.readByte();

    if (version != OPENKNX_HTA_FLASH_VERSION)
        logDebugP("Invalid flash version %u", version);
    else if (magicWord != OPENKNX_HTA_FLASH_MAGIC_WORD)
        logDebugP("Flash content invalid");
    else if (channelsStored != OPENKNX_HTA_CHANNEL_COUNT)
        logDebugP("Incompatible channel count; %u != %u", channelsStored, OPENKNX_HTA_CHANNEL_COUNT);
    else
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

    return success;
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
    if (cmd.length() < 5 || cmd.compare(0, 4, "hta ") != 0)
        return false;

    const std::string args = cmd.substr(4);

    if (args == "h")
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
        return true;
    }

    if (args == "stop")
    {
        stopMotor();
        return true;
    }

    // "ch NN <command>"
    if (args.length() < 7 || args.compare(0, 3, "ch ") != 0)
        return false;

    uint16_t channelIndex = 0;
    if (!parseNumber(args.substr(3, 2), channelIndex) ||
        channelIndex >= OPENKNX_HTA_CHANNEL_COUNT)
    {
        logInfoP("Invalid channel index, valid range is 0-%u", OPENKNX_HTA_CHANNEL_COUNT - 1);
        return true;
    }

    HeatingActuatorChannel *channel = _channel[channelIndex];
    const std::string channelCommand = args.substr(6);

    if (channelCommand == "opn")
        channel->driveToEndStop(true);
    else if (channelCommand == "cls")
        channel->driveToEndStop(false);
    else if (channelCommand == "cal")
        channel->startCalibration();
    else if (channelCommand == "info")
        channel->logChannelInfo(diagnoseKo);
    else
    {
        uint16_t targetPercent = 0;
        if (!parseNumber(channelCommand, targetPercent) || targetPercent > 100)
        {
            logInfoP("Invalid target position, valid range is 0-100 %%");
            return true;
        }

        channel->moveValveToPosition(targetPercent / 100.0f);
    }

    return true;
}
