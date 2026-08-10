#include "HeatingActuatorChannel.h"
#include "HeatingActuatorModule.h"
#include <cmath>

// checks one scene and applies it; all scenes are configured with identical parameters,
// only the scene letter differs
#define HTA_CHECK_SCENE(letter)                                                              \
    if (ParamHTA_ChScene##letter##Active &&                                                  \
        ParamHTA_ChScene##letter##Number > 0 &&                                              \
        (uint8_t)(ParamHTA_ChScene##letter##Number - 1) == sceneNumber)                       \
    {                                                                                        \
        logDebugP("processScene: scene " #letter);                                           \
        applyScene(ParamHTA_ChScene##letter##ChangeHvacMode,                                 \
                   ParamHTA_ChScene##letter##HvacMode,                                       \
                   ParamHTA_ChScene##letter##ChangeTargetTempInput,                          \
                   ParamHTA_ChScene##letter##TargetTemp,                                     \
                   ParamHTA_ChScene##letter##ChangeTargetTempShift,                          \
                   ParamHTA_ChScene##letter##TargetTempShift);                               \
        return;                                                                              \
    }

HeatingActuatorChannel::HeatingActuatorChannel(uint8_t channelNumber)
{
    _channelIndex = channelNumber;
}

HeatingActuatorChannel::~HeatingActuatorChannel() {}

const std::string HeatingActuatorChannel::name()
{
    return "HeatingChannel";
}

void HeatingActuatorChannel::processInputKo(GroupObject &ko)
{
    if (!ParamHTA_ChActive)
        return;

    // module wide objects
    switch (ko.asap())
    {
        case HTA_KoOperationMode:
        case HTA_KoSummerWinter:
            checkOperationMode();
            return;
    }

    switch (HTA_KoCalcIndex(ko.asap()))
    {
        case HTA_KoChEnforcedPosition:
            _externEnforcedPosition = ko.value(DPT_Switch);
            logDebugP("HTA_KoChEnforcedPosition: %u", _externEnforcedPosition);
            break;
        case HTA_KoChSetValueInput:
            _externSetValuePercent = (uint8_t)(ko.value(DPT_Scaling)) / 100.0f;
            _lastExternValue = delayTimerInit();
            logDebugP("HTA_KoChSetValueInput: %.2f", _externSetValuePercent);
            break;
        case HTA_KoChRoomTempInput:
            _externRoomTemp = ko.value(DPT_Value_Temp);
            _lastExternValue = delayTimerInit();
            logDebugP("HTA_KoChRoomTempInput: %.2f", _externRoomTemp);
            break;
        case HTA_KoChTargetTempInput:
        {
            const float newTargetTemp = ko.value(DPT_Value_Temp);
            logDebugP("HTA_KoChTargetTempInput: %.2f", newTargetTemp);
            setTargetTemp(newTargetTemp);
            break;
        }
        case HTA_KoChTargetTempShiftInput:
        {
            const float newTargetTempShift = ko.value(DPT_Value_Temp);
            logDebugP("HTA_KoChTargetTempShiftInput: %.2f", newTargetTempShift);
            checkTargetTempShift(newTargetTempShift);
            break;
        }
        case HTA_KoChTargetTempShiftStep:
        {
            const bool stepUp = ko.value(DPT_Switch);
            const float stepSize = targetTempShiftStepSize(ParamHTA_ChTargetTempShift);
            logDebugP("HTA_KoChTargetTempShiftStep: %u (step size: %.1f)", stepUp, stepSize);

            checkTargetTempShift(_externTargetTempShift + (stepUp ? stepSize : -stepSize));
            break;
        }
        case HTA_KoChHvacModeInput:
        case HTA_KoChHvacModeInputComfort:
        case HTA_KoChHvacModeInputNight:
        case HTA_KoChHvacModeInputProtect:
            checkHvacMode();
            break;
        case HTA_KoChTargetTempLockHeating:
            logDebugP("HTA_KoChTargetTempLockHeating: %u", (bool)ko.value(DPT_Switch));
            if ((bool)KoHTA_ChTargetTempLockHeatingStatus.value(DPT_Switch) != (bool)ko.value(DPT_Switch))
                KoHTA_ChTargetTempLockHeatingStatus.value(ko.value(DPT_Switch), DPT_Switch);
            break;
        case HTA_KoChTargetTempLockCooling:
            logDebugP("HTA_KoChTargetTempLockCooling: %u", (bool)ko.value(DPT_Switch));
            if ((bool)KoHTA_ChTargetTempLockCoolingStatus.value(DPT_Switch) != (bool)ko.value(DPT_Switch))
                KoHTA_ChTargetTempLockCoolingStatus.value(ko.value(DPT_Switch), DPT_Switch);
            break;
        case HTA_KoChManualMode:
            logDebugP("HTA_KoChManualMode: %u", (bool)ko.value(DPT_Switch));
            setManualMode(ko.value(DPT_Switch), _currentManualModeOn);
            break;
        case HTA_KoChScene:
            // only react on scene calls, scene learning is not supported
            if ((uint8_t)ko.value(Dpt(18, 1, 0)) == 0)
                processScene(ko.value(Dpt(18, 1, 1)));
            break;
    }
}

void HeatingActuatorChannel::setup(bool configured)
{
    logDebugP("Setup channel %u", _channelIndex);

    // preset PIN state before changing PIN mode
    digitalWriteFast(MOTOR_PINS[_channelIndex], MOT_OFF);
    pinMode(MOTOR_PINS[_channelIndex], OUTPUT);

    // set it again the standard way, just in case
    motorOutputOff();

    if (!configured)
        return;

    // if the operation mode is fixed by parameters, it is known without any telegram;
    // otherwise it stays at its default until the change over object is received
    if (isOperationModeFixed())
        checkOperationMode();

    if (ParamHTA_ChSetValueChangeSend && ParamHTA_ChSetValueCyclicTimeMS > 0)
        _setValueCyclicSendTimer = delayTimerInit();
    if (ParamHTA_ChTargetTempChangeSend && ParamHTA_ChTargetTempCyclicTimeMS > 0)
        _targetTempCyclicSendTimer = delayTimerInit();
    if (ParamHTA_ChEmergencyModeChangeSend && ParamHTA_ChEmergencyModeCyclicTimeMS > 0)
        _emergencyModeCyclicSendTimer = delayTimerInit();
    if (ParamHTA_ChManualModeChangeSend && ParamHTA_ChManualModeCyclicTimeMS > 0)
        _manualModeCyclicSendTimer = delayTimerInit();

    _lastExternValue = delayTimerInit();
}

void HeatingActuatorChannel::loop(bool motorPower, uint32_t currentCount, float current, float currentLast)
{
    if (!ParamHTA_ChActive)
        return;

    processInput();
    processCyclicSending();

    if (_motorState != MotorState::MOT_IDLE)
        processRunningMotor(currentCount, current, currentLast);
    else if (!motorPower)
        // the motor of another channel is running, this channel has to wait
        processIdle();

    processOutput();
}

//
// operation mode (heating/cooling)
//

bool HeatingActuatorChannel::isOperationModeFixed()
{
    return ParamHTA_OperationMode != HTA_OPERATION_MODE_HEATCOOLING ||
           ParamHTA_ChOperationMode != HTA_OPERATION_MODE_HEATCOOLING;
}

void HeatingActuatorChannel::checkOperationMode()
{
    // the channel can only narrow down the device wide operation mode
    uint8_t operationMode = ParamHTA_OperationMode;
    if (operationMode == HTA_OPERATION_MODE_HEATCOOLING)
        operationMode = ParamHTA_ChOperationMode;

    bool newOperationModeHeating;
    switch (operationMode)
    {
        case HTA_OPERATION_MODE_HEATING:
            newOperationModeHeating = true;
            break;
        case HTA_OPERATION_MODE_COOLING:
            newOperationModeHeating = false;
            break;
        default:
            if (ParamHTA_OperationModeChange == HTA_OPERATION_MODE_CHANGE_OBJECT_HEATING_COOLING)
                newOperationModeHeating = KoHTA_OperationMode.value(DPT_Switch);
            else
                newOperationModeHeating = !KoHTA_SummerWinter.value(DPT_Switch);
            break;
    }

    setOperationMode(newOperationModeHeating);
}

void HeatingActuatorChannel::setOperationMode(bool newOperationModeHeating)
{
    if (_currentOperationModeHeating == newOperationModeHeating)
        return;

    // the regulator has to be re-tuned for the new operation mode
    if (pid.isRunning())
    {
        pid.stop();
        pid.reset();
    }

    _currentOperationModeHeating = newOperationModeHeating;
    logDebugP("setOperationMode (heating=%u)", _currentOperationModeHeating);
}

bool HeatingActuatorChannel::isOperationModeHeating()
{
    return _currentOperationModeHeating;
}

//
// HVAC mode and target temperature
//

void HeatingActuatorChannel::checkHvacMode()
{
    const HvacMode externHvacMode = static_cast<HvacMode>((uint8_t)KoHTA_ChHvacModeInput.value(DPT_HVACMode));
    const bool externHvacComfort = KoHTA_ChHvacModeInputComfort.value(DPT_Switch);
    const bool externHvacNight = KoHTA_ChHvacModeInputNight.value(DPT_Switch);
    const bool externHvacProtect = KoHTA_ChHvacModeInputProtect.value(DPT_Switch);

    logDebugP("checkHvacMode (mode=%u, comfort=%u, night=%u, protect=%u)",
              externHvacMode, externHvacComfort, externHvacNight, externHvacProtect);

    HvacMode newHvacMode = HvacMode::HVAC_NONE;

    if (externHvacMode == HvacMode::HVAC_PROTECT || externHvacProtect)
        newHvacMode = HvacMode::HVAC_PROTECT;
    else if (ParamHTA_ChHvacModePriority == 0)
    {
        // Protect>Comfort>Night>Standby

        if (externHvacMode == HvacMode::HVAC_COMFORT || externHvacComfort)
            newHvacMode = HvacMode::HVAC_COMFORT;
        else if (externHvacMode == HvacMode::HVAC_NIGHT || externHvacNight)
            newHvacMode = HvacMode::HVAC_NIGHT;
    }
    else
    {
        // Protect>Night>Comfort>Standby

        if (externHvacMode == HvacMode::HVAC_NIGHT || externHvacNight)
            newHvacMode = HvacMode::HVAC_NIGHT;
        else if (externHvacMode == HvacMode::HVAC_COMFORT || externHvacComfort)
            newHvacMode = HvacMode::HVAC_COMFORT;
    }

    if (newHvacMode == HvacMode::HVAC_NONE)
        newHvacMode = HvacMode::HVAC_STANDBY;

    setHvacMode(newHvacMode);
}

void HeatingActuatorChannel::setHvacMode(HvacMode newHvacMode)
{
    if (_currentHvacMode == newHvacMode)
        return;

    _currentHvacMode = newHvacMode;
    logDebugP("setHvacMode (_currentHvacMode=%u)", _currentHvacMode);

    if (ParamHTA_ChTargetTempResetOnHvacModeChange)
        _externTargetTemp = HTA_TEMPERATUR_INVALID;
    if (ParamHTA_ChTargetTempShiftResetOnHvacModeChange && _externTargetTempShift != 0)
    {
        _externTargetTempShift = 0;
        KoHTA_ChTargetTempShiftStatus.value(_externTargetTempShift, DPT_Value_Temp);
    }

    if ((uint8_t)KoHTA_ChHvacModeStatus.value(DPT_HVACMode) != newHvacMode)
        KoHTA_ChHvacModeStatus.value(newHvacMode, DPT_HVACMode);
}

float HeatingActuatorChannel::targetTempShiftStepSize(uint8_t step)
{
    switch (step)
    {
        case 0:
            return 0.1f;
        case 1:
            return 0.2f;
        case 2:
            return 0.5f;
        default:
            return 1.0f;
    }
}

bool HeatingActuatorChannel::isTargetTempLocked()
{
    if (_currentOperationModeHeating)
        return KoHTA_ChTargetTempLockHeating.value(DPT_Switch);

    return KoHTA_ChTargetTempLockCooling.value(DPT_Switch);
}

void HeatingActuatorChannel::setTargetTemp(float newTargetTemp)
{
    if (isTargetTempLocked())
    {
        logDebugP("Target temperature locked, ignore setTargetTemp.");
        return;
    }

    if (_externTargetTemp == newTargetTemp)
        return;

    _externTargetTemp = newTargetTemp;

    if (ParamHTA_ChTargetTempShiftResetOnNewTargetTemp)
        setTargetTempShift(0);
}

void HeatingActuatorChannel::checkTargetTempShift(float newTargetTempShift)
{
    const float maxTargetTempShift = ParamHTA_ChTargetTempShiftMax;
    if (newTargetTempShift > maxTargetTempShift)
        newTargetTempShift = maxTargetTempShift;
    else if (newTargetTempShift < -maxTargetTempShift)
        newTargetTempShift = -maxTargetTempShift;

    switch (_currentHvacMode)
    {
        case HvacMode::HVAC_COMFORT:
            if (ParamHTA_ChTargetTempShiftApplyToComfort)
                setTargetTempShift(newTargetTempShift);
            break;
        case HvacMode::HVAC_NIGHT:
            // changing the shift can additionally switch over to comfort mode
            if (ParamHTA_ChTargetTempShiftActionNight)
                setHvacMode(HvacMode::HVAC_COMFORT);

            if (ParamHTA_ChTargetTempShiftApplyToNight || ParamHTA_ChTargetTempShiftActionNight)
                setTargetTempShift(newTargetTempShift);
            break;
        case HvacMode::HVAC_STANDBY:
            if (ParamHTA_ChTargetTempShiftActionStandby)
                setHvacMode(HvacMode::HVAC_COMFORT);

            if (ParamHTA_ChTargetTempShiftApplyToStandby || ParamHTA_ChTargetTempShiftActionStandby)
                setTargetTempShift(newTargetTempShift);
            break;
        default:
            break;
    }
}

void HeatingActuatorChannel::setTargetTempShift(float newTargetTempShift)
{
    if (isTargetTempLocked())
    {
        logDebugP("Target temperature locked, ignore setTargetTempShift.");
        return;
    }

    if (_externTargetTempShift == newTargetTempShift)
        return;

    _externTargetTempShift = newTargetTempShift;
    logDebugP("setTargetTempShift (_externTargetTempShift=%.2f)", _externTargetTempShift);

    KoHTA_ChTargetTempShiftStatus.value(_externTargetTempShift, DPT_Value_Temp);
}

void HeatingActuatorChannel::checkEmergencyMode()
{
    const bool newEmergencyMode =
        ParamHTA_ChEmergencyMode &&
        delayCheck(_lastExternValue, ParamHTA_ChEmergencyModeDelayTimeMS);

    if (_currentEmergencyMode == newEmergencyMode)
        return;

    _currentEmergencyMode = newEmergencyMode;
    logDebugP("checkEmergencyMode (_currentEmergencyMode=%u)", _currentEmergencyMode);

    KoHTA_ChEmergencyModeStatus.value(_currentEmergencyMode, DPT_Switch);
}

//
// manual mode
//

void HeatingActuatorChannel::setManualMode(bool manualMode, bool manualModeOn)
{
    // restart the automatic switch back timer whenever manual mode is entered
    if (manualMode && !_currentManualMode)
        _currentManualModeStarted = delayTimerInit();

    _currentManualMode = manualMode;
    _currentManualModeOn = manualModeOn;
}

//
// scenes
//

void HeatingActuatorChannel::processScene(uint8_t sceneNumber)
{
    if (!ParamHTA_ChScenesActive)
        return;

    // the scene number parameter is one based ("scene 1"), 0 means unused,
    // the scene number on the bus is zero based
    HTA_CHECK_SCENE(A)
    HTA_CHECK_SCENE(B)
    HTA_CHECK_SCENE(C)
    HTA_CHECK_SCENE(D)
    HTA_CHECK_SCENE(E)
    HTA_CHECK_SCENE(F)
    HTA_CHECK_SCENE(G)
    HTA_CHECK_SCENE(H)
    HTA_CHECK_SCENE(I)
    HTA_CHECK_SCENE(J)
    HTA_CHECK_SCENE(K)
    HTA_CHECK_SCENE(L)
}

void HeatingActuatorChannel::applyScene(bool changeHvacMode, uint8_t hvacMode,
                                        bool changeTargetTemp, int8_t targetTemp,
                                        bool changeTargetTempShift, uint8_t targetTempShiftStep)
{
    // the HVAC mode parameter starts at comfort, while HvacMode starts at HVAC_NONE
    if (changeHvacMode)
        setHvacMode(static_cast<HvacMode>(hvacMode + 1));
    if (changeTargetTemp)
        setTargetTemp(targetTemp);
    if (changeTargetTempShift)
        setTargetTempShift(targetTempShiftStepSize(targetTempShiftStep));
}

//
// motor control
//
// The H-bridge and its power supply are shared by all channels, so the module decides
// which channel may run its motor. runMotor()/stopMotor() are therefore only called by
// HeatingActuatorModule, the channel asks for the motor via requestMotor().
//

void HeatingActuatorChannel::runMotor(bool open)
{
    digitalWrite(MOTOR_PINS[_channelIndex], MOT_ON);

    _motorStarted = delayTimerInit();
    _motorRunTime = 0;
    _motorStopReason = MotorStopReason::Unknown;
    _motorState = open ? MotorState::MOT_OPENING : MotorState::MOT_CLOSING;

    logDebugP("Run motor (%s)", open ? "opening" : "closing");
}

void HeatingActuatorChannel::stopMotor(MotorStopReason reason)
{
    motorOutputOff();

    if (_motorState == MotorState::MOT_IDLE)
        return;

    const MotorState motorState = _motorState;
    _motorState = MotorState::MOT_IDLE;
    _motorRunTime = millis() - _motorStarted;
    _motorStopped = delayTimerInit();
    _motorStopReason = reason;

    // the tracked position has to stay in sync, no matter why the motor was stopped
    if (reason == MotorStopReason::EndStop)
        _currentPositionPercent = motorState == MotorState::MOT_OPENING ? HTA_POSITION_FULLY_OPEN : HTA_POSITION_FULLY_CLOSED;
    else
        applyMotorTravel(motorState, _motorRunTime);

    switch (reason)
    {
        case MotorStopReason::TargetReached:
            // nothing left to do
            _moveRequested = false;
            break;
        case MotorStopReason::NoMotor:
        case MotorStopReason::Overcurrent:
        case MotorStopReason::Timeout:
            // the movement cannot be completed, give up until a new set value arrives
            _moveRequested = false;
            break;
        default:
            // interrupted or stopped at an end stop, a pending movement is continued
            break;
    }

    logDebugP("Stop motor (reason: %u, runTime: %u ms, position: %.4f)",
              (uint8_t)reason, _motorRunTime, _currentPositionPercent);
}

void HeatingActuatorChannel::motorOutputOff()
{
    digitalWrite(MOTOR_PINS[_channelIndex], MOT_OFF);
}

bool HeatingActuatorChannel::requestMotor(bool open)
{
    return openknxHeatingActuatorModule.runMotor(_channelIndex, open);
}

uint32_t HeatingActuatorChannel::calibratedDriveTime(MotorState motorState)
{
    return motorState == MotorState::MOT_OPENING ? _calibratedDriveOpenTime : _calibratedDriveCloseTime;
}

uint32_t HeatingActuatorChannel::maxMotorRunTime()
{
    if (_calibrationState == CalibrationState::CAL_COMPLETE)
    {
        const uint32_t calibratedTime = calibratedDriveTime(_motorState);
        if (calibratedTime > 0)
            return calibratedTime + calibratedTime / 4 + HTA_MOT_RUNTIME_MARGIN;
    }

    return HTA_MOT_MAX_DRIVE_TIME;
}

void HeatingActuatorChannel::applyMotorTravel(MotorState motorState, uint32_t runTime)
{
    if (_calibrationState != CalibrationState::CAL_COMPLETE ||
        _currentPositionPercent == HTA_POSITION_INVALID)
        return;

    const uint32_t calibratedTime = calibratedDriveTime(motorState);
    if (calibratedTime == 0)
        return;

    const float travelled = runTime / (float)calibratedTime;
    if (motorState == MotorState::MOT_OPENING)
    {
        _currentPositionPercent += travelled;
        if (_currentPositionPercent > HTA_POSITION_FULLY_OPEN)
            _currentPositionPercent = HTA_POSITION_FULLY_OPEN;
    }
    else
    {
        _currentPositionPercent -= travelled;
        if (_currentPositionPercent < HTA_POSITION_FULLY_CLOSED)
            _currentPositionPercent = HTA_POSITION_FULLY_CLOSED;
    }
}

void HeatingActuatorChannel::processRunningMotor(uint32_t currentCount, float current, float currentLast)
{
    const uint32_t runTime = millis() - _motorStarted;

    // last resort protection, e.g. if the current measurement does not work
    if (runTime >= maxMotorRunTime())
    {
        logErrorP("STOP: maximum motor run time exceeded (runTime: %u ms)", runTime);
        openknxHeatingActuatorModule.stopMotor(MotorStopReason::Timeout);
        return;
    }

    // the inrush current directly after motor start is far above the operating current,
    // so current based decisions are only possible once the measurement has settled
    if (currentCount >= HTA_MOT_CURRENT_SETTLE_COUNT)
    {
        if (current < OPENKNX_HTA_CURRENT_MOT_MIN_LIMIT)
        {
            logErrorP("STOP: no motor connected (current: %.2f mA, min: %.2f mA)",
                      current, (float)OPENKNX_HTA_CURRENT_MOT_MIN_LIMIT);
            openknxHeatingActuatorModule.stopMotor(MotorStopReason::NoMotor);
            return;
        }

        // a rising current above the configured maximum means the valve reached its end stop
        const uint8_t motorMaxCurrent = _motorState == MotorState::MOT_OPENING
                                            ? ParamHTA_ChMotorMaxCurrentOpen
                                            : ParamHTA_ChMotorMaxCurrentClose;
        if (currentLast > 0 &&
            current > currentLast + HTA_MOT_CURRENT_RISE &&
            current > motorMaxCurrent)
        {
            logDebugP("STOP: end stop reached (current: %.2f, last: %.2f, limit: %u)",
                      current, currentLast, motorMaxCurrent);
            openknxHeatingActuatorModule.stopMotor(MotorStopReason::EndStop);
            return;
        }
    }

    // moving the valve to a calculated position, everything else runs into the end stop
    if (!_moveRequested ||
        _calibrationState != CalibrationState::CAL_COMPLETE ||
        _targetPositionPercent == HTA_POSITION_INVALID)
        return;

    const uint32_t calibratedTime = calibratedDriveTime(_motorState);
    if (calibratedTime == 0)
        return;

    const float travelled = runTime / (float)calibratedTime;
    const bool targetReached = _motorState == MotorState::MOT_OPENING
                                   ? _currentPositionPercent + travelled >= _targetPositionPercent
                                   : _currentPositionPercent - travelled <= _targetPositionPercent;
    if (targetReached)
        openknxHeatingActuatorModule.stopMotor(MotorStopReason::TargetReached);
}

void HeatingActuatorChannel::processIdle()
{
    switch (_calibrationState)
    {
        case CalibrationState::CAL_INIT:
        case CalibrationState::CAL_OPENING:
        case CalibrationState::CAL_CLOSING:
            processCalibration();
            return;
        default:
            break;
    }

    // a pending movement has priority over a newly calculated set value
    if (processMove())
        return;

    calculateNewSetValue();
}

//
// calibration
//
// The valve is first closed completely to get a defined starting point, then it is opened
// and closed completely again while the drive times are measured. Every step has to end at
// a mechanical end stop, otherwise the measured times cannot be trusted.
//

void HeatingActuatorChannel::startCalibration()
{
    logInfoP("Start calibration");

    _calibrationState = CalibrationState::CAL_INIT;
    _calibrationRunStarted = false;
    _calibratedDriveOpenTime = 0;
    _calibratedDriveCloseTime = 0;
    _currentPositionPercent = HTA_POSITION_INVALID;
}

void HeatingActuatorChannel::setCalibrationStep(CalibrationState calibrationState)
{
    _calibrationState = calibrationState;
    _calibrationRunStarted = false;
}

void HeatingActuatorChannel::abortCalibration(const char *reason)
{
    logErrorP("Calibration failed: %s", reason);

    _calibrationState = CalibrationState::CAL_ERROR;
    _calibrationRunStarted = false;
    _moveRequested = false;
}

void HeatingActuatorChannel::processCalibration()
{
    if (!_calibrationRunStarted)
    {
        // retried until the module grants the shared motor driver
        _calibrationRunStarted = requestMotor(_calibrationState == CalibrationState::CAL_OPENING);
        return;
    }

    if (_motorStopReason != MotorStopReason::EndStop)
    {
        abortCalibration("motor did not reach the mechanical end stop");
        return;
    }

    switch (_calibrationState)
    {
        case CalibrationState::CAL_INIT:
            // valve is closed now, the position is set by the end stop detection
            setCalibrationStep(CalibrationState::CAL_OPENING);
            break;
        case CalibrationState::CAL_OPENING:
            if (_motorRunTime < HTA_MOT_MIN_DRIVE_TIME)
            {
                abortCalibration("measured opening time implausible");
                break;
            }

            _calibratedDriveOpenTime = _motorRunTime;
            setCalibrationStep(CalibrationState::CAL_CLOSING);
            break;
        case CalibrationState::CAL_CLOSING:
            if (_motorRunTime < HTA_MOT_MIN_DRIVE_TIME)
            {
                abortCalibration("measured closing time implausible");
                break;
            }

            _calibratedDriveCloseTime = _motorRunTime;
            _calibrationState = CalibrationState::CAL_COMPLETE;
            _calibrationRunStarted = false;

            logInfoP("Calibration complete (open: %u ms, close: %u ms)",
                     _calibratedDriveOpenTime, _calibratedDriveCloseTime);
            break;
        default:
            break;
    }
}

//
// valve positioning
//

bool HeatingActuatorChannel::moveValveToPosition(float targetPositionPercent)
{
    if (targetPositionPercent < HTA_POSITION_FULLY_CLOSED)
        targetPositionPercent = HTA_POSITION_FULLY_CLOSED;
    else if (targetPositionPercent > HTA_POSITION_FULLY_OPEN)
        targetPositionPercent = HTA_POSITION_FULLY_OPEN;

    setTargetPosition(targetPositionPercent);

    // the movement is carried out by the channel loop as soon as the motor is available
    _moveRequested = true;
    _motorStopReason = MotorStopReason::Unknown;

    if (_calibrationState == CalibrationState::CAL_COMPLETE)
        return true;

    if (_calibrationState == CalibrationState::CAL_NONE)
    {
        logInfoP("Moving to position requires calibration first");
        startCalibration();
    }

    return false;
}

// runs the motor until the mechanical end stop is reached, without position control
bool HeatingActuatorChannel::driveToEndStop(bool open)
{
    _moveRequested = false;
    setTargetPosition(open ? HTA_POSITION_FULLY_OPEN : HTA_POSITION_FULLY_CLOSED);

    return requestMotor(open);
}

bool HeatingActuatorChannel::processMove()
{
    if (!_moveRequested)
        return false;

    if (_calibrationState != CalibrationState::CAL_COMPLETE)
    {
        // without a calibration the position cannot be determined
        if (_calibrationState == CalibrationState::CAL_ERROR)
            _moveRequested = false;

        return false;
    }

    const float remaining = _targetPositionPercent - _currentPositionPercent;
    if (fabsf(remaining) < HTA_POSITION_TOLERANCE)
    {
        _moveRequested = false;
        logDebugP("Target position reached (position: %.4f, target: %.4f)",
                  _currentPositionPercent, _targetPositionPercent);
        return false;
    }

    // retried until the module grants the shared motor driver
    requestMotor(remaining > 0);
    return true;
}

void HeatingActuatorChannel::setTargetPosition(float targetPositionPercent)
{
    _targetPositionPercent = targetPositionPercent;

    if (ParamHTA_ChSetValueChangeSend)
        sendSetValueStatus();
}

void HeatingActuatorChannel::sendSetValueStatus()
{
    const uint8_t setValuePercent = getSetValueTarget();

    if (ParamHTA_ChControlMode == HTA_CONTROL_MODE_EXTERN || _currentOperationModeHeating)
        KoHTA_ChSetValueStatusHeatingOrExtern.value(setValuePercent, DPT_Scaling);
    else
        KoHTA_ChSetValueStatusCooling.value(setValuePercent, DPT_Scaling);
}

uint8_t HeatingActuatorChannel::getSetValueTarget()
{
    if (_targetPositionPercent == HTA_POSITION_INVALID)
        return 0;

    return (uint8_t)roundf(_targetPositionPercent * 100);
}

bool HeatingActuatorChannel::considerForRequestAndMaxSetValue()
{
    return ParamHTA_ChConsiderForRequestAndMaxSetValue;
}

//
// set value calculation
//

void HeatingActuatorChannel::calculateNewSetValue()
{
    std::string debugLogMessage = "";

    // check if emergency mode should be active
    checkEmergencyMode();

    // first check for possible enforced position
    float setValuePercent = HTA_POSITION_INVALID;
    if (ParamHTA_ChEnforcedPosition &&
        _externEnforcedPosition)
    {
        if (ParamHTA_ChControlMode == HTA_CONTROL_MODE_EXTERN || _currentOperationModeHeating)
            setValuePercent = ParamHTA_ChEnforcedSetValueHeatingOrExtern / 100.0f;
        else
            setValuePercent = ParamHTA_ChEnforcedSetValueCooling / 100.0f;

#ifdef OPENKNX_DEBUG
        debugLogMessage = string_format("calculateNewSetValue: enforced position (setValuePercent: %.2f)", setValuePercent);
#endif
    }
    // check if manual mode is active
    else if (ParamHTA_ChManualMode && _currentManualMode)
    {
        if (_currentManualModeOn)
            setValuePercent = ParamHTA_ChManualModeSetValueOn / 100.0f;
        else
            setValuePercent = ParamHTA_ChManualModeSetValueOff / 100.0f;

#ifdef OPENKNX_DEBUG
        debugLogMessage = string_format("calculateNewSetValue: manual mode (_currentManualModeOn: %u, setValuePercent: %.2f)", _currentManualModeOn, setValuePercent);
#endif
    }
    // check if emergency mode is active
    else if (_currentEmergencyMode)
    {
        if (ParamHTA_ChControlMode == HTA_CONTROL_MODE_EXTERN || _currentOperationModeHeating)
            setValuePercent = ParamHTA_ChEmergencyModeSetValueHeatingOrExtern / 100.0f;
        else
            setValuePercent = ParamHTA_ChEmergencyModeSetValueCooling / 100.0f;

#ifdef OPENKNX_DEBUG
        debugLogMessage = string_format("calculateNewSetValue: emergency mode (setValuePercent: %.2f)", setValuePercent);
#endif
    }
    // check for external control
    else if (ParamHTA_ChControlMode == HTA_CONTROL_MODE_EXTERN)
    {
        setValuePercent = _externSetValuePercent;

#ifdef OPENKNX_DEBUG
        debugLogMessage = string_format("calculateNewSetValue: external control (setValuePercent: %.2f)", setValuePercent);
#endif
    }
    // standard internal regulator target temperature calculation
    else
    {
        float targetTemp = _externTargetTemp;
        if (targetTemp == HTA_TEMPERATUR_INVALID)
        {
            switch (_currentHvacMode)
            {
                case HvacMode::HVAC_COMFORT:
                    targetTemp = _currentOperationModeHeating ? ParamHTA_ChTargetTempHeatingComfort : ParamHTA_ChTargetTempCoolingComfort;
                    break;
                case HvacMode::HVAC_NIGHT:
                    targetTemp = _currentOperationModeHeating ? ParamHTA_ChTargetTempHeatingNight : ParamHTA_ChTargetTempCoolingNight;
                    break;
                case HvacMode::HVAC_PROTECT:
                    targetTemp = _currentOperationModeHeating ? ParamHTA_ChTargetTempHeatingProtect : ParamHTA_ChTargetTempCoolingProtect;
                    break;
                default:
                    targetTemp = _currentOperationModeHeating ? ParamHTA_ChTargetTempHeatingStandby : ParamHTA_ChTargetTempCoolingStandby;
                    break;
            }
        }

        targetTemp += _externTargetTempShift;

        if (_currentTargetTemp != targetTemp)
        {
            _currentTargetTemp = targetTemp;
            pid.setPoint(targetTemp);

            if (ParamHTA_ChTargetTempChangeSend)
                KoHTA_ChTargetTempStatus.value(_currentTargetTemp, DPT_Value_Temp);
        }

        float newPidPositionPercent = HTA_POSITION_INVALID;
        if (_externRoomTemp != HTA_TEMPERATUR_INVALID)
        {
            if (!pid.isRunning())
            {
                if (_currentOperationModeHeating)
                {
                    pid.setInterval(ParamHTA_ChHeatingPidInterval);
                    pid.setK(ParamHTA_ChHeatingPidP, ParamHTA_ChHeatingPidI / 10.0f, ParamHTA_ChHeatingPidD / 10.0f);
                }
                else
                {
                    pid.setInterval(ParamHTA_ChCoolingPidInterval);
                    pid.setK(ParamHTA_ChCoolingPidP, ParamHTA_ChCoolingPidI / 10.0f, ParamHTA_ChCoolingPidD / 10.0f);
                }

                pid.setOutputRange(0, 255);
                pid.start();

                logDebugP("calculateNewSetValue: regulator PID initialized (P: %.2f, I: %.2f, D: %.2f, interval: %u)", pid.getKp(), pid.getKi(), pid.getKd(), pid.getInterval());
            }

            if (pid.compute(_externRoomTemp))
                newPidPositionPercent = pid.getOutput() / 255.0f;
        }

        if (newPidPositionPercent != HTA_POSITION_INVALID)
        {
            setValuePercent = newPidPositionPercent;

#ifdef OPENKNX_DEBUG
            debugLogMessage = string_format("calculateNewSetValue: regulator (_currentHvacMode: %u, _externTargetTempShift: %.2f, targetTemp: %.2f, _externRoomTemp: %.2f, _targetPositionPercent: %.2f, newPidPositionPercent: %.2f)", _currentHvacMode, _externTargetTempShift, targetTemp, _externRoomTemp, _targetPositionPercent, newPidPositionPercent);
#endif
        }
    }

#ifdef OPENKNX_DEBUG
    if (debugLogMessage != "" &&
        _lastDebugLogMessage != debugLogMessage)
    {
        logDebugP("%s", debugLogMessage.c_str());
        _lastDebugLogMessage = debugLogMessage;
    }
#endif

    if (setValuePercent == HTA_POSITION_INVALID)
        return;

    // check if we need to move the valve
    if (_targetPositionPercent != HTA_POSITION_INVALID &&
        fabsf(_targetPositionPercent - setValuePercent) < HTA_POSITION_STEP)
        return;

    // starts the calibration first, if it was not done yet
    moveValveToPosition(setValuePercent);
}

//
// cyclic sending
//

void HeatingActuatorChannel::processCyclicSending()
{
    if (_targetPositionPercent != HTA_POSITION_INVALID &&
        ParamHTA_ChSetValueChangeSend && _setValueCyclicSendTimer > 0 &&
        delayCheck(_setValueCyclicSendTimer, ParamHTA_ChSetValueCyclicTimeMS))
    {
        sendSetValueStatus();
        _setValueCyclicSendTimer = delayTimerInit();
    }

    if (_currentTargetTemp != HTA_TEMPERATUR_INVALID &&
        ParamHTA_ChTargetTempChangeSend && _targetTempCyclicSendTimer > 0 &&
        delayCheck(_targetTempCyclicSendTimer, ParamHTA_ChTargetTempCyclicTimeMS))
    {
        KoHTA_ChTargetTempStatus.value(_currentTargetTemp, DPT_Value_Temp);
        _targetTempCyclicSendTimer = delayTimerInit();
    }

    if (ParamHTA_ChEmergencyModeChangeSend &&
        _emergencyModeCyclicSendTimer > 0 &&
        delayCheck(_emergencyModeCyclicSendTimer, ParamHTA_ChEmergencyModeCyclicTimeMS))
    {
        KoHTA_ChEmergencyModeStatus.value(_currentEmergencyMode, DPT_Switch);
        _emergencyModeCyclicSendTimer = delayTimerInit();
    }

    if (ParamHTA_ChManualModeChangeSend &&
        ((bool)KoHTA_ChManualModeStatus.value(DPT_Switch) != _currentManualMode ||
         (_manualModeCyclicSendTimer > 0 &&
          delayCheck(_manualModeCyclicSendTimer, ParamHTA_ChManualModeCyclicTimeMS))))
    {
        KoHTA_ChManualModeStatus.value(_currentManualMode, DPT_Switch);
        _manualModeCyclicSendTimer = delayTimerInit();
    }
}

//
// local button and LED
//

void HeatingActuatorChannel::processInput()
{
    if (!ParamHTA_ChManualMode)
        return;

#ifdef OPENKNX_HTA_GPIO_INPUT_OFFSET
    const bool buttonPressed = openknx.gpio.digitalRead(OPENKNX_HTA_GPIO_INPUT_OFFSET + _channelIndex) == GPIO_INPUT_ON;
    if (buttonPressed != _currentButtonPressed &&
        delayCheck(_currentButtonChanged, HTA_INPUT_DEBOUNCE))
    {
        _currentButtonPressed = buttonPressed;
        _currentButtonChanged = delayTimerInit();

        if (buttonPressed)
        {
            _currentButtonPressedStarted = delayTimerInit();

            // the first press enables manual mode, every further press toggles the set value
            setManualMode(true, _currentManualMode ? !_currentManualModeOn : true);
            logDebugP("processInput: manual mode button pressed (_currentManualModeOn: %u)", _currentManualModeOn);
        }
    }

    // keeping the button pressed switches back to automatic mode
    if (_currentButtonPressed && _currentManualMode &&
        delayCheck(_currentButtonPressedStarted, HTA_MANUAL_MODE_CHANGE_TO_AUTO_TIME_DELAY) &&
        (ParamHTA_ChManualModeChangeToAuto == HTA_MANUAL_MODE_CHANGE_TO_AUTO_BUTTON ||
         ParamHTA_ChManualModeChangeToAuto == HTA_MANUAL_MODE_CHANGE_TO_AUTO_BUTTON_TIME))
    {
        setManualMode(false, _currentManualModeOn);
        logDebugP("processInput: manual mode button off");
    }
#endif

    // manual mode automatically ends after the configured time
    if (_currentManualMode &&
        ParamHTA_ChManualModeChangeToAutoTimeMS > 0 &&
        delayCheck(_currentManualModeStarted, ParamHTA_ChManualModeChangeToAutoTimeMS) &&
        (ParamHTA_ChManualModeChangeToAuto == HTA_MANUAL_MODE_CHANGE_TO_AUTO_TIME ||
         ParamHTA_ChManualModeChangeToAuto == HTA_MANUAL_MODE_CHANGE_TO_AUTO_BUTTON_TIME))
    {
        setManualMode(false, _currentManualModeOn);
        logDebugP("processInput: manual mode time off");
    }
}

void HeatingActuatorChannel::processOutput()
{
#ifdef OPENKNX_HTA_GPIO_OUTPUT_OFFSET
    float ledOnPercent = 0;
    uint32_t ledOnTime = 0;

    if (_currentManualMode)
    {
        // permanently on in manual mode, off if the manual set value is off
        if (_currentManualModeOn)
        {
            ledOnPercent = 1;
            ledOnTime = HTA_OUTPUT_LED_PHASE;
        }
    }
    else if (_targetPositionPercent != HTA_POSITION_INVALID)
    {
        ledOnPercent = _targetPositionPercent;

        // minimum of 1 % and maximum of 99 % LED on to signal automatic mode
        if (ledOnPercent < 0.01)
            ledOnPercent = 0.01;
        else if (ledOnPercent > 0.99)
            ledOnPercent = 0.99;

        ledOnTime = round(HTA_OUTPUT_LED_PHASE * ledOnPercent);
    }

    if (ledOnTime == HTA_OUTPUT_LED_PHASE)
    {
        if (!_currentLedOn)
            setOutputLed(true);
    }
    else if (ledOnTime == 0)
    {
        if (_currentLedOn)
            setOutputLed(false);
    }
    else if (_currentLedOn && delayCheck(_currentLedChangeStarted, ledOnTime))
        setOutputLed(false);
    else if (!_currentLedOn && delayCheck(_currentLedChangeStarted, HTA_OUTPUT_LED_PHASE - ledOnTime))
        setOutputLed(true);

    if (_currentLedOnTime != ledOnTime)
    {
        _currentLedOnTime = ledOnTime;
        logDebugP("processOutput (ledOnPercent: %.2f, ledOnTime: %u)", ledOnPercent, ledOnTime);
    }
#endif
}

void HeatingActuatorChannel::setOutputLed(bool on)
{
#ifdef OPENKNX_HTA_GPIO_OUTPUT_OFFSET
    openknx.gpio.digitalWrite(OPENKNX_HTA_GPIO_OUTPUT_OFFSET + _channelIndex, on ? GPIO_OUTPUT_ON : GPIO_OUTPUT_OFF);
    _currentLedChangeStarted = delayTimerInit();
    _currentLedOn = on;
#endif
}

//
// power fail and persistence
//

void HeatingActuatorChannel::savePower()
{
    if (_motorState != MotorState::MOT_IDLE)
        openknxHeatingActuatorModule.stopMotor(MotorStopReason::Unknown);

    // a calibration in progress cannot be continued after a restart
    switch (_calibrationState)
    {
        case CalibrationState::CAL_INIT:
        case CalibrationState::CAL_OPENING:
        case CalibrationState::CAL_CLOSING:
            setCalibrationStep(CalibrationState::CAL_NONE);
            break;
        default:
            break;
    }
}

bool HeatingActuatorChannel::restorePower()
{
    return true;
}

void HeatingActuatorChannel::writeChannelData()
{
    openknx.flash.writeByte(_calibrationState);
    openknx.flash.writeInt(_calibratedDriveOpenTime);
    openknx.flash.writeInt(_calibratedDriveCloseTime);
    openknx.flash.writeFloat(_currentPositionPercent);
}

void HeatingActuatorChannel::readChannelData()
{
    // all bytes have to be read, even if the content turns out to be unusable
    const CalibrationState calibrationState = static_cast<CalibrationState>(openknx.flash.readByte());
    const uint32_t calibratedDriveOpenTime = openknx.flash.readInt();
    const uint32_t calibratedDriveCloseTime = openknx.flash.readInt();
    const float currentPositionPercent = openknx.flash.readFloat();

    // only a completed and plausible calibration can be restored, anything else
    // would make the position calculation run wild
    if (calibrationState != CalibrationState::CAL_COMPLETE ||
        calibratedDriveOpenTime < HTA_MOT_MIN_DRIVE_TIME || calibratedDriveOpenTime > HTA_MOT_MAX_DRIVE_TIME ||
        calibratedDriveCloseTime < HTA_MOT_MIN_DRIVE_TIME || calibratedDriveCloseTime > HTA_MOT_MAX_DRIVE_TIME ||
        currentPositionPercent < HTA_POSITION_FULLY_CLOSED || currentPositionPercent > HTA_POSITION_FULLY_OPEN)
    {
        logDebugP("Channel %u: no usable calibration stored, calibration required", _channelIndex);
        return;
    }

    _calibrationState = calibrationState;
    _calibratedDriveOpenTime = calibratedDriveOpenTime;
    _calibratedDriveCloseTime = calibratedDriveCloseTime;
    _currentPositionPercent = currentPositionPercent;
}

void HeatingActuatorChannel::logChannelInfo(bool diagnoseKo)
{
    logInfoP("Channel info:");
    logIndentUp();

    logInfoP("_currentPositionPercent: %.2f", _currentPositionPercent);
    logInfoP("_targetPositionPercent: %.2f", _targetPositionPercent);
    logInfoP("_moveRequested: %u", _moveRequested);
    logInfoP("_motorState: %u (stop reason: %u, run time: %u ms)", _motorState, (uint8_t)_motorStopReason, _motorRunTime);
    logInfoP("_calibrationState: %u", _calibrationState);
    logInfoP("_calibratedDriveOpenTime: %u", _calibratedDriveOpenTime);
    logInfoP("_calibratedDriveCloseTime: %u", _calibratedDriveCloseTime);
    logInfoP("_externEnforcedPosition: %u", _externEnforcedPosition);
    logInfoP("_currentEmergencyMode: %u", _currentEmergencyMode);
    logInfoP("_currentManualMode: %u (On=%u)", _currentManualMode, _currentManualModeOn);
    logInfoP("_currentOperationModeHeating: %u", _currentOperationModeHeating);
    logInfoP("_currentHvacMode: %u", _currentHvacMode);

    if (diagnoseKo)
    {
        openknx.console.writeDiagnoseKo("HTA cur %.2f", _currentPositionPercent);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA tar %.2f", _targetPositionPercent);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA mot %u %u", _motorState, (uint8_t)_motorStopReason);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA cal %u", _calibrationState);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA calo %u", _calibratedDriveOpenTime);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA calc %u", _calibratedDriveCloseTime);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA enf %u", _externEnforcedPosition);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA man %u %u", _currentManualMode, _currentManualModeOn);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA hvac %u", _currentHvacMode);
        openknx.console.writeDiagnoseKo("");
    }

    logIndentDown();
}
