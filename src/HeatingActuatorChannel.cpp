#include "HeatingActuatorChannel.h"
#include "HeatingActuatorModule.h"
#include <cmath>

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

    switch (HTA_KoCalcIndex(ko.asap()))
    {
        case HTA_KoChEnforcedPosition:
            _externEnforcedPosition = ko.value(DPT_Switch);
            logDebugP("HTA_KoChEnforcedPosition: %u", _externEnforcedPosition);
            break;
        case HTA_KoChManualMode:
            logDebugP("HTA_KoChManualMode: %u", (bool)ko.value(DPT_Switch));
            setManualMode(ko.value(DPT_Switch), _currentManualModeOn);
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
// emergency and manual mode
//

void HeatingActuatorChannel::checkEmergencyMode()
{
    // no set value from the climate control module for too long
    const bool newEmergencyMode =
        ParamHTA_ChEmergencyMode &&
        ParamHTA_ChEmergencyModeDelayTimeMS > 0 &&
        delayCheck(_lastExternValue, ParamHTA_ChEmergencyModeDelayTimeMS);

    if (_currentEmergencyMode == newEmergencyMode)
        return;

    _currentEmergencyMode = newEmergencyMode;
    logDebugP("checkEmergencyMode (_currentEmergencyMode=%u)", _currentEmergencyMode);

    KoHTA_ChEmergencyModeStatus.value(_currentEmergencyMode, DPT_Switch);
}

void HeatingActuatorChannel::setManualMode(bool manualMode, bool manualModeOn)
{
    // restart the automatic switch back timer whenever manual mode is entered
    if (manualMode && !_currentManualMode)
        _currentManualModeStarted = delayTimerInit();

    _currentManualMode = manualMode;
    _currentManualModeOn = manualModeOn;
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

// set value provided by the climate control module; the movement itself is decided in
// calculateNewSetValue(), where the local overrides have priority over this value
void HeatingActuatorChannel::moveValveToPosition(float targetPositionPercent)
{
    if (targetPositionPercent < HTA_POSITION_FULLY_CLOSED)
        targetPositionPercent = HTA_POSITION_FULLY_CLOSED;
    else if (targetPositionPercent > HTA_POSITION_FULLY_OPEN)
        targetPositionPercent = HTA_POSITION_FULLY_OPEN;

    _externSetValuePercent = targetPositionPercent;
    _lastExternValue = delayTimerInit();
}

// runs the motor until the mechanical end stop is reached, without position control
bool HeatingActuatorChannel::driveToEndStop(bool open)
{
    _moveRequested = false;
    _targetPositionPercent = open ? HTA_POSITION_FULLY_OPEN : HTA_POSITION_FULLY_CLOSED;

    return requestMotor(open);
}

void HeatingActuatorChannel::requestValvePosition(float targetPositionPercent)
{
    _targetPositionPercent = targetPositionPercent;
    _moveRequested = true;
    _motorStopReason = MotorStopReason::Unknown;

    if (_calibrationState == CalibrationState::CAL_NONE)
    {
        logInfoP("Moving to position requires calibration first");
        startCalibration();
    }
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

uint8_t HeatingActuatorChannel::getSetValueTarget()
{
    if (_targetPositionPercent == HTA_POSITION_INVALID)
        return 0;

    return (uint8_t)roundf(_targetPositionPercent * 100);
}

//
// set value calculation
//
// The regular set value comes from the climate control module, this only adds the
// local overrides in their order of priority.
//

void HeatingActuatorChannel::calculateNewSetValue()
{
    std::string debugLogMessage = "";

    // check if emergency mode should be active
    checkEmergencyMode();

    float setValuePercent = HTA_POSITION_INVALID;

    // first check for possible enforced position
    if (ParamHTA_ChEnforcedPosition &&
        _externEnforcedPosition)
    {
        setValuePercent = ParamHTA_ChEnforcedSetValue / 100.0f;

#ifdef OPENKNX_DEBUG
        debugLogMessage = string_format("calculateNewSetValue: enforced position (setValuePercent: %.2f)", setValuePercent);
#endif
    }
    // check if manual mode is active
    else if (ParamHTA_ChManualMode && _currentManualMode)
    {
        setValuePercent = (_currentManualModeOn ? ParamHTA_ChManualModeSetValueOn : ParamHTA_ChManualModeSetValueOff) / 100.0f;

#ifdef OPENKNX_DEBUG
        debugLogMessage = string_format("calculateNewSetValue: manual mode (_currentManualModeOn: %u, setValuePercent: %.2f)", _currentManualModeOn, setValuePercent);
#endif
    }
    // check if emergency mode is active
    else if (_currentEmergencyMode)
    {
        setValuePercent = ParamHTA_ChEmergencyModeSetValue / 100.0f;

#ifdef OPENKNX_DEBUG
        debugLogMessage = string_format("calculateNewSetValue: emergency mode (setValuePercent: %.2f)", setValuePercent);
#endif
    }
    // regular operation: follow the climate control module
    else
    {
        setValuePercent = _externSetValuePercent;

#ifdef OPENKNX_DEBUG
        if (setValuePercent != HTA_POSITION_INVALID)
            debugLogMessage = string_format("calculateNewSetValue: climate control (setValuePercent: %.2f)", setValuePercent);
#endif
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
    requestValvePosition(setValuePercent);
}

//
// cyclic sending
//

void HeatingActuatorChannel::processCyclicSending()
{
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
    logInfoP("_externSetValuePercent: %.2f", _externSetValuePercent);
    logInfoP("_moveRequested: %u", _moveRequested);
    logInfoP("_motorState: %u (stop reason: %u, run time: %u ms)", _motorState, (uint8_t)_motorStopReason, _motorRunTime);
    logInfoP("_calibrationState: %u", _calibrationState);
    logInfoP("_calibratedDriveOpenTime: %u", _calibratedDriveOpenTime);
    logInfoP("_calibratedDriveCloseTime: %u", _calibratedDriveCloseTime);
    logInfoP("_externEnforcedPosition: %u", _externEnforcedPosition);
    logInfoP("_currentEmergencyMode: %u", _currentEmergencyMode);
    logInfoP("_currentManualMode: %u (On=%u)", _currentManualMode, _currentManualModeOn);

    if (diagnoseKo)
    {
        openknx.console.writeDiagnoseKo("HTA cur %.2f", _currentPositionPercent);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA tar %.2f", _targetPositionPercent);
        openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HTA ext %.2f", _externSetValuePercent);
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
    }

    logIndentDown();
}
