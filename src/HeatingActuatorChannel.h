#pragma once
#include "OpenKNX.h"
#include "FormatHelper.h"

// valve position: 0.0 = fully closed ... 1.0 = fully open
#define HTA_POSITION_INVALID -1
#define HTA_POSITION_FULLY_CLOSED 0
#define HTA_POSITION_FULLY_OPEN 1

// smallest set value change which triggers a new valve movement (1 %)
#define HTA_POSITION_STEP 0.01f
// deviation below which the target position counts as reached (0.5 %)
#define HTA_POSITION_TOLERANCE 0.005f

#define HTA_MOT_RESTART_DELAY 1000 // avoid power spikes, especially when changing motor direction

// number of current measurements to skip after motor start:
// the inrush current must not be mistaken for a mechanical end stop or an overload
#define HTA_MOT_CURRENT_SETTLE_COUNT 500
// current rise (mA) which, together with the configured maximum, indicates a mechanical end stop
#define HTA_MOT_CURRENT_RISE 0.1f

// plausibility limits for the time of a complete valve travel
#define HTA_MOT_MIN_DRIVE_TIME 1000
#define HTA_MOT_MAX_DRIVE_TIME 120000
// time a calibrated movement may exceed its calibrated drive time before it is aborted
#define HTA_MOT_RUNTIME_MARGIN 5000

// the stuck protection interval is configured in days
#define HTA_STUCK_PROTECTION_DAY_MS 86400000UL

#define HTA_OUTPUT_LED_PHASE 3000
#define HTA_INPUT_DEBOUNCE 50

#define HTA_MANUAL_MODE_CHANGE_TO_AUTO_DISABLED 0
#define HTA_MANUAL_MODE_CHANGE_TO_AUTO_TIME 1
#define HTA_MANUAL_MODE_CHANGE_TO_AUTO_BUTTON 2
#define HTA_MANUAL_MODE_CHANGE_TO_AUTO_BUTTON_TIME 3
#define HTA_MANUAL_MODE_CHANGE_TO_AUTO_TIME_DELAY 3000

// why a motor run ended; the channel needs this to know whether the result of the
// run can be trusted (calibration) and whether a pending movement has to be continued
enum class MotorStopReason : uint8_t
{
    Unknown,       // stopped from outside: console command, power fail, unspecified
    TargetReached, // the requested target position has been reached
    EndStop,       // mechanical end stop detected via the motor current
    NoMotor,       // no motor current measured, nothing connected
    Overcurrent,   // hardware current limit exceeded, motor blocked
    Timeout        // maximum motor run time exceeded
};

/*
 * Drives the valve of one channel to a requested position.
 *
 * The set value itself is not calculated here: it is provided by the climate control
 * module via moveValveToPosition(). This channel only adds the local overrides
 * (enforced position, manual mode, emergency mode) and the motor/position handling.
 */
class HeatingActuatorChannel : public OpenKNX::Channel
{
  public:
    HeatingActuatorChannel(uint8_t channelNumber);
    ~HeatingActuatorChannel();

    void processInputKo(GroupObject &ko);
    void setup(bool configured);
    void loop(bool motorPower, uint32_t currentCount, float current, float currentLast);

    // the following three are called by HeatingActuatorModule only, as it owns the shared
    // H-bridge; inside the channel use requestMotor()/openknxHeatingActuatorModule.stopMotor()
    void runMotor(bool open);
    void stopMotor(MotorStopReason reason);
    void motorOutputOff();

    uint8_t getSetValueTarget();

    void startCalibration();
    void moveValveToPosition(float targetPositionPercent);
    bool driveToEndStop(bool open);

    void savePower();
    bool restorePower();
    void writeChannelData();
    void readChannelData();

    void logChannelInfo(bool diagnoseKo);

  protected:

  private:
    enum MotorState : uint8_t
    {
        MOT_IDLE,
        MOT_OPENING,
        MOT_CLOSING
    };

    // the numeric values are persisted in flash, do not change them
    enum CalibrationState : uint8_t
    {
        CAL_NONE,
        CAL_INIT,
        CAL_OPENING,
        CAL_CLOSING,
        CAL_COMPLETE,
        CAL_ERROR
    };

    const std::string name() override;

    void checkEmergencyMode();
    void setManualMode(bool manualMode, bool manualModeOn);

    void calculateNewSetValue();
    void processCyclicSending();

    void processRunningMotor(uint32_t currentCount, float current, float currentLast);
    void processIdle();
    bool isStuckProtectionDue();
    void startStuckProtection();
    void processCalibration();
    void setCalibrationStep(CalibrationState calibrationState);
    void abortCalibration(const char *reason);
    bool processMove();
    bool requestMotor(bool open);
    uint32_t maxMotorRunTime();
    uint32_t calibratedDriveTime(MotorState motorState);
    void applyMotorTravel(MotorState motorState, uint32_t runTime);
    void requestValvePosition(float targetPositionPercent);

    void processInput();
    void processOutput();
    void setOutputLed(bool on);

    uint32_t _emergencyModeCyclicSendTimer = 0;
    uint32_t _manualModeCyclicSendTimer = 0;

    MotorState _motorState = MotorState::MOT_IDLE;
    MotorStopReason _motorStopReason = MotorStopReason::Unknown;
    uint32_t _motorStarted = 0;
    uint32_t _motorStopped = 0;
    uint32_t _motorRunTime = 0;

    CalibrationState _calibrationState = CalibrationState::CAL_NONE;
    bool _calibrationRunStarted = false;
    uint32_t _calibratedDriveOpenTime = 0;
    uint32_t _calibratedDriveCloseTime = 0;

    // start of the last complete valve travel; survives a restart via the flash data
    uint32_t _stuckProtectionTimer = 0;

    float _currentPositionPercent = HTA_POSITION_INVALID;
    float _targetPositionPercent = HTA_POSITION_INVALID;
    bool _moveRequested = false;

    bool _externEnforcedPosition = false;

    // last set value provided by the climate control module
    float _externSetValuePercent = HTA_POSITION_INVALID;
    uint32_t _lastExternValue = 0;

    bool _currentEmergencyMode = false;

    bool _currentManualMode = false;
    bool _currentManualModeOn = false;
    uint32_t _currentManualModeStarted = 0;

    bool _currentButtonPressed = false;
    uint32_t _currentButtonChanged = 0;
    uint32_t _currentButtonPressedStarted = 0;

    bool _currentLedOn = false;
    uint32_t _currentLedOnTime = 0;
    uint32_t _currentLedChangeStarted = 0;

    std::string _lastDebugLogMessage = "";
};
