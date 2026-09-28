#ifndef LINE_FOLLOWER_CONTROLLER_H
#define LINE_FOLLOWER_CONTROLLER_H

#include <stdint.h>

struct SensorInputs {
    bool leftDetected;
    bool rightDetected;
};

struct MotorOutputs {
    uint8_t leftPwm;
    uint8_t rightPwm;
    bool leftForward;
    bool rightForward;
};

struct ControllerOutputs {
    MotorOutputs motors;
    bool indicatorActive;
    bool isStopped;
};

enum class RobotState {
    STANDBY,
    RACING,
    RESCUING,
    EMERGENCY_STOP
};

class LineFollowerController {
public:
    static const uint8_t BASE_SPEED = 160;
    static const uint8_t TRIM_RIGHT = 20;

    LineFollowerController()
        : state_(RobotState::STANDBY),
          wasButtonPressed_(false) {
        outputs_.motors.leftPwm = 0;
        outputs_.motors.rightPwm = 0;
        outputs_.motors.leftForward = true;
        outputs_.motors.rightForward = true;
        outputs_.indicatorActive = false;
        outputs_.isStopped = true;
    }

    void update(bool buttonPressed, const SensorInputs& sensors, uint32_t currentTimeMs) {
        (void)sensors;
        (void)currentTimeMs;

        if (state_ == RobotState::STANDBY) {
            if (buttonPressed) {
                wasButtonPressed_ = true;
            } else if (wasButtonPressed_) {
                // Button was held and now released -> launch race routine
                state_ = RobotState::RACING;
                outputs_.indicatorActive = true;
                outputs_.isStopped = false;
                outputs_.motors.leftPwm = BASE_SPEED;
                outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
                outputs_.motors.leftForward = true;
                outputs_.motors.rightForward = true;
            }
        }
    }

    ControllerOutputs getOutputs() const {
        return outputs_;
    }

    RobotState getState() const {
        return state_;
    }

private:
    RobotState state_;
    bool wasButtonPressed_;
    ControllerOutputs outputs_;
};

#endif // LINE_FOLLOWER_CONTROLLER_H
