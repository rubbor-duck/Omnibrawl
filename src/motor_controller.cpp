#include "motor_controller.hpp"
#include <cmath>

// Replace with your actual GPIO assignments
static constexpr uint8_t STEP_PINS[3] = {25, 26, 27};
static constexpr uint8_t DIR_PINS[3] = {14, 16, 17};
static constexpr uint8_t ENABLE_PINS[3] = {18, 19, 21};

void MotorController::begin() {
    engine.init();

    for (int i = 0; i < 3; i++) {
        motors[i] = engine.stepperConnectToPin(STEP_PINS[i]);

        if (motors[i] == nullptr) {
            continue;
        }

        motors[i]->setDirectionPin(DIR_PINS[i]);
        motors[i]->setEnablePin(ENABLE_PINS[i]);
        motors[i]->setAutoEnable(true);

        motors[i]->setSpeedInHz(MAX_SPEED_HZ);
        motors[i]->setAcceleration(ACCELERATION);
    }
}

void MotorController::setMotorPercentages(
    float m0, float m1, float m2
) {
    float percentages[3] = {m0, m1, m2};

    for (int i = 0; i < 3; i++) {
        if (motors[i] == nullptr) {
            continue;
        }

        float percentage = percentages[i];

        // Constrain to -100% through +100%
        percentage = fmaxf(-100.0f, fminf(100.0f, percentage));

        if (percentage == 0.0f) {
            motors[i]->stopMove();
            continue;
        }

        uint32_t speed = static_cast<uint32_t>(
            fabsf(percentage) / 100.0f * MAX_SPEED_HZ
        );

        // Avoid requesting zero steps/sec
        if (speed == 0) speed = 1;

        motors[i]->setSpeedInHz(speed);

        if (percentage > 0) {
            motors[i]->runForward();
        } else {
            motors[i]->runBackward();
        }
    }
}

void MotorController::stopAll() {
    for (int i = 0; i < 3; i++) {
        if (motors[i] != nullptr) {
            motors[i]->stopMove();
        }
    }
}