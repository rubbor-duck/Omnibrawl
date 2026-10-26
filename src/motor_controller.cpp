
#include "motor_controller.hpp"
#include "pins.hpp"
#include <cmath>

// GPIO assignments from pins.hpp
static constexpr int STEP_PINS[3] = {
    Pins::MOTOR0_STEP,
    Pins::MOTOR1_STEP,
    Pins::MOTOR2_STEP
};

static constexpr int DIR_PINS[3] = {
    Pins::MOTOR0_DIR,
    Pins::MOTOR1_DIR,
    Pins::MOTOR2_DIR
};

static constexpr int ENABLE_PINS[3] = {
    Pins::MOTOR0_EN,
    Pins::MOTOR1_EN,
    Pins::MOTOR2_EN
};

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

        if (percentage == 0.0f) {
            motors[i]->stopMove();
            continue;
        }

        uint32_t speed = static_cast<uint32_t>(
            fabsf(percentage) / 100.0f * MAX_SPEED_HZ
        );

        if (speed == 0) {
            speed = 1;
        }

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
