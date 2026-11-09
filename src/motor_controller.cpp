
#include "motor_controller.hpp"
#include "pins.hpp"
#include <cmath>

// This file is the hardware interface between the robot's motion planner and the
// physical stepper motors. It maps each wheel command to a direction, speed, and
// enable state for the motor driver.

// GPIO assignments from pins.hpp. Each motor uses its own step, direction, and
// enable pin so the controller can command the three wheels independently.
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
    // Initialize the FastAccelStepper engine and attach each of the three
    // motors to its configured step pin. After this step, each wheel is ready to
    // receive direction, speed, and enable commands.
    engine.init();

    for (int motorIndex = 0; motorIndex < 3; motorIndex++) {
        motors[motorIndex] = engine.stepperConnectToPin(STEP_PINS[motorIndex]);

        if (motors[motorIndex] == nullptr) {
            continue;
        }

        motors[motorIndex]->setDirectionPin(DIR_PINS[motorIndex]);
        motors[motorIndex]->setEnablePin(ENABLE_PINS[motorIndex]);
        motors[motorIndex]->setAutoEnable(true);

        motors[motorIndex]->setSpeedInHz(MAX_SPEED_HZ);
        motors[motorIndex]->setAcceleration(ACCELERATION);
    }
}

void MotorController::setMotorPercentages(
    float motor0Percent,
    float motor1Percent,
    float motor2Percent
) {
    // This is the main control point for the actual motors.
    // Each wheel command is converted from a percentage (-100..100) into a
    // stepper speed and direction. Positive values move forward, negative values
    // reverse, and zero stops that motor.
    float motorPercentages[3] = {motor0Percent, motor1Percent, motor2Percent};

    for (int motorIndex = 0; motorIndex < 3; motorIndex++) {
        if (motors[motorIndex] == nullptr) {
            continue;
        }

        float motorPercentage = motorPercentages[motorIndex];

        if (motorPercentage == 0.0f) {
            motors[motorIndex]->stopMove();
            continue;
        }

        uint32_t speed = static_cast<uint32_t>(
            fabsf(motorPercentage) / 100.0f * MAX_SPEED_HZ
        );

        if (speed == 0) {
            speed = 1;
        }

        motors[motorIndex]->setSpeedInHz(speed);

        if (motorPercentage > 0) {
            motors[motorIndex]->runForward();
        } else {
            motors[motorIndex]->runBackward();
        }
    }
}

void MotorController::stopAll() {
    for (int motorIndex = 0; motorIndex < 3; motorIndex++) {
        if (motors[motorIndex] != nullptr) {
            motors[motorIndex]->stopMove();
        }
    }
}
