#include "robot.hpp"
#include <cassert>

// The Robot class stores the latest desired wheel percentages for the robot.
// This keeps the higher-level movement code separate from the low-level motor
// driver, so the control logic can be tested and adjusted without rewriting the
// hardware interface.

void Robot::enableRobotMotors(bool motor0Enabled, bool motor1Enabled, bool motor2Enabled) {
    motorEnabled[0] = motor0Enabled;
    motorEnabled[1] = motor1Enabled;
    motorEnabled[2] = motor2Enabled;
}

void Robot::setRobotMotorPercentages(float motor0Percentage, float motor1Percentage, float motor2Percentage) {
    // Store the latest motion command for all three wheels.
    // This is a lightweight middle layer between the kinematics code and the
    // actual motor controller, so the rest of the robot logic can read the same
    // values consistently.
    motorSpeeds[0] = motor0Percentage;
    motorSpeeds[1] = motor1Percentage;
    motorSpeeds[2] = motor2Percentage;
}

float Robot::getRobotMotorPercentage(int motorIndex) const {
    assert(motorIndex >= 0 && motorIndex < 3);
    return motorSpeeds[motorIndex];
}