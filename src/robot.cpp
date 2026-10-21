#include "robot.hpp"
#include <cassert>

void Robot::enableRobotMotors(bool m0State, bool m1State, bool m2State) {
    motorEnabled[0] = m0State;
    motorEnabled[1] = m1State;
    motorEnabled[2] = m2State;
}

void Robot::setRobotMotorPercentages(float m0Percentage, float m1Percentage, float m2Percentage) {
    // TODO: Normalize motor percentages while preserving ratios
    motorSpeeds[0] = m0Percentage;
    motorSpeeds[1] = m1Percentage;
    motorSpeeds[2] = m2Percentage;
}

float Robot::getRobotMotorPercentage(int motor) const {
    assert(motor >= 0 && motor < 3);
    return motorSpeeds[motor];
}