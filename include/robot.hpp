#pragma once

class Robot {
private:
    float speed = 0.0f;
    float motorSpeeds[3] = {0.0f, 0.0f, 0.0f};
    bool motorEnabled[3] = {false, false, false};

public:
    void enableRobotMotors(bool motor0Enabled, bool motor1Enabled, bool motor2Enabled);

    void setRobotMotorPercentages(
        float motor0Percentage,
        float motor1Percentage,
        float motor2Percentage
    );

    float getRobotMotorPercentage(int motorIndex) const;
};