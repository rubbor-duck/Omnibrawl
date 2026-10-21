#pragma once

class Robot {
private:
    float speed = 0.0f;
    float motorSpeeds[3] = {0.0f, 0.0f, 0.0f};
    bool motorEnabled[3] = {false, false, false};

public:
    void enableRobotMotors(bool m0State, bool m1State, bool m2State);

    void setRobotMotorPercentages(
        float m0Percentage,
        float m1Percentage,
        float m2Percentage
    );

    float getRobotMotorPercentage(int motor) const;
};