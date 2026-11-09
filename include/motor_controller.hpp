#pragma once
#include <FastAccelStepper.h>

class MotorController {
private:
    FastAccelStepperEngine engine;
    FastAccelStepper* motors[3] = {nullptr, nullptr, nullptr};

    static constexpr uint32_t MAX_SPEED_HZ = 1600;
    static constexpr uint32_t ACCELERATION = 3200;

public:
    void begin();

    void setMotorPercentages(float motor0Percent, float motor1Percent, float motor2Percent);
    void stopAll();
};