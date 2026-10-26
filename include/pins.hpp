#pragma once

namespace Pins {

    void initializePins();
    
    // Motor 0
    constexpr int MOTOR0_EN = 32;
    constexpr int MOTOR0_STEP = 33;
    constexpr int MOTOR0_DIR = 25;

    // Motor 1
    constexpr int MOTOR1_EN = 26;
    constexpr int MOTOR1_STEP = 17;
    constexpr int MOTOR1_DIR = 14;

    // Motor 2
    constexpr int MOTOR2_EN = 23;
    constexpr int MOTOR2_STEP = 19;
    constexpr int MOTOR2_DIR = 18;

    // IMU
    constexpr int IMU_SCL = 22;
    constexpr int IMU_SDA = 21;
}