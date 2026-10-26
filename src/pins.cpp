#include <Arduino.h>
#include <Wire.h>
#include "pins.hpp"

namespace Pins {

void initializePins() {
    // TMC2209 enable is active-low: HIGH keeps motors disabled.
    digitalWrite(MOTOR0_EN, HIGH);
    digitalWrite(MOTOR1_EN, HIGH);
    digitalWrite(MOTOR2_EN, HIGH);

    pinMode(MOTOR0_EN, OUTPUT);
    pinMode(MOTOR0_STEP, OUTPUT);
    pinMode(MOTOR0_DIR, OUTPUT);

    pinMode(MOTOR1_EN, OUTPUT);
    pinMode(MOTOR1_STEP, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);

    pinMode(MOTOR2_EN, OUTPUT);
    pinMode(MOTOR2_STEP, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);

    digitalWrite(MOTOR0_STEP, LOW);
    digitalWrite(MOTOR1_STEP, LOW);
    digitalWrite(MOTOR2_STEP, LOW);

    // Wire.begin(IMU_SDA, IMU_SCL);
}

}
