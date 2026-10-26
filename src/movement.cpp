#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "main.hpp"
#include "web_server.hpp"
#include "robot.hpp"

namespace {
    BLA::Matrix<3, 2> wheelMatrix;
    BLA::Matrix<2> inputVector;
}

void initializeTransformations() {
    // Same 3-wheel transformation matrix as the original.
    wheelMatrix = {1, 0, -0.5, 0.866, -0.5, -0.866};
    inputVector.Fill(0);
}

//TO DO: ADD IMU DATA FOR HEADER
std::array<float, 3> calcMotorPercentages(int x, int y, int phoneHeading, int imuHeading) {

    //PHASE 1: Switch to polar coordinates relative to the phone heading

    imuHeading = imuHeading; //DO NOTHING WITH IMUHEADER

    // 1.1 Calculate the radius (r) using std::hypot (avoids overflow/underflow)
    double r = std::hypot(x, y);

    // 1.2 Calculate the angle (theta) in radians using std::atan2
    double theta_deg = std::atan2(y, x) * (180.0 / M_PI);

    //1.3 Adjust the angle relative to the phone heading
    theta_deg = theta_deg + phoneHeading;

    //PHASE 2: Convert back to Cartesian coordinates relative to the phone heading
    double rotatedX = r * std::cos(theta_deg * (M_PI / 180.0));
    double rotatedY = r * std::sin(theta_deg * (M_PI / 180.0));

    //PHASE 3: Apply the wheel matrix transformation to get the motor outputs
    inputVector = {static_cast<float>(rotatedX), static_cast<float>(rotatedY)};
    BLA::Matrix<3, 1> matrixOutput = wheelMatrix * inputVector;


    //PHASE 4: Normalize motor outputs and mutiply by magnitude
    double joystickMagnitude = std::min(r, 100.0);

    double maxOutput = std::max({
        std::abs(matrixOutput(0)),
        std::abs(matrixOutput(1)),
        std::abs(matrixOutput(2))
    });

    if (maxOutput > 0.0) {
        for (int i = 0; i < 3; i++) {
            matrixOutput(i) = (matrixOutput(i) / maxOutput) * joystickMagnitude;
        }
    }

    //PHASE 5: Return the calculated wheel outputs
    std::array<float, 3> wheelOutputs = {matrixOutput(0), matrixOutput(1), matrixOutput(2)};

    return wheelOutputs;
}

// std::array<int, 3>  calcMotorPercentages() {
//     std::array<float, 3> wheelOutputs = motor_matrix_translation(command.x, command.y, command.heading);

//     Serial.printf("Translated values: M1:{%f}, M2:{%f}, M3:{%f}\n", wheelOutputs[0], wheelOutputs[1], wheelOutputs[2]);
//     // headingValid is reserved for the future heading-aware drive logic.
// }