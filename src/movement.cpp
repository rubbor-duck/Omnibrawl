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
    wheelMatrix = {1, 0, -0.5, 0.866025403784, -0.5, -0.866025403784};
    inputVector.Fill(0);
}

//TO DO: ADD IMU DATA FOR HEADER
std::array<float, 3> calcMotorPercentages(int x, int y, int phoneHeading, int imuHeading) {
    inputVector = {static_cast<float>(x), static_cast<float>(y)};
    BLA::Matrix<3, 1> matrixOutput = wheelMatrix * inputVector;

    imuHeading = imuHeading; //DO NOTHING WITH IMUHEADER
    phoneHeading = phoneHeading; //DO NOTHING WITH PHONEHEADING

    std::array<float, 3> wheelOutputs = {matrixOutput(0), matrixOutput(1), matrixOutput(2)};
    return wheelOutputs;
}

// std::array<int, 3>  calcMotorPercentages() {
//     std::array<float, 3> wheelOutputs = motor_matrix_translation(command.x, command.y, command.heading);

//     Serial.printf("Translated values: M1:{%f}, M2:{%f}, M3:{%f}\n", wheelOutputs[0], wheelOutputs[1], wheelOutputs[2]);
//     // headingValid is reserved for the future heading-aware drive logic.
// }