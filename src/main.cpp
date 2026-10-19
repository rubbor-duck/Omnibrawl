#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "main.hpp"
#include "web_server.hpp"

namespace {
BLA::Matrix<3, 2> wheelMatrix;
BLA::Matrix<2> inputVector;
}

void initializeDrive() {
    // Same 3-wheel transformation matrix as the original.
    wheelMatrix = {1, 0, -0.5, 0.866025403784, -0.5, -0.866025403784};
    inputVector.Fill(0);
}

void motor_matrix_translation(int x, int y, int heading) {
    inputVector = {x, y};
    BLA::Matrix<3, 1> wheelOutputs = wheelMatrix * inputVector;
    // Same as the original: results are calculated but not applied to motors.
    (void)wheelOutputs;
    (void)heading;
}

void updateDrive() {
    const ControlCommand command = WebServer::getCommand();
    motor_matrix_translation(command.x, command.y, command.heading);
    Serial.println("");

    // headingValid is reserved for the future heading-aware drive logic.
}

void setup() {
    Serial.begin(115200);
    WebServer::begin();
    initializeDrive();
}

void loop() {
    WebServer::update();
    updateDrive();
}
