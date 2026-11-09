#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "main.hpp"
#include "web_server.hpp"
#include "robot.hpp"

namespace {
    // The wheel matrix converts a desired X/Y movement vector into a set of
    // three wheel commands for an omni-directional robot.
    // desiredDirectionVector holds the joystick command after it has been rotated
    // relative to the phone heading so the robot can move in the desired direction.
    BLA::Matrix<3, 2> omniWheelMatrix;
    BLA::Matrix<2> desiredDirectionVector;
}

void initializeTransformations() {
    // This is the kinematic model for a 3-wheel omni-drive robot.
    // Each row represents one wheel, and the columns represent the X/Y motion
    // components needed to move the robot in any direction.
    omniWheelMatrix = {1, 0, -0.5, 0.866, -0.5, -0.866};
    desiredDirectionVector.Fill(0);
}

// TODO: Add IMU heading support so the robot can correct for drift or rotate
// using real orientation data instead of only the phone heading.
std::array<float, 3> calcMotorPercentages(int x, int y, int phoneHeading, int imuHeading) {

    // PHASE 1: Convert the joystick input into polar coordinates and rotate it
    // to match the phone's current heading. This makes movement relative to the
    // controller orientation instead of the robot's fixed frame.
    imuHeading = imuHeading; // Placeholder until the IMU is integrated.

    // 1.1 Calculate the stick magnitude using std::hypot (avoids overflow/underflow)
    double stickMagnitude = std::hypot(x, y);

    // 1.2 Calculate the angle in degrees using std::atan2.
    double thetaDegrees = std::atan2(y, x) * (180.0 / M_PI);

    // 1.3 Adjust the angle relative to the phone heading.
    thetaDegrees = thetaDegrees + phoneHeading;

    // PHASE 2: Convert the rotated polar vector back to Cartesian coordinates.
    // This gives the robot's desired X/Y motion after accounting for heading.
    double rotatedXComponent = stickMagnitude * std::cos(thetaDegrees * (M_PI / 180.0));
    double rotatedYComponent = stickMagnitude * std::sin(thetaDegrees * (M_PI / 180.0));

    // PHASE 3: Apply the wheel matrix transformation to determine the three
    // wheel commands needed to achieve the desired robot motion.
    desiredDirectionVector = {static_cast<float>(rotatedXComponent), static_cast<float>(rotatedYComponent)};
    BLA::Matrix<3, 1> matrixOutput = omniWheelMatrix * desiredDirectionVector;

    // PHASE 4: Normalize the output so the largest motor command is limited to
    // the joystick magnitude. This prevents any wheel from exceeding the command
    // range while preserving the relative direction of the motion vector.
    double joystickMagnitude = std::min(stickMagnitude, 100.0);

    double maxOutput = std::max({
        std::abs(matrixOutput(0)),
        std::abs(matrixOutput(1)),
        std::abs(matrixOutput(2))
    });

    if (maxOutput > 0.0) {
        for (int wheelIndex = 0; wheelIndex < 3; wheelIndex++) {
            matrixOutput(wheelIndex) = (matrixOutput(wheelIndex) / maxOutput) * joystickMagnitude;
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