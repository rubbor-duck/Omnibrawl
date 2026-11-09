#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "main.hpp"
#include "web_server.hpp"
#include "robot.hpp"
#include "movement.hpp"
#include "pins.hpp"
#include "motor_controller.hpp"

// Main application entry point for the robot.
// This file wires together the hardware, the web controller, and the motion
// logic. The robot reads commands from the phone, converts them into wheel
// percentages, and sends those values to the motor driver.
MotorController motorController;

void setup() {
    // Initialize the robot on startup:
    // 1. Open serial for debugging output.
    // 2. Configure all GPIO pins used by the motors.
    // 3. Start the motor driver and web server.
    // 4. Prepare the wheel-transformation math used for omni-directional motion.
    Serial.begin(115200);
    Pins::initializePins();
    motorController.begin();
    WebServer::begin();
    initializeTransformations();
}

Robot robot;

std::array<float, 3> wheelPercentages = {0.0, 0.0, 0.0};
int joystickX = 0;
int joystickY = 0;
int imuHeadingDegrees = 0;
int phoneHeadingDegrees = 0;

void loop() {
    // Update the web-side controller state before each movement cycle.
    WebServer::update();

    // 1a. Read the latest command from the phone/web page.
    // The web server stores joystick X/Y and the heading reported by the phone.
    const ControlCommand controllerCommand = WebServer::getCommand();

    joystickX = controllerCommand.x;
    joystickY = controllerCommand.y;
    phoneHeadingDegrees = controllerCommand.heading;

    // 1b. Read the IMU heading when that subsystem is connected.
    // This placeholder keeps the variable stable until the actual IMU code is added.
    // const IMUData imuData = IMU::getData();
    imuHeadingDegrees = imuHeadingDegrees;

    // 2. Convert the joystick command into per-wheel motor percentages.
    // This is the motion-control step that turns a desired direction into
    // wheel speeds for the 3-wheel omni-drive system.
    wheelPercentages = calcMotorPercentages(joystickX, joystickY, phoneHeadingDegrees, imuHeadingDegrees);

    // 3. Store the computed wheel values in the Robot object.
    // This keeps the robot model synchronized with the most recent control command.
    robot.setRobotMotorPercentages(wheelPercentages[0], wheelPercentages[1], wheelPercentages[2]);

    static unsigned long lastPrintMs = 0;
    const unsigned long now = millis();

    if (now - lastPrintMs >= 250) {
        lastPrintMs = now;
        Serial.printf(
            "Joystick X:%d Y:%d | Motors: %.1f, %.1f, %.1f\n",
            joystickX,
            joystickY,
            robot.getRobotMotorPercentage(0),
            robot.getRobotMotorPercentage(1),
            robot.getRobotMotorPercentage(2)
        );
    }

    // 3. Apply robot's motor percentages to drivers (USING MOTOR_CONTROLLER.CPP)
    motorController.setMotorPercentages(
        robot.getRobotMotorPercentage(0), 
        robot.getRobotMotorPercentage(1), 
        robot.getRobotMotorPercentage(2)
    );
    
    
}
