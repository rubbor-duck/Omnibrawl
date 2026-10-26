#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "main.hpp"
#include "web_server.hpp"
#include "robot.hpp"
#include "movement.hpp"
#include "pins.hpp"
#include "motor_controller.hpp"

MotorController motorController;

void setup() {
    Serial.begin(115200);
    Pins::initializePins();
    motorController.begin();
    WebServer::begin();
    initializeTransformations();
}

Robot robot1;

std::array<float, 3> motorPercentages = {0.0, 0.0, 0.0};
int joystickX = 0;
int joystickY = 0;
int imuHeading = 0;
int phoneHeading = 0;

void loop() {
    WebServer::update();

    // 1a. Get joystick XY value (USING WEBS_SERVER.CPP)
    const ControlCommand commandToRobot = WebServer::getCommand();

    joystickX = commandToRobot.x;
    joystickY = commandToRobot.y;
    phoneHeading = commandToRobot.heading;

    // 1b. Get IMU measurements (WILL USE IMU.CPP)
    // const IMUData imuData = IMU::getData();
    imuHeading = imuHeading;

    // 2. Calculate motor percentages (USING MOVEMENT.CPP)
    motorPercentages = calcMotorPercentages(joystickX, joystickY, phoneHeading, imuHeading);
    
    //3. Store motor percentages in robot instance (USING ROBOT.CPP)
    robot1.setRobotMotorPercentages(motorPercentages[0], motorPercentages[1], motorPercentages[2]);




    static unsigned long lastPrintMs = 0;
    const unsigned long now = millis();

    if (now - lastPrintMs >= 250) {
        lastPrintMs = now;
        Serial.printf(
            "Joystick X:%d Y:%d | Motors: %.1f, %.1f, %.1f\n",
            joystickX,
            joystickY,
            robot1.getRobotMotorPercentage(0),
            robot1.getRobotMotorPercentage(1),
            robot1.getRobotMotorPercentage(2)
        );
    }

    // 3. Apply robot's motor percentages to drivers (USING MOTOR_CONTROLLER.CPP)
    motorController.setMotorPercentages(
        robot1.getRobotMotorPercentage(0), 
        robot1.getRobotMotorPercentage(1), 
        robot1.getRobotMotorPercentage(2)
    );
    
    
}
