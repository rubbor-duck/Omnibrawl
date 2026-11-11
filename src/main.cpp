#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "scheduler_support.hpp"
#include <algorithm>
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "main.hpp"
#include "web_server.hpp"
#include "robot.hpp"
#include "movement.hpp"
#include "pins.hpp"
#include "motor_controller.hpp"

TaskHandle_t WebServerHandle = NULL;
TaskHandle_t imuSensorHandle = NULL;
TaskHandle_t motorCalcHandle = NULL;

Robot robot1;

std::array<float, 3> motorPercentages = {0.0, 0.0, 0.0};
int joystickX = 0;
int joystickY = 0;
int imuHeading = 0;
int phoneHeading = 0;

void webServerTask(void *parameter){
    for(;;){
        WebServer::update();
        // 1a. Get joystick XY value (USING WEBS_SERVER.CPP)
        const ControlCommand commandToRobot = WebServer::getCommand();

        joystickX = commandToRobot.x;
        joystickY = commandToRobot.y;
        phoneHeading = commandToRobot.heading;
    }
}

//TODO: Add more to imuSensor once the IMU gets fully implemented
void imuSensor(void *parameter){
    for(;;){
        imuHeading = imuHeading;
    }
}

void motorCalc(void *parameter){
    for(;;){
        // 2. Calculate motor percentages (USING MOVEMENT.CPP)
        motorPercentages = calcMotorPercentages(joystickX, joystickY, phoneHeading, imuHeading);
        
        //3. Store motor percentages in robot instance (USING ROBOT.CPP)
        robot1.setRobotMotorPercentages(motorPercentages[0], motorPercentages[1], motorPercentages[2]);
    }
}

// Made the allocaed stack size 2048 bytes for now
// Will need to change the size once we figure out each tasks stack usage
// Made all priorities the same so it will do a round robin
void createRTOSTasks(){
    xTaskCreate(
        webServerTask,
        "Webserver",
        10000,
        NULL,
        3,
        &WebServerHandle
    );

    xTaskCreate(
        imuSensor,
        "imuSensor",
        2048,
        NULL,
        3,
        &imuSensorHandle
    );

    xTaskCreate(
        motorCalc,
        "motorCalc",
        2048,
        NULL,
        3,
        &motorCalcHandle
    );
}



MotorController motorController;

void setup(){
    Serial.begin(115200);
    Pins::initializePins();
    motorController.begin();
    WebServer::begin();
    initializeTransformations();

    createRTOSTasks();
}

void loop(){

}