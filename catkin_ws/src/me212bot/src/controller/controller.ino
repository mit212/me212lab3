// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016

#include "Arduino.h"
#include "helper.h"

EncoderMeasurement  encoder(26);      // encoder handler class, set the motor type 53 or 26 here
RobotPose           robotPose;        // robot position and orientation calculation class
PIController        wheelVelCtrl;     // velocity PI controller class
SerialComm          serialComm;       // serial communication class
unsigned long       prevTime = 0;

boolean usePathPlanner = true;

void setup() {
    Serial.begin(115200);       // initialize Serial Communication
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor
    delay(1e3);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    unsigned long currentTime = micros();
    
    if (currentTime - prevTime >= PERIOD_MICROS) {
      
        // 1. Check encoder
        encoder.update(); 

        // 2. Update position
        robotPose.update(encoder.dThetaL, encoder.dThetaR); 

        // 3. Send odometry through serial communication
        serialComm.send(robotPose); 
        serialComm.receiveSerialData();

        // 4. Send the velocity command to wheel velocity controller
        wheelVelCtrl.doPIControl("Left",  serialComm.desiredWV_L, encoder.v_L); 
        wheelVelCtrl.doPIControl("Right", serialComm.desiredWV_R, encoder.v_R);

        prevTime = currentTime; // update time
    } 
}




