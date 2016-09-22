// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016

#ifndef ArduinoLab2Helper_h
#define ArduinoLab2Helper_h

#include "Arduino.h"
#include "DualMC33926MotorShield.h"
#include "LS7366.h"


//Some useful constant definitions
const float FREQ = 1000.0;                         // (Hz)
const float PERIOD = 1.0 / FREQ;
const float PERIOD_MICROS = PERIOD * 1e6;
const float SERIAL_FREQ = 100.0;                   // (Hz)
const float SERIAL_PERIOD = 1.0 / SERIAL_FREQ;
const float SERIAL_PERIOD_MICROS = SERIAL_PERIOD * 1e6;
const float b = 0.225; // (m)
const float r = 0.037; // wheel radius (m)

class EncoderMeasurement {
  public: 
    signed long encoder1CountPrev, encoder2CountPrev;  // encoder 1 and 2 counts in ticks for the previous cycle
    float v_L, v_R;                                    // left and right wheel velocity in m/s
    float dThetaL, dThetaR;
    float maxMV;
    
    EncoderMeasurement(int motor_type);                // motor_type: 26 or 53
    
    void init() {
        initEncoders();         // initialize Encoders
        clearEncoderCount();    // clear Encoder Counts
    }

    void update();
    
  private:
    const float wheelRadius = r;   // (meter)
    
    const float rev2enc_26 = 2000;
    const float gearing_26 = 26;
    
    const float rev2enc_53 = 1000;
    const float gearing_53 = 53;
    
    const float voltage = 22.2;              // (Volt)
    const float motor_const = 26.94;         // (rad/s/Volt)
    
    float enc2rev;
    float enc2rad;
    float enc2wheel;

};

class RobotPose {
  public:
    float X, Y;          // robot X,Y position in meters
    float Th;            // robot orientation in rad
    float pathDistance;  // trajectory path distance in meters
    
    RobotPose():
      Th(0),
      X(0), Y(0), pathDistance(0) {}
      
    void update(float dThetaL, float dThetaR); // update the odometry from delta in R and L wheel positions
    
};

class PIController {
  public:
    PIController(): mIntegratedVError1(0), mIntegratedVError2(0) {}
    void init() { md.init();  Serial.println("Motor Driver Initialized..."); }
    void doPIControl(String side, float desV, float currV);
    
  private:
    DualMC33926MotorShield md;
    float mIntegratedVError1, mIntegratedVError2;
    const float Kpv1 = 100,      Kpv2 = 100;      // P gain for motor 1 and 2
    const float Kiv1 = 10000,    Kiv2 = 10000;     // I gain for motor 1 and 2
};

class SerialComm {
  public:
    float desiredWV_R, desiredWV_L;
  
    SerialComm(): desiredWV_R(0), desiredWV_L(0){
        prevSerialTime = micros();
    }
    void receiveSerialData(){
        if (Serial.available() > 0) {
            String commandString = Serial.readStringUntil('\n');  // read a line
            float command[2];
            for (int i = 0, indexPointer = 0; indexPointer != -1 ; i++ ) {
                indexPointer = commandString.indexOf(',');
                String tempString = commandString.substring(0, indexPointer);
                command[i] = tempString.toFloat();
                commandString = commandString.substring(indexPointer+1);
            }
            desiredWV_R = command[0];
            desiredWV_L = command[1];
        }
    }
    void send(const RobotPose& robotPose) {
        unsigned long current_time = micros();
        if (current_time - prevSerialTime >= SERIAL_PERIOD_MICROS) {
            Serial.print(robotPose.X, 6);   Serial.print(",");  //X 
            Serial.print(robotPose.Y, 6);   Serial.print(",");  //Y 
            Serial.println(robotPose.Th);                       //Th
            prevSerialTime = current_time;
        }
    }
  private: 
    unsigned long prevSerialTime;
};


#endif


