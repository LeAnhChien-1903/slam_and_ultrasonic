#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

// My project libraries 
#include "ultrasonic.h"
#include "stepper.h"
// #include "MPU6050.h"
ros::NodeHandle nh;

//Create six instances for range messages
std_msgs::Float32 range0;
std_msgs::Float32 range45;
std_msgs::Float32 range90;
std_msgs::Float32 range135;
std_msgs::Float32 range180;
std_msgs::Float32 range270; 
// Create instances for MPU6050 message
//std_msgs::Float32 MPU6050;
// Create publisher objects for all ultrasonic and MPU6050
ros::Publisher pub_range0("/robot/sensor/sonar0", &range0);
ros::Publisher pub_range45("/robot/sensor/sonar45", &range45);
ros::Publisher pub_range90("/robot/sensor/sonar90", &range90);
ros::Publisher pub_range135("/robot/sensor/sonar135", &range135);
ros::Publisher pub_range180("/robot/sensor/sonar180", &range180);
ros::Publisher pub_range270("/robot/sensor/sonar270", &range270);
//ros::Publisher pub_MPU6050("/robot/sensor/MPU6050", &MPU6050);
// Create subscriber objects for motor
void rosControlMotor(const std_msgs::Float32MultiArray& params)
{
    float omegaLeft = params.data[0];
    float omegaRight = params.data[1];
    uint16_t steps = uint16_t(params.data[2]);
    if (omegaLeft != 0 && omegaRight == 0)
    {
        digitalWrite(EnPinLeft, LOW);
        digitalWrite(EnPinRight, HIGH);
        turnRight(omegaLeft, steps);
    }
    else if (omegaRight != 0 && omegaLeft == 0)
    {
        digitalWrite(EnPinLeft, HIGH);
        digitalWrite(EnPinRight, LOW);
        turnLeft(omegaRight, steps);
    }
    else if (omegaLeft != 0 && omegaRight != 0)
    {
        digitalWrite(EnPinLeft, LOW);
        digitalWrite(EnPinRight, LOW);
        goStraight(omegaLeft, steps);
    }
    else
    {
        digitalWrite(EnPinLeft, HIGH);
        digitalWrite(EnPinRight, HIGH);
    }
    digitalWrite(EnPinLeft, HIGH);
    digitalWrite(EnPinRight, HIGH);
}
ros::Subscriber<std_msgs::Float32MultiArray> subs_controlMotor("/robot/motor", &rosControlMotor); 
// Prototype function
void ultrasonicReader();
void MPU6050reader();

void setup()
{
    nh.initNode();
    nh.subscribe(subs_controlMotor);
//    nh.advertise(pub_MPU6050);
    nh.advertise(pub_range0);
    nh.advertise(pub_range45);
    nh.advertise(pub_range90);
    nh.advertise(pub_range135);
    nh.advertise(pub_range180);
    nh.advertise(pub_range270);
//    MPU6050_setup();
    ultrasonicInit();
    motorSetup();
}
void loop()
{
  //  MPU6050reader();
    ultrasonicReader();
    nh.spinOnce(); // Handles ROS event
}
void ultrasonicReader()
{
    ultrasonicCycle();
    applyKF();
    if (sensor0Kalman <= 200)
    {
        range0.data = sensor0Kalman*0.01;
    }
    else range0.data = 2.5;
    if (sensor45Kalman <= 200)
    {
        range45.data = sensor45Kalman*0.01;
    }
    else range45.data = 2.5;
    if (sensor90Kalman <= 200)
    {
        range90.data = sensor90Kalman*0.01;
    }
    else range90.data = 2.5;
    if (sensor135Kalman <= 200)
    {
        range135.data = sensor135Kalman*0.01;
    }
    else range135.data = 2.5;
    if (sensor180Kalman <= 200)
    {
        range180.data = sensor180Kalman*0.01;
    }
    else range180.data = 2.5;
    if (sensor270Kalman <= 200)
    {
        range270.data = sensor270Kalman*0.01;
    }
    else range270.data = 2.5;
    // Publish data 
    pub_range0.publish(&range0);
    pub_range45.publish(&range45);
    pub_range90.publish(&range90);
    pub_range135.publish(&range135);
    pub_range180.publish(&range180);
    pub_range270.publish(&range270);  
}
// void MPU6050reader()
// {
//     applyKalmanMPU();
//     MPU6050.data = yawAngle;
//     pub_MPU6050.publish(&MPU6050);
// }