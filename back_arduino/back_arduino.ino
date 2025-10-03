// Arduino code for the back arduino
#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int32MultiArray.h>
// #include <MPU6050_light.h>
// #include <std_msgs/Float32MultiArray.h>

// Arduino pin for trusters. 
// F means front, B, means back, V means vertical, L means left, R means right. 
// F truster go backward, B truster go forward
byte trusterPinFL= 6;
byte trusterPinFR = 11;
byte trusterPinVFL = 5;
byte trusterPinVFR = 10;
byte trusterPinVBL = 3;
byte trusterPinVBR = 9;
Servo trusterFL;
Servo trusterFR;
Servo trusterVFL;
Servo trusterVFR;
Servo trusterVBL;
Servo trusterVBR;
// Signal value for truster to move forward or backward
int thruster_max = 300;
int noMove = 1500;

// // Timer
// unsigned short timer = 0;

// // Displacement values
// short x_disp = 0;
// short y_disp = 0;
// short z_disp = 0;

// MPU6050 mpu(Wire);


void turnLeft(const int value)
{
  // Modified to accomodate the fact that thrusterFL is not functioning
  trusterFR.writeMicroseconds(noMove - value);
  // trusterBR.writeMicroseconds(noMove + value);
  trusterFL.writeMicroseconds(noMove);
  // trusterBL.writeMicroseconds(backward);  
}

void turnRight(const int value)
{
  trusterFR.writeMicroseconds(noMove + value);
  // trusterBR.writeMicroseconds(backward);
  trusterFL.writeMicroseconds(noMove);
  // trusterBL.writeMicroseconds(noMove + value);  
}

void goForward(const int value)
{
  trusterFR.writeMicroseconds(noMove + value);
  trusterFL.writeMicroseconds(noMove + value); 
}

void goUpDown(const int value)
{
  trusterVFL.writeMicroseconds(noMove + value);
  trusterVBR.writeMicroseconds(noMove + value);
  trusterVBL.writeMicroseconds(noMove - value);
  trusterVFR.writeMicroseconds(noMove - value);
}

void roll(const int value)
{
  trusterVFL.writeMicroseconds(noMove + value);
  trusterVBR.writeMicroseconds(noMove - value); 
  trusterVBL.writeMicroseconds(noMove - value); 
  trusterVFR.writeMicroseconds(noMove + value); 
}

void pitch(const int value)
{
  trusterVFL.writeMicroseconds(noMove + value);
  trusterVBR.writeMicroseconds(noMove - value);
  trusterVBL.writeMicroseconds(noMove + value);
  trusterVFR.writeMicroseconds(noMove - value);
}

void motorCallback(const std_msgs::Int32MultiArray& msg)
{
  if (abs(msg.data[1]) > thruster_max)
  {
    if (msg.data[1] > 0){
      msg.data[1] = (int)thruster_max;
    } else {
      msg.data[1] = -1 *  (int)thruster_max;
    } 
  }
  if (msg.data[0] == 0)
  {
    goUpDown(0);
    goForward(0);
    // Modified for broken FL thruster
    // if (msg.data[1] > 0)
    // {
    //   goForward(msg.data[1]);
    // }
    // else {
    //   goForward(0);
    // }
  }
  else if (msg.data[0] == 1)
  {
    goUpDown(0);
    if (msg.data[1] > 0)
    {
      turnRight(msg.data[1]);
    }
    else if (msg.data[1] < 0)
    {
      turnLeft(abs(msg.data[1]));
    }
    else
    {
      goForward(0);
    }
  }
  else if (msg.data[0] == 2)
  {
    goForward(0);
    goUpDown(msg.data[1]);
  }
  else if (msg.data[0] == 3)
  {
    goForward(0);
    pitch(msg.data[1]);
  }
  else if (msg.data[0] == 4)
  {
    goForward(0);
    roll(msg.data[1]);
  }
  
}

ros::NodeHandle node_handle;

// std_msgs::Float32MultiArray imu_val;

ros::Subscriber<std_msgs::Int32MultiArray> motor_subscriber("/thruster", &motorCallback);
// ros::Publisher imu_pub("i", &imu_val);

void setup() {
  Serial.begin(57600);
  trusterFL.attach(trusterPinFL);
  trusterFR.attach(trusterPinFR);
  trusterVFL.attach(trusterPinVFL);
  trusterVFR.attach(trusterPinVFR);
  trusterVBL.attach(trusterPinVBL);
  trusterVBR.attach(trusterPinVBR);

  // send "stop" signal to ESC to initialize it
  trusterFL.writeMicroseconds(1500);
  trusterFR.writeMicroseconds(1500);
  trusterVFL.writeMicroseconds(1500);
  trusterVFR.writeMicroseconds(1500);
  trusterVBL.writeMicroseconds(1500);
  trusterVBR.writeMicroseconds(1500);
  delay(7000); // delay to allow the ESC to recognize the stopped signal

  // // Setup IMU
  // Wire.begin();
  // mpu.begin();
  // delay(1000);
  // mpu.calcOffsets(true,true);

  // Setup ROS
  node_handle.initNode();
  node_handle.subscribe(motor_subscriber);
  // node_handle.advertise(imu_pub);
}

void loop() {
  if (!Serial || !node_handle.connected()) {
    goForward(0);
    goUpDown(0);
  }

  // unsigned short new_time = millis();
  // mpu.update();

  // // Calculate displacement
  // x_disp = x_disp + mpu.getAccX() * (new_time-timer);
  // y_disp = y_disp + mpu.getAccY() * (new_time-timer);
  // z_disp = z_disp + mpu.getAccZ() * (new_time-timer);
  
  // float imu_data[] = {mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ(), x_disp, y_disp, z_disp};
  // imu_val.data = imu_data;
  
  // imu_pub.publish(&imu_val);
  // timer=new_time;

  node_handle.spinOnce();
}
