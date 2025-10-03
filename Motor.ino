#include <Servo.h>
#include <std_msgs/Int32MultiArray.h>

// Arduino pin for trusters. 
// F means front, B, means back, V means vertical, L means left, R means right. 
// F truster go backward, B truster go forward
byte trusterPinFL= 9;
byte trusterPinFR = 10;
byte trusterPinBL= 11;
byte trusterPinBR = 12;
byte trusterPinVFL = 13;
byte trusterPinVFR = 14;
byte trusterPinVBL = 15;
byte trusterPinVBR = 16;
Servo trusterFL;
Servo trusterFR;
Servo trusterBL;
Servo trusterBR;
Servo trusterVFL;
Servo trusterVFR;
Servo trusterVBL;
Servo trusterVBR;
// Signal value for truster to move forward or backward
int forward_max = 200; 
int backward_max = -200;
int noMove = 1500;

void turnLeft(const int value)
{
  // trusterFR.writeMicroseconds(backward);
  trusterBR.writeMicroseconds(noMove + value);
  trusterFL.writeMicroseconds(noMove + value);
  // trusterBL.writeMicroseconds(backward);  
}

void turnRight(const int value)
{
  trusterFR.writeMicroseconds(noMove + value);
  // trusterBR.writeMicroseconds(backward);
  // trusterFL.writeMicroseconds(backward);
  trusterBL.writeMicroseconds(noMove + value);  
}

void goForward(const int value)
{
  trusterFR.writeMicroseconds(noMove + value);
  trusterFL.writeMicroseconds(noMove + value); 
}

void goBackward(const int value)
{
  trusterBR.writeMicroseconds(noMove + value);
  trusterBL.writeMicroseconds(noMove + value);  
}

void goUpDown(const int value)
{
  trusterVFL.writeMicroseconds(noMove - value);
  trusterVBR.writeMicroseconds(noMove - value);
  trusterVBL.writeMicroseconds(noMove + value);
  trusterVFR.writeMicroseconds(noMove + value);
}

void pitch(const int value)
{
  trusterVFL.writeMicroseconds(noMove - value);
  trusterVBR.writeMicroseconds(noMove + value);
  trusterVBL.writeMicroseconds(noMove + value);
  trusterVFR.writeMicroseconds(noMove - value);
}

void roll(const int value)
{
  trusterVFL.writeMicroseconds(noMove - value);
  trusterVBR.writeMicroseconds(noMove + value);
  trusterVBL.writeMicroseconds(noMove - value);
  trusterVFR.writeMicroseconds(noMove + value);
}

void motorCallback(const std_msgs::Int32MultiArray& msg)
{
  if (msg.data[1] > forward_max)
  {
    msg.data[1] = forward_max
  }
  if (msg.data[1] < backward_max)
  {
    msg.data[1] = backward_max
  }
  if (msg.data[0] == 0)
  {
    if (msg.data[1] >= 0)
    {
      goForward(msg.data[1]);
    }
    if (msg.data[1] <= 0)
    {
      goBackward(abs(msg.data[1]))
    }
  }
  else if (msg.data[0] == 1)
  {
    if (msg.data[1] > 0)
    {
      turnRight(msg.data[1]);
    }
    else if (msg.data[1] < 0)
    {
      turnLeft(abs(msg.data[1]))
    }
    else
    {
      turnLeft(0);
      turnRight(0);
    }
  }
  else if (msg.data[0] == 2)
  {
    goUpDown(msg.data[1])
  }
  else if (msg.data[0] == 3)
  {
    pitch(msg.data[1])
  }
  else if (msg.data[0] == 4)
  {
    roll(msg.data[1])
  }
  
}

ros::NodeHandle node_handle;

ros::Subscriber<std_msgs::Int32MultiArray> motor_subscriber("thruster", &motorCallback);

void setup() {
  trusterFL.attach(trusterPinFL);
  trusterFR.attach(trusterPinFR);
  trusterBL.attach(trusterPinBL);
  trusterBR.attach(trusterPinBR);
  trusterVFL.attach(trusterPinVFL);
  trusterVFR.attach(trusterPinVFR);
  trusterVBL.attach(trusterPinVBL);
  trusterVBR.attach(trusterPinVBR);

  // send "stop" signal to ESC to initialize it
  trusterFL.writeMicroseconds(1500);
  trusterFR.writeMicroseconds(1500);
  trusterBL.writeMicroseconds(1500);
  trusterBR.writeMicroseconds(1500);
  trusterVFL.writeMicroseconds(1500);
  trusterVFR.writeMicroseconds(1500);
  trusterVBL.writeMicroseconds(1500);
  trusterVBR.writeMicroseconds(1500);
  delay(7000); // delay to allow the ESC to recognize the stopped signal
  Serial.begin(9600);
  node_handle.initNode();
  node_handle.subscribe(motor_subscriber);
}

void loop() {
  node_handle.spinOnce();
  delay(1);
}
