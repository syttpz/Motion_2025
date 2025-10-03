// Arduino code for the front arduino
// Leak publisher is disabled because the leak sensor seems to be malfunctioning, uncomment relevent code once the sensor is repalced. 

#include <ros.h>
// #include <std_msgs/Bool.h>
#include <Servo.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <DHT11.h>


byte leak_pin = 1;
byte temperature_pin = 2;
byte light1_pin = 3;
byte light2_pin = 5;
byte trusterPinBL= 10;
byte trusterPinBR = 11;
Servo trusterBL;
Servo trusterBR;
DHT11 dht11(temperature_pin);

// Signal value for truster to move forward or backward
int thruster_max = 300; 
int noMove = 1500;

// ROS subscribers
ros::NodeHandle nh;
// std_msgs::Bool leak_val;
std_msgs::Float32 temp_val;
std_msgs::Float32 info_val;

// ros::Publisher leak_pub("leak", &leak_val);
// ros::Publisher info_pub("info", &info_val);
ros::Publisher temperature_pub("temp", &temp_val);


void turnLeft(const int value)
{
  // modified to accomodate the fact that FL thruster is not functioning.
  // trusterFR.writeMicroseconds(backward);
  trusterBR.writeMicroseconds(noMove);
  // trusterFL.writeMicroseconds(noMove + value);
  trusterBL.writeMicroseconds(noMove - value);  
}

void turnRight(const int value)
{
  // trusterFR.writeMicroseconds(noMove + value);
  trusterBR.writeMicroseconds(noMove);
  // trusterFL.writeMicroseconds(backward);
  trusterBL.writeMicroseconds(noMove + value);  
}

void goBackward(const int value)
{
  trusterBR.writeMicroseconds(noMove + value);
  trusterBL.writeMicroseconds(noMove + value);  
}

void goForward(const int value)
{
  trusterBR.writeMicroseconds(noMove - value);
  trusterBL.writeMicroseconds(noMove - value);  
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
  if (msg.data[0] == 1)
  {
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
      goBackward(0);
    }
  }
  else if (msg.data[0] == 0)
  {
    if (msg.data[1] <= 0)
    {
      goBackward(abs(msg.data[1]));
    }
    else {
      goForward(msg.data[1]);
    }
  }
  else {
    goBackward(0);
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> motor_subscriber("/thruster", &motorCallback);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  // Setup pins
  pinMode(leak_pin, INPUT);
  pinMode(light1_pin, OUTPUT);
  pinMode(light2_pin, OUTPUT);

  // Setup thrusters
  trusterBL.attach(trusterPinBL);
  trusterBR.attach(trusterPinBR);
  trusterBL.writeMicroseconds(1500);
  trusterBR.writeMicroseconds(1500);

  delay(7000); // delay to allow the ESC to recognize the stopped signal
  
  // Turn on light
  digitalWrite(light1_pin, HIGH);
  digitalWrite(light2_pin, HIGH);

  // Set up ROS node
  nh.initNode();
  // nh.advertise(leak_pub);
  // nh.advertise(info_pub);
  nh.advertise(temperature_pub);
  nh.subscribe(motor_subscriber);
}

void loop() {
  if (!Serial || !nh.connected()) {
    goForward(0);
    digitalWrite(light1_pin, LOW);
    digitalWrite(light2_pin, LOW);
  }
  else {
    digitalWrite(light1_pin, HIGH);
    digitalWrite(light2_pin, HIGH);
  }
  //Measure from sensor
  // bool leak;
  short temperature;
  // leak = digitalRead(leak_pin);
  // leak_val.data = leak;
  temperature = dht11.readTemperature();
  temp_val.data = temperature;
  // leak_pub.publish(&leak_val);
  temperature_pub.publish(&temp_val);

  nh.spinOnce();
}
