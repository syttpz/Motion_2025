#include <ros.h>
// #include <std_msg/String.h>
#include <std_msgs/Bool.h>
// #include <std_msgs/Float64.h>
byte leak_pin = 1;

ros::NodeHandle nh;

std_msgs::Bool leak_val;
// std_msgs::Float64 depth_val;
ros::Publisher leak("leak_sensor", &leak_val);
// ros::Publisher depth("pressure_sensor", &depth_val);
bool leak;


void setup() {
  // put your setup code here, to run once:
  pinMode(leak_pin, INPUT);
  Serial.begin(9600);

  nh.initNode();
  nh.advertise(leak);
  // nh.advertise(depth);
}

void loop() {
  //Measure force from sensor
  leak = digitalRead(leak_pin)
  leak_val.data = leak
  
  
  leak.publish(&leak_val);
  // depth.publish(&depth_val);
  nh.spinOnce();
  delay(10);
}
