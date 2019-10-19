#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
std_msgs::Int32MultiArray ultrasonic_ranges;
geometry_msgs::Twist drive_command;

long last_time_action_done_millis = 0;
const int trigPin0 = 7;
const int echoPin0 = 8;
const int trigPin1 = 9;
const int echoPin1 = 10;

long duration;
long int distances[5];

void setDriveCommand(const geometry_msgs::Twist& command) {
  drive_command = command;
}

ros::Publisher ultrasonic_pub("arduino/ultrasonic_ranges", &ultrasonic_ranges);
ros::Subscriber<geometry_msgs::Twist> command_sub("turtle1/cmd_vel", &setDriveCommand);

int measureDistance(int tPin, int ePin) {
  digitalWrite(tPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(tPin, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(tPin, LOW);

  return pulseIn(ePin, HIGH) * 0.01715;
}

void fillRanges() {
  ultrasonic_ranges.data[0] = measureDistance(trigPin0, echoPin0);
  ultrasonic_ranges.data[1] = measureDistance(trigPin1, echoPin1);
  ultrasonic_ranges.data[2] = 100;
  ultrasonic_ranges.data[3] = 100;
  ultrasonic_ranges.data[4] = 100;
}

void setup()
{
  ultrasonic_ranges.data_length = 5;
  ultrasonic_ranges.data = distances;
  
  pinMode(trigPin0, OUTPUT);
  pinMode(echoPin0,INPUT);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1,INPUT);
  
  nh.initNode();
  nh.advertise(ultrasonic_pub);
}


void loop()
{
  if (millis() > last_time_action_done_millis + 300)
  {
     last_time_action_done_millis = millis();

     fillRanges();
     ultrasonic_pub.publish( &ultrasonic_ranges );
  }

  nh.subscribe(command_sub);
  nh.spinOnce();
}
