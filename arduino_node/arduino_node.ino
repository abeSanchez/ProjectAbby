#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
std_msgs::Float32MultiArray ultrasonic_ranges;
geometry_msgs::Twist drive_command;

void setDriveCommand(const geometry_msgs::Twist& command) {
  drive_command = command;
  digitalWrite(13, HIGH-digitalRead(13));
}

ros::Publisher ultrasonic_pub("arduino/ultrasonic_ranges", &ultrasonic_ranges);
ros::Subscriber<geometry_msgs::Twist> command_sub("turtle1/cmd_vel", &setDriveCommand);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  ultrasonic_ranges.data_length = 5;
  nh.advertise(ultrasonic_pub);
}

void loop()
{
  fillRanges();
  ultrasonic_pub.publish( &ultrasonic_ranges );
  nh.subscribe(command_sub);
  nh.spinOnce();
  delay(1000);
}

void fillRanges() {
  int i;

  for (i = 0; i < 5; i++) {
      ultrasonic_ranges.data[i] = i;
  }
}
