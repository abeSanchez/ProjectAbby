#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
std_msgs::Int32MultiArray ultrasonic_ranges;

long last_time_action_done_millis = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define enA 5 // Motors
#define in1 6
#define in2 7

#define frontTrig 10 // Ultrasounds Front
#define frontEcho 9

long int distances[5];
float csThrottleValue = 0.0;
float csSteeringValue = 0.0;

ros::Publisher ultrasonic_pub("arduino/ultrasonic_ranges", &ultrasonic_ranges);
ros::Subscriber<geometry_msgs::Twist> command_sub("turtle1/cmd_vel", &setDriveCommand);

void setDriveCommand(const geometry_msgs::Twist& command) {
 csThrottleValue = (float)command.linear.x;
 csSteeringValue = (float)command.angular.z;
}

void fillRanges() {
  ultrasonic_ranges.data[0] = readUltrasound(frontTrig, frontEcho);
  ultrasonic_ranges.data[1] = 1000;
  ultrasonic_ranges.data[2] = 1000;
  ultrasonic_ranges.data[3] = 1000;
  ultrasonic_ranges.data[4] = 1000;
}

void setSteerAngle(float steerAngle) {
  int convertedSteerAngle = map(steerAngle, -1, 1, 100, 550);
  pwm.setPWM(0, 0, convertedSteerAngle);
}

void setThrottleESC(float throttle) { //ESC
  int convertedThrottle = map(throttle, -1, 1, 0, 1000);
  pwm.setPWM(1, 0, convertedThrottle);
}

void setThrottle(float throttle) { //Direct Control
  int convertedThrottle = abs(map(throttle, -1, 1, -255, 255));

  int forward = 0;
  int backward = 1;
  int rotationDirection = forward;

  if (throttle > 0) { 
    rotationDirection = forward; 
  }

  if (rotationDirection == forward) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  if (throttle < 0) { 
    rotationDirection = backward; 
  }

  if (rotationDirection == backward) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  analogWrite(enA, convertedThrottle);
}

int readUltrasound(int trigPin, int echoPin) {
  int duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // 0.00001 seconds of reading
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; // Convert distance to centimeters

  return distance;
}

void setup() {
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);

  pinMode(enA, OUTPUT); // Motors
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  pinMode(frontTrig, OUTPUT); // Ultrasound
  digitalWrite(frontTrig, LOW);
  pinMode(frontEcho, INPUT);
}

void loop() {
  if (millis() > last_time_action_done_millis + 300)
  {
     last_time_action_done_millis = millis();

     fillRanges();
     ultrasonic_pub.publish( &ultrasonic_ranges );
  }

  setSteerAngle(0);
  setThrottle(0);
  
  int frontDistance = readUltrasound(frontTrig, frontEcho);

  nh.subscribe(command_sub);
  nh.spinOnce();
}