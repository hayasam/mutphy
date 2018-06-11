/**
 * Group number: 34
 * Student 1:
 * Tom van der Meulen, 4189558
 * Student 2:
 * Levi Dekker, 4224175
 */

#include <ros.h>
//#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

// Arduino MEGA connected pins
#define HB_1REV   7
#define HB_1EN    24
#define HB_1FWD   6
#define HB_2REV   3
#define HB_2EN    25
#define HB_2FWD   2
#define trigPin   23
#define echoPin   22
#define led       13


/**
 * Declarations
 */

class NewHardware : public ArduinoHardware {
  public: NewHardware():ArduinoHardware(&Serial1 , 57600){};
};

// ROS parameters
//ros::NodeHandle nh;
ros::NodeHandle_<NewHardware> nh;

// Global parameters
double vmax_ = 0.16;
double vl_, vr_;
unsigned long t_ = millis();
int time_ok_;

// Function declarations
void clip_v(double *vl, double *vr);
void convert2motor(geometry_msgs::Twist twist);
void messageCb(const geometry_msgs::Twist &twist);
long getdistance();
double distancefactor();
void writespeed(double vl, double vr);

//ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb );


/**
 * Implementation
 */

void clip_v(double *vl, double *vr) {
  int isclipped = 0;

  if (abs(*vl) > vmax_) {
    *vr = *vr * vmax_/ abs(*vl);
    *vl = *vl * vmax_/ abs(*vl);
    isclipped = 1;
  }

  if (abs(*vr) > vmax_) {
    *vl = *vl * vmax_ / abs(*vr);
    *vr = *vr * vmax_ / abs(*vr);
    isclipped = 1;
  }

  if (isclipped) {
    //nh.logwarn("Motor speeds have been scaled down to ensure that they do not exceed vmax");
  }
}

void convert2motor(geometry_msgs::Twist twist) {
  
  double lx = twist.linear.x;
  double az = twist.angular.z; 

  double rotsp = az * 0.0475;

  digitalWrite(HB_1EN, HIGH);
  digitalWrite(HB_2EN, HIGH);
  
  double vl = (-rotsp + lx);
  double vr = (rotsp + lx);

  //Serial.println("Hoi");
  //nh.loginfo("Hoi");
  
  clip_v(&vl, &vr);
  
  vl_ = vl;
  vr_ = vr;
}

void messageCb(const geometry_msgs::Twist &twist) { // const std_msgs::Empty& toggle_msg){
  t_ = millis();
  time_ok_ = 1;
  
  convert2motor(twist);
}

long getdistance(){
  long duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/58);
  
  return distance;
}

double distancefactor(){
  double distance = (double) getdistance();
  double df;
  
  if (distance < 15) {
    df = 0;
  } else if (distance < 30) {
    digitalWrite(led,HIGH);
    df = ( distance)/30;
  }
  else {
    digitalWrite(led,LOW);
    df = 1;
  }
  
  return df;
}

void writespeed(double vl, double vr) {
  if (1) { //(millis()-t_) < 100
    if (vl >= 0) {
      analogWrite(HB_1FWD, 255*vl/vmax_);
      analogWrite(HB_1REV, 0);
    } else {
      analogWrite(HB_1FWD, 0);
      analogWrite(HB_1REV, -255*vl/vmax_);
    }
    if (vr >= 0) {      
      analogWrite(HB_2FWD, 255*vr/vmax_);
      analogWrite(HB_2REV, 0);
    } else {
      analogWrite(HB_2FWD, 0);
      analogWrite(HB_2REV, -255*vr/vmax_);
    }
  } else {
    analogWrite(HB_1FWD, 0);
    analogWrite(HB_1REV, 0);
    analogWrite(HB_2FWD, 0);
    analogWrite(HB_2REV, 0);
  }
}

void checkTime() {
  int dt = millis() - t_;

  if (dt > 500) {
    time_ok_ = 0;
  }
}

void setup()
{ 
  pinMode(HB_1REV, OUTPUT);
  pinMode(HB_1EN, OUTPUT);
  pinMode(HB_1FWD, OUTPUT);
  pinMode(HB_2REV, OUTPUT);
  pinMode(HB_2EN, OUTPUT);
  pinMode(HB_2FWD, OUTPUT);

  //Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  time_ok_ = 0;
}

void loop()
{  
  nh.spinOnce();
  checkTime();
  double df = (double) distancefactor();
  writespeed(vl_*df*time_ok_, vr_*df*time_ok_);
  delay(1);
}
