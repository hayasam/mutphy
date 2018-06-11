//  ∗∗
//  ∗ Group number: 21
//  ∗ Student 1:
//  ∗ Daan van der Valk, 4094751
//  ∗ Student 2:
//  ∗ Stefan de Vringer, 4374851
//  ∗/


// Includes required for ROS and the Twist data structure
#include <ros.h>
#include <geometry_msgs/Twist.h>


void reset_movement();

// SETTINGS
// Settings for obstacle detection (see requirements)
// Robot will be stopped for some obstacle if it’s at 20 cm distance or closer
const int maximumRange = 20;

// Setting for timer: the register value is set to reset_timer after initiating movement
const int reset_timer = 40000;

// Setting for speed
// Only used for the directions commanded by our line follower application
const int robot_speed = 63;


// CONSTANTS
// Pin numbers: robot motor
const int p1REV = 7;
const int p1EN = 24;
const int p1FWD = 6;
const int p2REV = 3;
const int p2EN = 25;
const int p2FWD = 2;

// Pin numbers: ultrasonic sensor 
const int echoPin = 22;
const int triggerPin = 23;

// Pin number: yellow LED
const int ledPin = 13;

// Used for distance calculation of the nearest object
long duration, distance;



// ISR for the timer overflow:
// in this case, the connection with the control (laptop) has timed out
// Stop robot untill new signals are received
ISR(TIMER1_OVF_vect)
{
  TCNT1 = reset_timer;            // reset timer
  reset_movement();
}



// MOVEMENT FUNCTIONS
// Keep engines active, but not moving
void reset_movement() {
  digitalWrite(p1EN, HIGH);
  digitalWrite(p2EN, HIGH);
  digitalWrite(p1FWD, LOW);
  digitalWrite(p2FWD, LOW);
  digitalWrite(p1REV, LOW);
  digitalWrite(p2REV, LOW);
}

// 3 functions for moving forwards
void movement_straight_forward() {
  reset_movement();
  analogWrite(p1FWD, robot_speed);
  analogWrite(p2FWD, robot_speed);
}
void movement_left_forward() {
  reset_movement();
  analogWrite(p2FWD, robot_speed);
}
void movement_right_forward() {
  reset_movement();
  analogWrite(p1FWD, robot_speed);
}

// 1 stop function
void movement_stop() {
  reset_movement();
}

// 2 rotation functions
void movement_left_rotation() {
  reset_movement();
  digitalWrite(p1REV, HIGH);
  digitalWrite(p2FWD, HIGH);
}

void movement_right_rotation() {
  reset_movement();
  digitalWrite(p1FWD, HIGH);
  digitalWrite(p2REV, HIGH);
}

// 3 functions for moving backwards
void movement_straight_backward() {
  reset_movement();
  digitalWrite(p1REV, HIGH);
  digitalWrite(p2REV, HIGH);
}
void movement_left_backward() {
  reset_movement();
  digitalWrite(p2REV, HIGH);
}
void movement_right_backward() {
  reset_movement();
  digitalWrite(p1REV, HIGH);
}


// Control function for handling incoming Twist messages
void inputHandler(const geometry_msgs::Twist& msg) {
  // Retrieve linear and angular values
  float linear = msg.linear.x;
  float angular = msg.angular.z;
  
  // Linear > 0: movement forwards
  if(linear > 0) {
    if(angular > 0)
      movement_left_forward();
    else if(angular == 0)
      movement_straight_forward();
    else
      movement_right_forward();
      
  // Linear = 0: stop or rotate
  } else if(linear == 0) {
    if(angular > 0)
      movement_left_rotation();
    else if(angular == 0)
      movement_stop();
    else
      movement_right_rotation();
      
  // Linear < 0: movement backwards
  } else {
    if(angular > 0)
      movement_left_backward();
    else if(angular == 0)
      movement_straight_backward();
    else
      movement_right_backward();
  }
  
  // Set timing register to reset clock
  TCNT1 = reset_timer;
}


// Create NodeHandle to connect to ROS as a node
class NewHardware : public ArduinoHardware {
  public : NewHardware():ArduinoHardware(&Serial1, 57600){};
}; ros::NodeHandle_<NewHardware> nh;
// Create subscriber to receive Twist messages
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &inputHandler);


// Setup Node
void setup() {
    nh.initNode();          // Start ROS node
    nh.subscribe(sub);      // Subscribe to /cmd_vel topic
  
    // Set control pins as output
    pinMode(ledPin, OUTPUT);
    
    pinMode(p1REV, OUTPUT);
    pinMode(p1EN, OUTPUT);
    pinMode(p1FWD, OUTPUT);
    pinMode(p2REV, OUTPUT);
    pinMode(p2EN, OUTPUT);
    pinMode(p2FWD, OUTPUT);
    
    pinMode(triggerPin, OUTPUT);
    
    // Set echo pin as input
    pinMode(echoPin, INPUT);
    
    // initialize timer1 
    noInterrupts();           // Disable all interrupts temporarely
    TCCR1A = 0;               // Timer1 control register A to 0
    TCCR1B = 0;               // Timer1 control register B to 0
    TCNT1 = reset_timer;      // Set Timer1 register to reset_timer
    
    //Set prescalar to 256 (16MHz / 256)
    TCCR1B |= (1 << CS12);
    TIMSK1 |= (1 << TOIE1);   // Enable overflow interrupt
    interrupts();             // Enable interrupts again
}



// Repeated code used to actively control the robot
void loop() {
  // Use ultrasonic sensor to measure the closest object in front of the robot
  digitalWrite(triggerPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
   
  // Calculate the distance (in cm) of the object
  distance = duration/58;

  // If the obstacle is too close, put on LED and do not move
  if (distance <= maximumRange) {
    digitalWrite(ledPin, HIGH);
  // Otherwise there is no near obstacle and the robot may move by receiving Twist messages
  } else {
    nh.spinOnce();
    digitalWrite(ledPin, LOW);
  }
  
  delay(1);
}
