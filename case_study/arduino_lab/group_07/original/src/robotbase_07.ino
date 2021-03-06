/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

bool toggled = false;
float TURN_RATE = 255.0/90;

// Map pins to a constant
const int LEFT_FOR = 6;
const int LEFT_BACK = 7;
const int RIGHT_FOR = 2;
const int RIGHT_BACK = 3;
const int trigger = 23;
const int echo = 22;
const int led  = 13;

// Global speed of the car
float distanceSpeed = 0;
float leftForSpeed = 0;
float leftBackSpeed = 0;
float rightForSpeed = 0;
float rightBackSpeed = 0;

// Last time recorded and timeout delay.
unsigned long lastSpeed = 0;
const unsigned long TIMEOUT = 1000; // in ms.


void updateSpeeds() {
    distanceSpeed = distanceCheck();

    // Check if there is a need for timeout.
    if(millis() - TIMEOUT > lastSpeed) {
        distanceSpeed = 0; // Just set the distance speed to 0, so the car stops.
    }

    // Update all speeds based on the new distanceSpeed value
    analogWrite(RIGHT_BACK, rightBackSpeed * distanceSpeed);
    analogWrite(RIGHT_FOR, rightForSpeed * distanceSpeed);
    analogWrite(LEFT_BACK, leftBackSpeed * distanceSpeed);
    analogWrite(LEFT_FOR, leftForSpeed * distanceSpeed);
    return;
}

float distanceCheck() {
    digitalWrite(trigger, LOW);
    digitalWrite(trigger, HIGH);
    digitalWrite(trigger, LOW);
    // Check the distance of the closest object
    float duration = pulseIn(echo, HIGH);
    // Convert into cm
    float distance = (duration / 2.0) / 29.0; // From the data sheet of the SR04

    // Create mapping from cm to speed.
    if(distance < 10) {
        return 0.0;
    } else if (distance < 20) {
        return 0.65;
    } else if (distance < 30) {
        return 0.75;
    } else if (distance < 40) {
        return 0.85;
    } else {
        return 1.0;
    }
}

/*
 * This function handles the logic for making a turn to the left.
 * It will gradually turn based on the twist_msg.angular.z value.
 */
void leftTurn(const geometry_msgs::Twist& twist_msg) {
    
    float turn = (twist_msg.linear.x + twist_msg.angular.z * 2);
    float turnSpeed = turn < 0 ? 0 : turn;
    
    // Calculate the speed of the tracks
    // Right track
    rightBackSpeed = LOW;
    float speed = (turnSpeed + (twist_msg.linear.x - turnSpeed) * 0.5) * distanceSpeed;
    rightForSpeed = (turnSpeed + (twist_msg.linear.x - turnSpeed) * 0.5);
    analogWrite(RIGHT_BACK, LOW);   // Reverse
    analogWrite(RIGHT_FOR, speed);   // Forward
    
    // Left track
    leftForSpeed = turnSpeed;
    leftBackSpeed = LOW;
    analogWrite(LEFT_BACK, LOW);   // Forward
    analogWrite(LEFT_FOR, turnSpeed);   // Reverse
}
/*
 * This function handles the logic for making a turn to the right.
 * It will gradually turn based on the twist_msg.angular.z value.
 */
void rightTurn(const geometry_msgs::Twist& twist_msg) {
   
    float turn = (twist_msg.linear.x - twist_msg.angular.z * 2);
    float turnSpeed = turn < 0 ? 0 : turn; 
	
    // Calculate the speed of the tracks 
    // Left track
    leftBackSpeed = LOW;
    float speed = (turnSpeed + (twist_msg.linear.x - turnSpeed) * 0.5) * distanceSpeed;
    leftForSpeed =  turnSpeed + (twist_msg.linear.x - turnSpeed) * 0.5;
    analogWrite(LEFT_BACK, LOW);   // Reverse
    analogWrite(LEFT_FOR, speed);   // Forward}

    // Right track
    rightForSpeed = turnSpeed;
    rightBackSpeed = LOW;
    analogWrite(RIGHT_BACK, LOW);   // Forward
    analogWrite(RIGHT_FOR, turnSpeed);   // Reverse
}

void messageCb( const geometry_msgs::Twist& twist_msg){
    digitalWrite(24, HIGH);
    digitalWrite(25, HIGH);

    distanceSpeed = distanceCheck();
    lastSpeed = millis();
	
    // Make a left turn
    if(twist_msg.angular.z < -10) {
        leftTurn(twist_msg);
		
    // Make a right turn
    } else if(twist_msg.angular.z > 10) {
        rightTurn(twist_msg);
		
    // Don't make any turn.
    } else {
	// Calculate the speed based on the distance to an object.
        float speed = twist_msg.linear.x * distanceSpeed;
		
	// Go Forward
        if(twist_msg.linear.x > 0) {
            rightBackSpeed = LOW;
            rightForSpeed = twist_msg.linear.x;
            analogWrite(RIGHT_BACK, LOW);   // Reverse
            analogWrite(RIGHT_FOR, speed );   // Forward

            leftBackSpeed = LOW;
            leftForSpeed = twist_msg.linear.x;
            analogWrite(LEFT_BACK, LOW);   // Reverse
            analogWrite(LEFT_FOR, speed);   // Forward
		
	// Go Backwards
        }  else {
            rightBackSpeed = -1 * twist_msg.linear.x;
            rightForSpeed = LOW;
            analogWrite(RIGHT_FOR, LOW);   // Forward
            analogWrite(RIGHT_BACK, -1 * speed);   // Reverse

            leftBackSpeed = -1 * twist_msg.linear.x;
            leftForSpeed = LOW;
            analogWrite(LEFT_FOR, LOW);   // Forward
            analogWrite(LEFT_BACK, -1 * speed);   // Reverse
        }
    }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

class NewHardware : public ArduinoHardware {
  public : NewHardware() : ArduinoHardware(&Serial1, 57600){};
}; ros::NodeHandle_<NewHardware> nh;


void setup() {
	// Set-up all pins and ROS.
    Serial.begin(9600);
    pinMode(13, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(22, INPUT);
    pinMode(7, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(24, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    updateSpeeds();
    nh.spinOnce();
    delay(1);
}

