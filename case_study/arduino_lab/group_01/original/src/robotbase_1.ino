/**
 * Group number: 1
 * Student 1:
 * Stefan Breetveld, 4374657
 * Student 2:
 * David Viteri, 4580958
 */

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//Node Laptop-Robot class & Serial config
class NewHardware : public ArduinoHardware {
  public: NewHardware() : ArduinoHardware (&Serial1 ,57600) { } ;
}; ros::NodeHandle_<NewHardware > nh;

// Define used pins.
int servo_left = 6; int servo_right = 2; int trig_US = 23; int echo_US = 22;
// Declare safety variables.
int safe_distance, com_ok = 1;
unsigned long duration, distance; //us & cm
int flag_US, counter = 0;
float left_speed, right_speed;

// Update information from the ultrasonic sensor
void update_US(){
	digitalWrite(trig_US, LOW);
	delayMicroseconds(2);
	digitalWrite(trig_US, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig_US, LOW);
	duration = pulseIn(echo_US, HIGH);

	distance = (duration*17)/1000;
	if (distance > 25)
		safe_distance = 1;
	else
		safe_distance = 0;
}

// Convert twist message to
void update_servos(const geometry_msgs::Twist& message){
	TCNT2 = 0x00;
	counter = 0;
    com_ok = 1;
    right_speed = message.linear.x;
	left_speed = message.linear.y;

}

// write the appropriate signal to the engines
void signal_servos() {
	if(safe_distance & com_ok){
		analogWrite(servo_right, right_speed);
		analogWrite(servo_left, left_speed);
	}
	else{
		analogWrite(servo_right, 0);
		analogWrite(servo_left, 0);
	}
}

//Twist geometry_msgs subscription
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &update_servos );
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// Initialize pins on the arduino
void setup()
{
  Serial.begin(9600);
	pinMode(7, OUTPUT);
	pinMode(24, OUTPUT);
	pinMode(servo_left, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(25, OUTPUT);
	pinMode(servo_right, OUTPUT);
	pinMode(trig_US, OUTPUT);
	pinMode(echo_US, INPUT);
	pinMode(13, OUTPUT);

	digitalWrite(24, HIGH);
	digitalWrite(25, HIGH);
	digitalWrite(6, LOW);

	nh.initNode();
	nh.subscribe(sub);
        nh.advertise(chatter);
	noInterrupts();

	//timer 2 config
	TCCR2A = 0;
	TCCR2B = 0;
	TCCR2A |= (1 << WGM21);
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);//fclk/1024
	OCR2A = 0xff;//Registro del Timer: 0-255
	TIMSK2 |= (1 << OCIE2A);

	interrupts();
}

// Core arduino loop. called continously.
void loop()
{
    Serial.println(counter);
	if(flag_US){//every 16.38ms
		update_US();
		flag_US=0;
	}
    signal_servos();
	//update_servos: automatically
    nh.spinOnce();

}

// Timer interrupt
ISR (TIMER2_COMPA_vect)  // timer2 overflow interrupt
{
    if(counter%3) {//every 49ms
          flag_US = 1;
    }

	if(counter >= 30)//.5 secs
		com_ok=0;

        counter++;

}
