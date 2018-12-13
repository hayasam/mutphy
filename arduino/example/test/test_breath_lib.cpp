#include <Arduino.h>
#include <unity.h>
#include "../src/breath_lib.h"

#ifdef UNIT_TEST

const int buttonPin = 2;
const int ledPin = 9; 

BreathLED breathLed(buttonPin,ledPin);

void test_pin_number(void){
		TEST_ASSERT_EQUAL(breathLed.getLedPin(),9);
		TEST_ASSERT_EQUAL(breathLed.getButtonPin(),2);
}

void test_led_on(void){
		breathLed.on();
		delay(30);
		TEST_ASSERT_EQUAL(digitalRead(ledPin), HIGH);
}

void test_led_off(void){
		breathLed.off();
		delay(30);
		TEST_ASSERT_EQUAL(digitalRead(ledPin), LOW);
}

void test_click_button(void){
		Serial.println("press button, please!");
		int now=millis();
		boolean isClicked = breathLed.is_button_clicked();
		while((isClicked!=true) && ((millis()-now)<=3000)){
				isClicked = breathLed.is_button_clicked();
		}
		TEST_ASSERT_TRUE(isClicked);
		//second click
		Serial.println("press button, please!");
		now=millis();
		isClicked = breathLed.is_button_clicked();
		while((isClicked!=true) && ((millis()-now)<=3000)){
				isClicked = breathLed.is_button_clicked();
		}
		TEST_ASSERT_TRUE(isClicked);
}

void test_led_breath(void){
  int current = millis();
  breathLed.start_breath();
  int duration = millis()-current;
  TEST_ASSERT_TRUE(duration>=3060);
  breathLed.stop_breath();
  TEST_ASSERT_EQUAL(digitalRead(ledPin),LOW);
}

void setup(){
		UNITY_BEGIN();
		RUN_TEST(test_pin_number);
		Serial.begin(9600);
}

void loop(){
		RUN_TEST(test_led_on);
		delay(500);
		RUN_TEST(test_led_off);
		delay(500);
		RUN_TEST(test_click_button);
		delay(500);
		RUN_TEST(test_led_breath);
		delay(500);
		UNITY_END(); // stop unit testing
}

#endif
