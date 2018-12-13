#include "breath_lib.h"

BreathLED::BreathLED(){
  ledState=LOW;
  lastButtonState=LOW;
  lastDebounceTime=0;
  deBounceDelay=50;
  pinMode(_pbutton, INPUT);
  pinMode(_pled, OUTPUT);
  digitalWrite(_pled, LOW);
  Serial.begin(9600);
}
int BreathLED::getButtonPin(){
  return _pbutton;
}

int BreathLED::getLedPin(){
  return _pled;
}

void BreathLED::on(){
  digitalWrite(_pled, HIGH);
}

void BreathLED::off(){
  digitalWrite(_pled, LOW);
}

int BreathLED::on_with_brightness(int brightness){
  if(brightness>=0 && brightness<=255){
    analogWrite(_pled, brightness);
  }else{
    Serial.println("please enter an integer in [0,255]!");
    brightness=-1;
  }
  return brightness;
}

boolean BreathLED::is_button_clicked(){
  boolean isClicked=false;
  int buttonState;
  int reading=digitalRead(_pbutton);
  if(reading!=lastButtonState){
    lastDebounceTime = millis();
  }
  delay(deBounceDelay);
  if(reading!=lastButtonState){
      buttonState=reading;
  }
  // only toggle the LED if the new button state is HIGH
  if (buttonState == HIGH) {
    ledState = !ledState;
    isClicked = true;
    Serial.println("button is clicked");
   }
   
  lastButtonState = reading;
  return isClicked;
}

void BreathLED::start_breath(){
  int brightness; 
  Serial.println("start breathing");
  // set the brightness of pin 9:
  for(brightness=0;brightness<=255;brightness = brightness+5){
    analogWrite(_pled, brightness);
    delay(30);
  }

  for(;brightness>=0;brightness = brightness-5){
    analogWrite(_pled, brightness);
    delay(30);
  }
  Serial.println("finish one loop");
}

void BreathLED::stop_breath(){
  digitalWrite(_pled, LOW);
  Serial.println("stop breathing");
}


