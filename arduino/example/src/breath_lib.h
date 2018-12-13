#ifndef bl
#define bl

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class BreathLED{
  public:
    BreathLED();
    void on();
    void off();
    int on_with_brightness(int brightness);
    void start_breath();
    void stop_breath();
    boolean is_button_clicked();
    int getButtonPin();
    int getLedPin();
  private:
    const int _pbutton=2;
    const int _pled=9;
    int ledState;
    int lastButtonState;
    unsigned long lastDebounceTime;
    unsigned long deBounceDelay; 
};

#endif
