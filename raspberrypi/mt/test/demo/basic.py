from RPi import GPIO
from time import sleep

pbutton = 2
pled = 4

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pled,GPIO.OUT,initial=(GPIO.LOW))
    GPIO.setup(pbutton,GPIO.IN)

def on():
    GPIO.output(pled,1)

def off():
    GPIO.output(pled,0)

def on_with_brightness(dc):  # from 1 to 100
    led = GPIO.PWM(pled,60)
    led.start(dc)
    sleep(0.1)
    return dc

def cleanup():
    GPIO.cleanup()

def start_breathe():
    led = GPIO.PWM(pled,60)
    led.start(0)
    print('start breathing')
    for dc in range(0,101,5):
        led.ChangeDutyCycle(dc)
        sleep(0.1)
    for dc in range(100,-1,-5):
        led.ChangeDutyCycle(dc)
        sleep(0.1)
    print('finish one loop')

def stop_breathe():
    led = GPIO.PWM(pled,60)
    led.stop()
    print('stop breathing')

