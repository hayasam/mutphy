import RPi.GPIO as GPIO
import pytest 
from chassis import *

#chassis = Chassis()

def test_gpio_setup():
    chassis = Chassis()
    for pin in chassis.get_stepper_pins():
        assert GPIO.gpio_function(pin) == GPIO.OUT
    for pin in chassis.get_sensor_pins():
        assert GPIO.gpio_function(pin) == GPIO.IN
    chassis.cleanup()

def test_move_direction(capsys):
    chassis = Chassis()
    # 0: forward
    chassis.move(0, 20, 20, 20, (lambda:True))
    out, err  = capsys.readouterr()
    assert out == '[1, 0, 1, 0, 0, 0, 0, 0]\n'
    time.sleep(0.5)
    # 1: backward
    chassis.move(1, 20, 20, 20, (lambda: True))
    out, err  = capsys.readouterr()
    assert out == '[0, 0, 0, 0, 1, 0, 1, 0]\n'
    time.sleep(0.5)
    # 2: strate left
    chassis.move(2, 20, 20, 20, (lambda: True)) 
    out, err  = capsys.readouterr()
    assert out == '[0, 0, 1, 0, 0, 0, 1, 0]\n'
    time.sleep(0.5)
    # 3: strate right
    chassis.move(3, 20, 20, 20, (lambda: True)) 
    out, err  = capsys.readouterr()
    assert out == '[1, 0, 0, 0, 1, 0, 0, 0]\n'
    time.sleep(0.5)
    # 4: left turn
    chassis.move(4, 0.02, 0.02, 0.02, (lambda: True)) 
    out, err  = capsys.readouterr()
    assert out == '[1, 0, 1, 0, 1, 0, 1, 0]\n'
    time.sleep(0.5)
    # 5: right turn
    time.sleep(0.5)
    chassis.move(5, 0.02, 0.02, 0.02, (lambda: True)) 
    out, err  = capsys.readouterr()
    assert out == '[0, 0, 0, 0, 0, 0, 0, 0]\n'
    chassis.cleanup()

def test_alignline():
    chassis = Chassis()
    #pytest.skip('cannot test now')
    # black to white
    chassis.alignline(1,0,10)
    time.sleep(0.5)
    chassis.move(0,20,20,20, (lambda: True))
    for state in chassis.read_current_front_light_sensor():
        assert state == 0
    time.sleep(0.5)
    chassis.move(1,50,20,20, (lambda: True))
    for state in chassis.read_current_front_light_sensor():
        assert state == 1
    #time.sleep(0.5)
    #chassis.move(0,20,20,20, (lambda: True))
    chassis.cleanup()  
    
def test_alignmiddle():
    chassis = Chassis()
    chassis.alignmiddle()
    time.sleep(0.5)
    chassis.move(2,20,20,20,(lambda:True))
    assert chassis.read_current_middle_light_sensor()==0
    time.sleep(0.5)
    chassis.move(3,40,20,20,(lambda:True))
    assert chassis.read_current_middle_light_sensor()==1
    #time.sleep(0.5)
    #chassis.move(2,20,20,20,(lambda:True))
    chassis.cleanup()
