from basic import *
from time import time
from RPi import GPIO
from pytest import approx,raises

def test_setup_cleanup():
    setup()
    assert GPIO.gpio_function(4)==GPIO.OUT
    assert GPIO.gpio_function(2)==GPIO.IN
    cleanup()
    with raises(RuntimeError) as excinfo:
        assert GPIO.gpio_function(4)==GPIO.UNKNOWN
    assert 'Please set pin numbering mode using GPIO.setmode(GPIO.BOARD) or GPIO.setmode(GPIO.BCM)' == str(excinfo.value)

def test_led():
    setup()
    on()
    assert GPIO.input(4)==1
    off()
    assert GPIO.input(4)==0
    cleanup()

def test_led_brightness():
    setup()
    assert 20 == on_with_brightness(20)
    cleanup()

def test_button():
   setup()
   initial_state = GPIO.input(2)
   #mock_click_button()
   print('please press the button!')
   GPIO.wait_for_edge(2,GPIO.BOTH)
   current_state = GPIO.input(2)
   assert current_state != initial_state
   cleanup()

def test_breathe(capsys):
    setup()
    current = time()
    start_breathe()
    end = time()
    assert (end-current) == approx(4,rel=1e-1)
    out, err  = capsys.readouterr()
    assert out == 'start breathing\nfinish one loop\n'
    stop_breathe()
    out, err = capsys.readouterr()
    assert out == 'stop breathing\n'
    cleanup()
         

