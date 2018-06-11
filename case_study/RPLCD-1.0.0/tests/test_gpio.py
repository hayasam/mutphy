# -*- coding: utf-8 -*-
from __future__ import print_function, division, absolute_import, unicode_literals

import pytest,mock

import RPi.GPIO as GPIO
from RPLCD.gpio import CharLCD
from RPLCD.common import LCD_SETDDRAMADDR

def test_gpio_init_connection(mocker, charlcd_kwargs):
    """
    test gpio's init_connection
    """
    lcd = CharLCD(**charlcd_kwargs)
    #init_connection = mocker.patch.object(lcd, '_init_connection')
    output = mocker.patch("RPi.GPIO.output")
    lcd._init_connection()
    #assert init_connection.call_count == 1
    #assert GPIO.input(lcd.pins.rs)==0
    #assert GPIO.input(lcd.pins.e)==0
    #assert GPIO.input(lcd.pins.rw)==0
    assert output.call_count == 3
    output_calls = [c[0] for c in output.call_args_list]
    assert output_calls[0]==(lcd.pins.rs,0)
    assert output_calls[1]==(lcd.pins.e,0)
    assert output_calls[2]==(lcd.pins.rw,0)
    

def test_gpio_pulse(mocker, charlcd_kwargs):
    """
    test gpio's init_connection
    """
    lcd = CharLCD(**charlcd_kwargs)
    output = mocker.patch("RPi.GPIO.output")
    lcd._pulse_enable()
    assert output.call_count== 3
    output_calls = [c[0] for c in output.call_args_list]
    assert output_calls[0]==(lcd.pins.e,0)
    assert output_calls[1]==(lcd.pins.e,1)
    assert output_calls[2]==(lcd.pins.e,0)
    
def test_gpio_write4bits(mocker,charlcd_kwargs):
    """
    test gpio's write4bits
    """
    lcd = CharLCD(**charlcd_kwargs)
    output = mocker.patch("RPi.GPIO.output")
    lcd._write4bits(14)
    assert output.call_count== 7
    output_calls = [c[0] for c in output.call_args_list]
    assert output_calls[0]==(lcd.pins[7],0)
    assert output_calls[1]==(lcd.pins[8],1)
    assert output_calls[2]==(lcd.pins[9],1)
    assert output_calls[3]==(lcd.pins[10],1) 

def test_gpio_write8bits(mocker,charlcd_kwargs):
    """
    test gpio's write4bits
    """
    lcd = CharLCD(**charlcd_kwargs)
    output = mocker.patch("RPi.GPIO.output")
    lcd._write8bits(15)
    assert output.call_count== 11
    output_calls = [c[0] for c in output.call_args_list]
    assert output_calls[0]==(lcd.pins[3],1)
    assert output_calls[1]==(lcd.pins[4],1)
    assert output_calls[2]==(lcd.pins[5],1)
    assert output_calls[3]==(lcd.pins[6],1) 
    assert output_calls[4]==(lcd.pins[7],0)
    assert output_calls[5]==(lcd.pins[8],0)
    assert output_calls[6]==(lcd.pins[9],0)
    assert output_calls[7]==(lcd.pins[10],0) 

def test_send1(mocker, charlcd_kwargs):
    """
    test gpio's send
    """
    lcd = CharLCD(**charlcd_kwargs)
    output = mocker.patch("RPi.GPIO.output")
    lcd._send(4,0)
    output_calls = [c[0] for c in output.call_args_list]
    #assert GPIO.input(lcd.pins.rs)==0
    #assert GPIO.input(lcd.pins.rw)==0
    assert output_calls[0]==(lcd.pins.rs,0)
    assert output_calls[1]==(lcd.pins.rw,0)


def test_send2(mocker, charlcd_kwargs):
    """
    test gpio's send
    """
    lcd = CharLCD(**charlcd_kwargs)
    output = mocker.patch("RPi.GPIO.output")
    lcd._send(4,1)
    output_calls = [c[0] for c in output.call_args_list]
    #assert GPIO.input(lcd.pins.rs)==1
    #assert GPIO.input(lcd.pins.rw)==0
    assert output_calls[0]==(lcd.pins.rs,1)
    assert output_calls[1]==(lcd.pins.rw,0)
