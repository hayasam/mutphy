import RPi.GPIO as GPIO
import pytest
from arm import *
import time

rotspeed=50
ninety_timeout=4.5*rotspeed/50

def test_lift():
    #pytest.skip('test_lift')
    # 1-up
    arm = Arm()
    time.sleep(0.5)
    start=time.time()
    dir = arm.lift(1,1)
    duration=time.time()-start
    assert duration == pytest.approx(1,rel=1e-1)
    assert dir==1
    # 0-down
    time.sleep(0.5)
    dir = arm.lift(0,1)
    assert dir==0
    time.sleep(0.1)
    arm.cleanup()

def test_grab():
    #pytest.skip('test_grab')
    arm = Arm()
    time.sleep(0.5)
    dir=arm.grab(5)
    assert dir==5
    time.sleep(0.1)
    arm.cleanup()

def test_resetheight():
    #pytest.skip('pass')
    arm = Arm()
    time.sleep(0.5)
    arm.lift(1,1)
    time.sleep(0.5)
    arm.resetheight()
    #arm.lift(0,0.1)
    #assert arm.read_current_pheight() == 0
    time.sleep(0.5)
    arm.lift(1,1)
    assert arm.read_current_pheight() == 1
    time.sleep(0.1)
    arm.cleanup()

def test_resetrotate_at_1(capsys):
    ##pytest.skip('test_resetrotate_at_1')
    arm = Arm()
    arm.rotate2pos1()
    out,err = capsys.readouterr()
    assert out == 'reached POS 1!\n'
    time.sleep(1)
    arm.resetrotate()
    out,err = capsys.readouterr()
    assert out == 'first rotate anti-clockwise\nreached POS 1!\nreached POS RET!\n'
    time.sleep(0.1)
    arm.cleanup()

def test_resetrotate_at_0(capsys):
    #pytest.skip('test_resetrotate_at_0')
    arm = Arm()
    arm.rotate2pos0()
    out,err = capsys.readouterr()
    assert out == 'reached POS 0!\n'
    time.sleep(1)
    arm.resetrotate()
    out,err = capsys.readouterr()
    assert out == 'first rotate anti-clockwise\nreached POS 0!\nreached POS RET!\n'
    time.sleep(0.1)
    arm.cleanup()
    
def test_resetrotate_between_ret_1_case1(capsys):
    #pytest.skip('pass')
    arm = Arm()
    arm.resetrotate()
    time.sleep(0.5)
    # case 1: close to 1
    arm.rotate(0,rotspeed,ninety_timeout*0.8)
    out,err = capsys.readouterr()
    time.sleep(0.5)
    arm.resetrotate()
    out,err = capsys.readouterr()
    assert out == 'first rotate anti-clockwise\nthen rotate clockwise\nreached POS 1!\nreached POS RET!\n'
    time.sleep(0.1)
    arm.cleanup()

def test_resetrotate_between_ret_1_case2(capsys):
    #pytest.skip('pass')
    # case 2: close to RET
    arm = Arm()
    time.sleep(0.5)
    arm.rotate(0,rotspeed,ninety_timeout*0.1)
    time.sleep(0.5)
    out,err = capsys.readouterr()
    arm.resetrotate()
    out,err = capsys.readouterr()
    assert out == 'first rotate anti-clockwise\nreached POS 1!\nreached POS RET!\n'
    time.sleep(0.1)
    arm.cleanup()

def test_resetrotate_between_0_ret_case1(capsys):
    #pytest.skip('pass')
    # case 1: close to RET
    arm = Arm()
    time.sleep(0.5)
    arm.rotate(1,rotspeed,ninety_timeout*0.2)
    out,err=capsys.readouterr()
    time.sleep(0.5)
    arm.resetrotate()
    out,err=capsys.readouterr()
    assert out == 'first rotate anti-clockwise\nthen rotate clockwise\nreached POS 0!\nreached POS RET!\n'
    time.sleep(0.1)
    arm.cleanup()

def test_resetrotate_between_0_ret_case2(capsys):
    #pytest.skip('pass')
    # case 2: close to 0
    arm = Arm()
    time.sleep(0.5)
    arm.rotate(1,rotspeed,ninety_timeout*0.8)
    out,err = capsys.readouterr()
    time.sleep(0.5)
    arm.resetrotate()
    out,err=capsys.readouterr()
    assert out == 'first rotate anti-clockwise\nreached POS 0!\nreached POS RET!\n'
    time.sleep(0.1)
    arm.cleanup()

def test_resetrotate_at_ret(capsys):
    #pytest.skip('pass')
    arm=Arm()
    time.sleep(0.5)
    arm.resetrotate()
    out,err = capsys.readouterr()
    time.sleep(0.5)
    start = time.time()
    arm.resetrotate()
    duration = time.time()-start
    assert duration < 1
    out,err = capsys.readouterr()
    assert 'then rotate clockwise' not in out
    arm.cleanup()

def test_flipover(capsys):
    #pytest.skip('pass')
    arm = Arm()
    time.sleep(0.5)
    out,err=capsys.readouterr()
    arm.rotate2pos0()
    out,err=capsys.readouterr()
    assert out == 'reached POS 0!\n'
    time.sleep(0.5)
    arm.flipover()
    out,err=capsys.readouterr()
    assert out =='reached POS 1!\nreached POS 1!\n'
    time.sleep(0.5)
    arm.flipover()
    out,err=capsys.readouterr()
    assert out == 'reached POS 0!\nreached POS 0!\n'
    time.sleep(0.5)
    arm.resetrotate()
    arm.cleanup()

