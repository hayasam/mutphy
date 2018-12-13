from pytest import *
from mt.scan import *

def test_get_indices():
    assert 0 == len(get_indices('a','bc'))
    assert 1 == len(get_indices('a','abc'))
    assert 2 == len(get_indices('a','abcab'))

def test_import():
    #module_name='example'
    #file_path='/Users/zhuqianqian/phd/physical_computing/mutation_testing/test/example.py'
    #file_path='/home/pi/mutation_testing/test/example.py'
    #module = import_file(module_name,file_path)
    from . import example
    #print(locals())
    assert example.pbutton == 2
    assert eval('example.pbutton')==2
    
def test_analyse_pin_arg():
    pled = 4
    pin_arg = 'pled'
    assert '4' == analyse_pin_arg(pin_arg,globals(),locals())

def test_parse_setup_stmt():
    setup_stmt1='GPIO.setup(1,GPIO.OUT)\n'
    args1 = parse_setup_stmt(setup_stmt1,0,globals(),locals())
    assert args1[0]=='1'
    assert args1[1]=='GPIO.OUT'
    assert args1[2]==''
    assert args1[3]=='GPIO.setup(PIN1,GPIO.OUT)\n'

    setup_stmt2='GPIO.setup(1,GPIO.OUT,initial=GPIO.HIGH)\n'
    args1 = parse_setup_stmt(setup_stmt2,0,globals(),locals())
    assert args1[0]=='1'
    assert args1[1]=='GPIO.OUT'
    assert args1[2]=='GPIO.HIGH'
    assert args1[3]=='GPIO.setup(PIN1,GPIO.OUT,initial=VALUE(GPIO.HIGH))\n'    

    setup_stmt3='GPIO.setup(pled,GPIO.OUT,initial=GPIO.HIGH)\n'
    #module_name='example'
    #file_path='/Users/zhuqianqian/phd/physical_computing/mutation_testing/test/example.py'
    file_path='/home/pi/mutation_testing/test/example.py'
    #module=import_file(module_name,file_path)
    from .demo import basic
    #print(locals())
    args1 = parse_setup_stmt(setup_stmt3,'basic',globals(),locals())
    assert args1[0]=='4'
