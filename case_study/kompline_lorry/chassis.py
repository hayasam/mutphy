import RPi.GPIO as GPIO
import time
import math 
import sys

class Chassis:
    # actions:
    # 0 forward
    # 1 backward
    # 2 strafe left
    # 3 strafe right
    # 4 left turn
    # 5 right turn
    
    # Steppers 
    _lfdir=14
    _lfpul=15
    _lfact=[1,0,0,1,1,0]

    _ltdir=18
    _ltpul=23
    _ltact=[1,0,1,0,1,0]

    _rfdir=24
    _rfpul=25
    _rfact=[0,1,0,1,1,0]

    _rtdir=8
    _rtpul=7
    _rtact=[0,1,1,0,1,0]

    # Sensors
    _lfbw=19
    _rfbw=26
    _rmbw=13

    _srate = 1200/314.159
    _turnrate = 10838
    _stepperunit = [_srate,_srate,_srate+0.1,_srate+0.1,_turnrate,_turnrate] 

    _sigint=2.5

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Chassis._ltdir,GPIO.OUT,initial=0)
        GPIO.setup(Chassis._rtdir,GPIO.OUT,initial=0)
        GPIO.setup(Chassis._lfdir,GPIO.OUT,initial=0)
        GPIO.setup(Chassis._rfdir,GPIO.OUT,initial=0)
        GPIO.setup(Chassis._ltpul,GPIO.OUT,initial=0)
        GPIO.setup(Chassis._rtpul,GPIO.OUT,initial=0)
        GPIO.setup(Chassis._lfpul,GPIO.OUT,initial=0)
        GPIO.setup(Chassis._rfpul,GPIO.OUT,initial=0)
        GPIO.setup(Chassis._lfbw,GPIO.IN)
        GPIO.setup(Chassis._rfbw,GPIO.IN)
        GPIO.setup(Chassis._rmbw,GPIO.IN)

    def move(self, act, lenu, v0u, v1u, cond):
        '''
        act: action id
        lenu: length (0-3: mm & 4-5: round)
        v0u: initial speed unit/s
        v1u: final speed unit/s
        cond: enable condition
        '''
        GPIO.output(Chassis._lfdir, Chassis._lfact[act])
        GPIO.output(Chassis._ltdir, Chassis._ltact[act])
        GPIO.output(Chassis._rfdir, Chassis._rfact[act])
        GPIO.output(Chassis._rtdir, Chassis._rtact[act])
        
        print(self.read_current_stepper_states())
    
        v0 = v0u * Chassis._stepperunit[act] 
        v1 = v1u * Chassis._stepperunit[act] 
        dv = v1 - v0 
        nstep = int(lenu * Chassis._stepperunit[act])

        i = 0
        while (cond() and i<nstep):
            v = v0 + 1 / (1 + math.exp(Chassis._sigint - i*Chassis._sigint*2/nstep)) * dv
            tau = 1 / v
            GPIO.output(Chassis._lfpul,1)
            GPIO.output(Chassis._rfpul,1)
            GPIO.output(Chassis._ltpul,1)
            GPIO.output(Chassis._rtpul,1)
            time.sleep(tau*0.5)
            GPIO.output(Chassis._rtpul,0)
            GPIO.output(Chassis._ltpul,0)
            GPIO.output(Chassis._rfpul,0)
            GPIO.output(Chassis._lfpul,0)
            time.sleep(tau*0.5)
            i = i + 1

    def alignline(self, initstat, initdir, errortol):
        '''
        initstat: initial region colour, 1-black, 0-white
        initdir: initial direction
        errortol: error tolerance (step)
        '''
        direct = initdir
        stat = initstat
        nstep = errortol
        stepdelay = 0.005

        while (nstep >= errortol):
            nstep = 0
            GPIO.output(Chassis._lfdir, Chassis._lfact[direct])
            GPIO.output(Chassis._ltdir, Chassis._ltact[direct])
            GPIO.output(Chassis._rfdir, Chassis._rfact[direct])
            GPIO.output(Chassis._rtdir, Chassis._rtact[direct])
            while(GPIO.input(Chassis._lfbw)==stat\
                  or GPIO.input(Chassis._rfbw)==stat):
                if GPIO.input(Chassis._lfbw)==stat:
                    GPIO.output(Chassis._lfpul,1)
                    GPIO.output(Chassis._ltpul,1)
                if GPIO.input(Chassis._rfbw)==stat:
                    GPIO.output(Chassis._rfpul,1)
                    GPIO.output(Chassis._rtpul,1)
                time.sleep(stepdelay)
                GPIO.output(Chassis._rtpul,0)
                GPIO.output(Chassis._ltpul,0)
                GPIO.output(Chassis._rfpul,0)
                GPIO.output(Chassis._lfpul,0)
                time.sleep(stepdelay)
                nstep = nstep + 1
            print(nstep)
            time.sleep(0.5)
            direct = 1 - direct
            stat = 1 - stat
        
        #move(0,1000, 50, 50, lscond) 
        #align()
        #time.sleep(1)
   
    def alignmiddle(self):
        '''
        read rmbw's state: 1-black(strate left-2), 0-white(strate right-3)
        '''
        initdir = [3,2]
        initstate = GPIO.input(Chassis._rmbw)
        self.move(initdir[initstate],300,20,20,(lambda:(GPIO.input(Chassis._rmbw)==initstate)))
        print('reached the middle line!')

    def mymove(self,direction,distance):
        if direction in range(4):
            self.move(direction,distance,50,50, (lambda: True))
        else:
            self.move(direction,0.05,0.01,0.01,(lambda:True))

    def myalignline(self):
        self.alignline(1,0,10)
        self.alignmiddle()
    
    def __str__(self):
        return str(GPIO.input(Chassis._lfbw))+' '+str(GPIO.input(Chassis._rfbw))
        
    def cleanup(self):
        GPIO.cleanup()

    def read_current_stepper_states(self):
        states = []
        for pin in self.get_stepper_pins():
            states.append(GPIO.input(pin))
        return states

    def read_current_front_light_sensor(self):
        states = []
        for pin in self.get_front_light_sensor_pins():
            states.append(GPIO.input(pin))
        return states
    def read_current_middle_light_sensor(self):
        return GPIO.input(Chassis._rmbw)
        
    def get_stepper_pins(self):
        pins = []
        pins.append(Chassis._lfdir)
        pins.append(Chassis._lfpul)
        pins.append(Chassis._ltdir)
        pins.append(Chassis._ltpul)
        pins.append(Chassis._rfdir)
        pins.append(Chassis._rfpul)
        pins.append(Chassis._rtdir)
        pins.append(Chassis._rtpul)
        return pins
   
    def get_front_light_sensor_pins(self):
        pins = []
        pins.append(Chassis._lfbw)
        pins.append(Chassis._rfbw)
        return pins

    def get_sensor_pins(self):
        pins = []
        pins.append(Chassis._lfbw)
        pins.append(Chassis._rfbw)
        pins.append(Chassis._rmbw)
        return pins

