import RPi.GPIO as GPIO
import time

# motors
plift0 = 20
plift1 = 21
phand = 2
prot0 = 12  # anti-clockwise
prot1 = 16  # clockwise

# sensor
pheight = 6
protate = 5
pvertical = 11

# rotator
rotspeed = 50 
ninety_timeout = 4.6*(rotspeed/50)

class Arm:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(plift0,GPIO.OUT,initial=1)
        GPIO.setup(plift1,GPIO.OUT,initial=1)
        GPIO.setup(phand,GPIO.OUT,initial=0)
        GPIO.setup(prot0,GPIO.OUT,initial=0)
        GPIO.setup(prot1,GPIO.OUT,initial=0)
        GPIO.setup(pheight,GPIO.IN,pull_up_down=GPIO.PUD_UP)
        GPIO.setup(protate,GPIO.IN,pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pvertical,GPIO.IN,pull_up_down=GPIO.PUD_UP)

    def lift(self,direction, at):
        '''
        direction: 1-up, 0-down
        at: action time, 4cm per second
        '''
        GPIO.output(plift0, direction)
        GPIO.output(plift1, 1-direction)
        time.sleep(at)
        GPIO.output(plift0, 1)
        GPIO.output(plift1, 1)
        return direction

    def grab(self,direction):
        '''
        direction: range-[4.8-7.8], 4.8-tight, 7.8-loose
        '''
        pwm=GPIO.PWM(phand,50)
        pwm.start(direction)
        print('direction no: '+str(direction))
        pwm.stop()
        return direction

    def get_rotators(self):
        pwms=[]
        pwm0=GPIO.PWM(prot0,100)
        pwm1=GPIO.PWM(prot1,100)
        pwms.append(pwm0)
        pwms.append(pwm1)
        return pwms

    def rotate(self,direction,speed,rot_time):
        '''
        direction: 0-anti-clockwise,1-clockwise
        '''
        pwms=self.get_rotators()
        pwms[direction].start(speed)
        time.sleep(rot_time)
        #input()
        pwms[direction].stop()
     
    def flipover(self):
        if(GPIO.input(pvertical)==0):
            self.rotate2pos1()
            time.sleep(0.2)
            self.rotate2pos1()
        else:
            self.rotate2pos0()
            time.sleep(0.2)
            self.rotate2pos0()

    def rotate2pos0(self):
        '''
        the rotator must be reset before calling this function
        posid: 0-A face up;clockwise, 1-A face down;anti-clockwisw
        '''
        pwms=self.get_rotators()
        timeout=ninety_timeout
        start=time.time()
        pwms[1].start(rotspeed)
        time.sleep(0.8)
        detected_sensor=pvertical
        while((GPIO.input(detected_sensor)==1) \
              and (time.time()-start<=timeout)):
            pass
            time.sleep(0.01)
        if(time.time()-start>timeout):
            print('somthing went wrong...')
        else:
            print('reached POS 0!')
        pwms[1].stop()
        #return time.time()-start
        
    def rotate2pos1(self):
        '''
        the rotator must be reset before calling this function
        posid: 0-A face up;clockwise, 1-A face down;anti-clockwisw
        '''
        pwms=self.get_rotators()
        timeout=ninety_timeout
        start=time.time()
        pwms[0].start(rotspeed)
        time.sleep(0.8)
        detected_sensor=protate
        while((GPIO.input(detected_sensor)==1) \
              and (time.time()-start<=timeout)):
            pass
            time.sleep(0.01)
        if(time.time()-start>timeout):
            print('somthing went wrong...')
        else:
            print('reached POS 1!')
        pwms[0].stop()
        #return time.time()-start
        
    def resetrotate(self):
        pwms=self.get_rotators()
        # first rotate anti-clockwise for 1/4 pi, then clockwise for anthor 1/2 pi
        timeout = 1/2 * ninety_timeout
        pos = -1
        print('first rotate anti-clockwise')
        start0 = time.time()
        pwms[1].start(rotspeed)
        while((time.time()-start0)<=timeout):
            time.sleep(0.01)
            if(GPIO.input(pvertical)==0):
                print('reached POS 0!')
                pos=0
                break
            if(GPIO.input(protate)==0):
                print('reached POS 1!')
                pos=1
                break
        pwms[1].stop()
        time.sleep(0.2)
        if(pos<0):
            print('then rotate clockwise')
        start1 = time.time()
        pwms[0].start(rotspeed)
        while((pos<0) and ((time.time()-start1)<=2*timeout)):
            time.sleep(0.01)
            if(GPIO.input(pvertical)==0):
                print('reached POS 0!')
                pos=0
                break
            if(GPIO.input(protate)==0):
                print('reached POS 1!')
                pos=1
                break
        pwms[0].stop()
        time.sleep(0.2)
        if(pos >= 0):
            self.rotate2reset(pos)
        #print('reached a predefined pos!')

    def rotate2reset(self,direction):
        '''
        direction: 0-anti-clockwise, 1-clockwise
        '''
        pwms=self.get_rotators()
        timeout=ninety_timeout
        start=time.time()
        pwms[direction].start(rotspeed)
        while((not((GPIO.input(pvertical)==0)\
                   and (GPIO.input(protate)==0)))\
              and (time.time()-start<=timeout)):
            pass
            time.sleep(0.01)
        if(time.time()-start>timeout):
            print('something went wrong...')
        else:
            print('reached POS RET!')
        pwms[direction].stop()

    def resetheight(self):   
        # lift direction: move down to the default height
        # down
        timeout = 3
        start = time.time()
        while ((GPIO.input(pheight)==1)\
               and (time.time()-start<=timeout)):
            GPIO.output(plift0,0)
            GPIO.output(plift1,1)
        if(time.time()-start>timeout):
            print('something went wrong...')
        else:
            print('reached the default height!')
        GPIO.output(plift0, 1)
        GPIO.output(plift1, 1)
        
    def read_current_pheight(self):
        return GPIO.input(pheight)

    def cleanup(self):
        GPIO.cleanup()

