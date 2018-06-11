import subprocess,os,signal
import serial,io,time
import pytest
import string
from operator import add

#-----------------------------
# test helper functions
#-----------------------------
def send_image(image_path):
    # source ros file
    output = subprocess.check_output('source /home/qianqianzhu/image_transport_ws/devel/setup.bash;env -0'
		                             ,shell=True, executable="/bin/bash")
    # replace env
    #os.environ.clear() 
    lines = output.decode().split('\0')
    for line in lines:
        (key, _, value) = line.partition("=")
        os.environ[key] = value

	#image = '/home/qianqianzhu/Pictures/straight.jpg'
	#image = '/home/qianqianzhu/Pictures/left.jpg'
	#image = '/home/qianqianzhu/Pictures/right.jpeg'
    ros_command = 'rosrun image_transport_tutorial my_publisher '+ image_path
    pro = subprocess.Popen(ros_command,shell=True,preexec_fn=os.setsid)

	# record pwm 
    subprocess.call('rm -f speed_log.txt',shell=True)
    device   = '/dev/ttyACM1' # serial port
    baud     = 57600                          # baud rate
    filename = 'speed_log.txt'                # log file to save data in
	 
    with serial.Serial(device,baud) as serialPort, open(filename,'wb') as outFile:
        print("start recording serial...")
        timer = time.time()
        while(time.time()-timer<=10):
            line = serialPort.readline() # must send \n! get a line of log
		    #print (line)                 # show line on screen
            outFile.write(line)          # write line of text to file
            outFile.flush()              # make sure it actually gets written out
        print("stop recording serial...")

	# stop sending an image
    os.killpg(os.getpgid(pro.pid), signal.SIGTERM)

	# read speed from pwm log
    f = open(filename,"rb")

    total = [0,0,0,0]
    #print("hello")

    for line in f:
        if len(line) == 9:
            line = line.decode('utf-8')
            columns = line.strip().split(',')
            if len(columns) == 4:
                delta = list(map(int,columns))
                total = list(map(add,total,delta))
                #print(total)
    f.close()
    return total

def send_ultrasensor(signal):
    device   = '/dev/ttyACM1' # serial port
    baud     = 57600                          # baud rate
    with serial.Serial(device,baud,timeout=5) as ard:
        time.sleep(0.5) # wait for Arduino
        # Serial write section
        ard.flush()
        print ("send STOP signal!")
        ard.write(str.encode(signal))
        time.sleep(0.5)

#-----------------------------
# test cases
#-----------------------------
def test_straight():
    send_ultrasensor("go")
    image_path = '/home/qianqianzhu/Pictures/straight.jpg'
    speed = send_image(image_path)
    left_speed = speed[1]
    right_speed = speed[3]
    assert left_speed == pytest.approx(right_speed,rel=5e-2)

def test_turn_left():
    send_ultrasensor("go")
    image_path = '/home/qianqianzhu/Pictures/left.jpeg'
    speed = send_image(image_path)
    left_speed = speed[1]
    right_speed = speed[3]
    assert left_speed < right_speed

def test_turn_right():
    send_ultrasensor("go")
    image_path = '/home/qianqianzhu/Pictures/right.jpg'
    speed = send_image(image_path)
    left_speed = speed[1]
    right_speed = speed[3]
    assert left_speed > right_speed

def test_stop_obstacle():
    send_ultrasensor("stop")
    image_path = '/home/qianqianzhu/Pictures/straight.jpg'
    speed = send_image(image_path)
    left_speed = speed[1]
    right_speed = speed[3]
    assert left_speed == 0
    assert right_speed == 0

def test_stop_no_image():
    send_ultrasensor("go")
    image_path = ''
    speed = send_image(image_path)
    left_speed = speed[1]
    right_speed = speed[3]
    assert left_speed == 0
    assert right_speed == 0

    
