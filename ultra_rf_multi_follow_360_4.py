import RPi.GPIO as GPIO
import time
from readchar import readchar
from gpiozero import Robot
import numpy as np

#GPIO.cleanup()

GPIO.setmode(GPIO.BCM)                     #Set GPIO pin numbering 

bit_a = 22
bit_b = 27
bit_c = 17
rf_pin = 15

GPIO.setup(bit_a, GPIO.OUT)
GPIO.setup(bit_b, GPIO.OUT)
GPIO.setup(bit_c, GPIO.OUT)
GPIO.setup(rf_pin,GPIO.OUT)

leftPins_x = (12, 16)
rightPins_x = (20, 21)

leftPins_y = (26, 19)
rightPins_y = (13, 6)

robot_x = Robot(left = leftPins_x, right = rightPins_x)
robot_y = Robot(left = leftPins_y, right = rightPins_y)


def robot_con(xl, xr, yl, yr):
    robot_x.value = (xl, xr)
    robot_y.value = (yl, yr)

def robot_con2(x, y):
    robot_x.value = (x, x)
    robot_y.value = (y, y)

def multiplex(code):

    if len(code) == 3:
        
        GPIO.output(bit_a, int(code[2]))
        GPIO.output(bit_b, int(code[1]))
        GPIO.output(bit_c, int(code[0]))
        time.sleep(0.00001)

    else:
        print("multiplex fail")

def distancesen(echo_pin, trig_pin):

    GPIO.setup(trig_pin,GPIO.OUT)                  #Set pin as GPIO out
    GPIO.setup(echo_pin,GPIO.IN)                   #Set pin as GPIO in
  
    GPIO.output(trig_pin, False)                 #Set TRIG as LOW      
    GPIO.output(rf_pin, False)                  #rf is active low
    time.sleep(0.05)

    GPIO.output(trig_pin, True)                  #Set TRIG as HIGH
    time.sleep(0.00001)                      #Delay of 0.00001 seconds
    GPIO.output(trig_pin, False)                 #Set TRIG as LOW
    
    pulse_start = time.time()
    
    while GPIO.input(echo_pin)==0:               #Check whether the ECHO is LOW
        pulse_start = time.time()              #Saves the last known time of LOW pulse
        
    GPIO.output(rf_pin, True)

    pulse_end = time.time()
    
    while GPIO.input(echo_pin)==1:               #Check whether the ECHO is HIGH
        pulse_end = time.time()                #Saves the last known time of HIGH pulse

    pulse_duration = pulse_end - pulse_start #Get pulse duration to avariable
    distance = (pulse_duration-0.05) * 34030    #Multiply pulse duration by 17150 to get distance

    time.sleep(0.2)
    
    return round(distance, 2)            #Round to two decimal points

def distancesen_all(echo_pin, trig_pin):

    out_of_range = 1000

    GPIO.setup(trig_pin,GPIO.OUT)                  #Set pin as GPIO out
    GPIO.setup(echo_pin,GPIO.IN)                   #Set pin as GPIO in

    dist_list = [0] * 8
  
    GPIO.output(trig_pin, False)                 #Set TRIG as LOW      
    GPIO.output(rf_pin, False)                  #rf is active low
    time.sleep(0.05)
    GPIO.output(rf_pin, True)
    
    GPIO.output(trig_pin, True)                  #Set TRIG as HIGH
    time.sleep(0.00001)                      #Delay of 0.00001 seconds
    GPIO.output(trig_pin, False)                 #Set TRIG as LOW

    
    pulse_start = time.time()

    time.sleep(0.01)

    while time.time() < pulse_start + 0.5:
        for i in range(0,len(dist_list)):
            #print(i)
            ultra_2(i+1)
            if GPIO.input(echo_pin)==1:
                dist_list[i] = time.time()

    for i in range(0,len(dist_list)):
        if (dist_list[i]-pulse_start) > 0.1:
            dist_list[i] = out_of_range
        else:
            dist_list[i] = (dist_list[i]-pulse_start - 0.05) *34030

    #return dist_list

    dist = min(dist_list)

    if dist == out_of_range:
        module = None
        dist = None
        
        return(module, dist)
    
    else:
        module = dist_list.index(dist)+1
        return(module, dist)


def multi_ultra(string, echo_pin, trig_pin):
    multiplex(string)
    return distancesen(echo_pin, trig_pin)

def ultra(element):
    if element == 1:
        return multi_ultra('000', 23, 24)
    elif element == 2:
        return multi_ultra('001', 23, 24)
    elif element == 3:
        return multi_ultra('010', 23, 24)
    elif element == 4:
        return multi_ultra('011', 23, 24)
    elif element == 5:
        return multi_ultra('100', 23, 24)
    elif element == 6:
        return multi_ultra('101', 23, 24)
    elif element == 7:
        return multi_ultra('110', 23, 24)
    elif element == 8:
        return multi_ultra('111', 23, 24)
    else:
        print("ultra fail")
        return None

def ultra_2(element):
    if element == 1:
        return multiplex('000')
    elif element == 2:
        return multiplex('001')
    elif element == 3:
        return multiplex('010')
    elif element == 4:
        return multiplex('011')
    elif element == 5:
        return multiplex('100')
    elif element == 6:
        return multiplex('101')
    elif element == 7:
        return multiplex('110')
    elif element == 8:
        return multiplex('111')
    else:
        print("ultra fail")
        return None


def valid_dist(value):

    #print("check valid")
    attempts = 5
    out_of_range2 = 600

    for i in range(1,attempts +1):
        dist = ultra(value)
        #print(dist)
        if (0 < dist < out_of_range2):
            #i = attempts+10
            break
        else:
            #dist = None
            dist = 10000
            
    return dist

def adj(module):
    if module == 1:
        list_module=[8,1,2]
    elif module == 2:
        list_module=[1,2,3]
    elif module == 3:
        list_module=[2,3,4]
    elif module == 4:
        list_module=[3,4,5]
    elif module == 5:
        list_module=[4,5,6]
    elif module == 6:
        list_module=[5,6,7]
    elif module == 7:
        list_module=[6,7,8]
    elif module == 8:
        list_module=[7,8,1]
    else:
        return None

    print(list_module)
    
    return list_module

def smart_al(module):
    #scan module

    #scan  adjacent
    adj_list = adj(module)
    #print(adj_list)
    dist_list = [0] * 3
    #print dist
    min_module = 5
    
    for i in range (0, len(adj_list)):
        #print(i, adj_list[i])
        dist_list[i] = valid_dist(adj_list[i])

    #print(dist_list)
    dist = min(dist_list)
    module = adj_list[dist_list.index(dist)]
        
    #if timeout, scan module 1

    return(module, dist)

def get_cartesian(module, dist):
    #get angle
    angle = (90 + (module - 1)*45) % 360
    
    x = dist * np.cos(np.radians(angle))
    y = dist * np.sin(np.radians(angle))

    return(x,y)

def module_forward(module):

    speed = 0.4

    print(module)
    
    if module == 1:
        return (speed, speed, 0, 0)
    elif module == 2:
        return (speed, speed, -speed, -speed)
    elif module == 3:
        return (0, 0, -speed, -speed)
    elif module == 4:
        return (-speed, -speed, -speed, -speed)
    elif module == 5:
        return (-speed, -speed, 0, 0)
    elif module == 6:
        return (-speed, -speed, speed, speed)
    elif module == 7:
        return (0, 0, speed, speed)
    elif module == 8:
        return (speed, speed, speed, speed)
    else:
        pass

multiplex('000')

print("***Starting***")


multiplex('000')

module = 1

while True:

    lower_lim = 100
    upper_lim = 200
    out_of_range = 400

    module,dist = distancesen_all(23,24)

    print (module,dist)

    if dist != None:
        if upper_lim < dist < out_of_range:
            #forward
            print("forward")
            command = module_forward(module)
            print(command)
            robot_con(command[0],command[1],command[2],command[3])
            #time.sleep(1)
            #robot_con(0, 0, 0, 0)
        elif dist < lower_lim:
            #pass
            #backward
            print("backward")
            command = module_forward(module)
            print(command)
            robot_con(-command[0],-command[1],-command[2],-command[3])
            #time.sleep(1)
            #robot_con(0, 0, 0, 0)
        else:
            #stop
            print("stop")
            robot_con(0, 0, 0, 0)
    else:
        print("stop, no distance")
        #module =  prev_module
        #print("after: ", module)
        robot_con(0, 0, 0, 0)

    #apply to each half of robot
