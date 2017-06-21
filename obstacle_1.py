import RPi.GPIO as GPIO
import time
from gpiozero import Robot
from pynput.keyboard import Key, KeyCode, Listener

leftPins_x = (12, 16)
rightPins_x = (20, 21)

leftPins_y = (26, 19)
rightPins_y = (13, 6)

GPIO.setmode(GPIO.BCM)  

front_pin = 9
back_pin = 5
left_pin = 11
right_pin = 10

GPIO.setup(front_pin,GPIO.IN)
GPIO.setup(back_pin,GPIO.IN)
GPIO.setup(left_pin,GPIO.IN)
GPIO.setup(right_pin,GPIO.IN)

robot_x = Robot(left = leftPins_x, right = rightPins_x)
robot_y = Robot(left = leftPins_y, right = rightPins_y)


def robot_con(xl, xr, yl, yr):
    robot_x.value = (xl, xr)
    robot_y.value = (yl, yr)

def read_IR():
    return (GPIO.input(front_pin), GPIO.input(back_pin), GPIO.input(left_pin), GPIO.input(right_pin))

def avoid(IR):
    speed = 0.5
    
    if IR[0] == False: #if front infrared, go backwards
        print('Backward')
        robot_con(-speed, -speed, 0, 0)
    elif IR[1] == False: #if back infrared, go forwards
        print('forward')
        robot_con(speed, speed, 0, 0)
    elif IR[2] == False: #if left infrared, go right
        print('left')
        robot_con(0, 0, -speed, -speed)
    elif IR[3] == False: #if right infrared, go left
        print('right')
        robot_con(0, 0, speed, speed)
    else: # else stop
        print('stop')
        robot_con(0, 0, 0, 0)

while True:
    avoid(read_IR())
    #print(read_IR())
