from readchar import readchar
from gpiozero import Robot
from time import sleep
from pynput.keyboard import Key, KeyCode, Listener

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
    
def on_press(key):
    #print('{0} pressed'.format(key))
    if key == KeyCode.from_char('w'):
        print 'forward'
        robot_con(1, 1, 0, 0)
    elif key == KeyCode.from_char('x'):
        print 'Backward'
        robot_con(-1, -1, 0, 0)
    elif key == KeyCode.from_char('a'):
        print 'Left'
        robot_con(0, 0, -1, -1)
    elif key == KeyCode.from_char('d'):
        print 'Right'
        robot_con(0, 0, 1, 1)
    elif key == KeyCode.from_char('q'):
        print 'Top Left'
        robot_con(1, 1, -1, -1)
    elif key == KeyCode.from_char('e'):
        print 'Top Right'
        robot_con(1, 1, 1, 1)
    elif key == KeyCode.from_char('c'):
        print 'Bottom Left'
        robot_con(-1, -1, 1, 1)
    elif key == KeyCode.from_char('z'):
        print 'Bottom Right'
        robot_con(-1, -1, -1, -1)
    elif key == KeyCode.from_char('f'):
        print 'anti-clockwise'
        robot_con(0.5, -0.5, -0.5, 0.5)
    elif key == KeyCode.from_char('g'):
        print 'clockwise'
        robot_con(-0.5, 0.5, 0.5, -0.5)
    

def on_release(key):
    #print('{0} release'.format(key))
    if key == KeyCode.from_char('o'):
        return False
    robot_con(0, 0, 0, 0)
    

with Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
