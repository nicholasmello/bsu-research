#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
b : switch to OmniMode/CommonMode
CTRL-C to quit
"""
Omni = 0


moveBindings = {
        'i':( 1, 0),
        'o':( 1,-1),
        'j':( 0, 1),
        'l':( 0,-1),
        'u':( 1, 1),
        ',':(-1, 0),
        '.':(-1, 1),
        'm':(-1,-1),
           }


speedBindings={
        'q':(1.1,1.1),
        'z':(0.9,0.9),
        'w':(1.1,1),
        'x':(0.9,1),
        'e':(1,  1.1),
        'c':(1,  0.9),
          }


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = 0.2
turn  = 0.5

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    x      = 0
    th     = 0
    count  = 0
    target_speed = 0
    target_turn  = 0
    target_HorizonMove = 0
    control_speed = 0
    control_turn  = 0
    control_HorizonMove = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()

            
            if key=='b':               
                Omni=~Omni
                if Omni: 
                    print("Switch to OmniMode")
                    moveBindings['.']=[-1,-1]
                    moveBindings['m']=[-1, 1]
                else:
                    print("Switch to CommonMode")
                    moveBindings['.']=[-1, 1]
                    moveBindings['m']=[-1,-1]
            
            
            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0

            
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn  = turn  * speedBindings[key][1]
                count = 0
                print(vels(speed,turn))

            
            elif key == ' ' or key == 'k' :
                x  = 0
                th = 0
                control_speed = 0
                control_turn  = 0
                HorizonMove   = 0

            
            else:
                count = count + 1
                if count > 4:
                    x  = 0
                    th = 0
                if (key == '\x03'):
                    break

            
            target_speed = speed * x
            target_turn  = turn * th
            target_HorizonMove = speed*th

            
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.1 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.1 )
            else:
                control_speed = target_speed

            
            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.5 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.5 )
            else:
                control_turn = target_turn

            
            if target_HorizonMove > control_HorizonMove:
                control_HorizonMove = min( target_HorizonMove, control_HorizonMove + 0.1 )
            elif target_HorizonMove < control_HorizonMove:
                control_HorizonMove = max( target_HorizonMove, control_HorizonMove - 0.1 )
            else:
                control_HorizonMove = target_HorizonMove
         
            twist = Twist() 
            
            if Omni==0:
                twist.linear.x  = control_speed; twist.linear.y = 0;  twist.linear.z = 0
                twist.angular.x = 0;             twist.angular.y = 0; twist.angular.z = control_turn
            else:
                twist.linear.x  = control_speed; twist.linear.y = control_HorizonMove; twist.linear.z = 0
                twist.angular.x = 0;             twist.angular.y = 0;                  twist.angular.z = 0

            print(twist.linear.x)
            pub.publish(twist) 

    
    except Exception as e:
        print(e)

    
    finally:
        twist = Twist()
        twist.linear.x = 0;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

