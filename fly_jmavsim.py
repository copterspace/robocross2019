#!/usr/bin/env python
# coding=UTF-8
import rospy
import mavros
import mavros.command as mc
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
import tf.transformations as t
import math

current_state=State()
current_pose = PoseStamped()
current_vel = Twist()

def localpose_callback(data):
    global current_pose
    current_pose = data

def publish_setvel(event):
    global current_pose, setvel_pub, setvel, setvel_forward
    q=current_pose.pose.orientation.x, current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w
    roll, pitch, yaw = t.euler_from_quaternion(q)
    setvel.linear.x = setvel_forward * math.cos(yaw)
    setvel.linear.y = setvel_forward * math.sin(yaw)
    setvel_pub.publish(setvel)

def main():
    global current_pose, setvel, setvel_pub, setvel_forward

    rospy.init_node("offbrd",anonymous=True)
    rate=rospy.Rate(10)
    pose_sub=rospy.Subscriber("/mavros/local_position/pose",PoseStamped,localpose_callback)
    setvel_pub=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=1)
    arming_s=rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
    set_mode=rospy.ServiceProxy("/mavros/set_mode",SetMode)
    setvel=Twist()
    setvel_forward = 0

    arming_s(True)
    set_mode(0,"AUTO.TAKEOFF")
    print 'Taking off.....\r'
    rospy.sleep(5)
    
    # keyboard manipulation
    import curses
    stdscr = curses.initscr()
    curses.noecho()
    stdscr.nodelay(1)
    stdscr.keypad(1)

    for i in range (0,10):
        setvel_pub.publish(setvel)
        rate.sleep()
    set_mode(0,"OFFBOARD")
    setvel_timer = rospy.Timer(rospy.Duration(0.05), publish_setvel)
    while (rospy.is_shutdown()==False):
        rate.sleep()
        # keyboard  hcommands handling    
        c = stdscr.getch()
        if c == ord('q'): break  # Exit the while()
        elif c == ord('u'): setvel.linear.z += 0.25
        elif c == ord('d'): setvel.linear.z -= 0.25
        elif c == curses.KEY_LEFT: setvel.angular.z += 0.25
        elif c == curses.KEY_RIGHT: setvel.angular.z -= 0.25
        elif c == curses.KEY_UP: setvel_forward += 0.25 
        elif c == curses.KEY_DOWN: setvel_forward -= 0.25
        elif c == ord('s'): setvel_forward=setvel.linear.z=setvel.angular.z=0
        if  c!=curses.ERR: 
          print setvel,'\r'
    curses.endwin()
    set_mode(0,"AUTO.LAND")
    print 'Landing.......\r'

if __name__=="__main__":
    main()


