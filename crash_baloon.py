#!/usr/bin/env python
# coding=UTF-8
# baloon position steering
import rospy
import mavros
import mavros.command as mc
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, TwistStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
import tf.transformations as t
import math

current_state=State()
current_pose = PoseStamped()
current_vel = Twist()
baloon_twist = TwistStamped()

def state_callback(data):
    global current_state
    current_state=data

def localpose_callback(data):
    global current_pose
    current_pose = data

def baloon_callback(data):
    global baloon_twist
    baloon_twist = data

def publish_setvel(event):
    global current_pose, setvel_pub, setvel, setvel_forward, baloon_twist
    q=current_pose.pose.orientation.x, current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w
    roll, pitch, yaw = t.euler_from_quaternion(q)
    setvel.linear.x = setvel_forward * math.cos(yaw)
    setvel.linear.y = setvel_forward * math.sin(yaw)
    setvel_pub.publish(setvel)

def main():
    global current_pose, setvel, setvel_pub, setvel_forward, baloon_twist

    rospy.init_node("offbrd",anonymous=True)
    rate=rospy.Rate(10)
    state=rospy.Subscriber("/mavros/state",State,state_callback)
    pose_sub=rospy.Subscriber("/mavros/local_position/pose",PoseStamped,localpose_callback)
    baloon_sub=rospy.Subscriber("/baloon_detector/twist",TwistStamped,baloon_callback)
    setvel_pub=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=1)
    arming_s=rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
    set_mode=rospy.ServiceProxy("/mavros/set_mode",SetMode)
    setvel=Twist()
    setvel_forward = 0

    arming_s(True)
    set_mode(0,"AUTO.TAKEOFF")
    print 'Taking off.....\r'
    rospy.sleep(5)
    
    for i in range (0,10):
        setvel_pub.publish(setvel)
        rate.sleep()
    set_mode(0,"OFFBOARD")
    
    setvel_timer = rospy.Timer(rospy.Duration(0.05), publish_setvel)
    
    while not rospy.is_shutdown():
        time_delay = rospy.Time.now().to_sec() - baloon_twist.header.stamp.to_sec()
        #print baloon_twist
        print 'time delay = ',time_delay
        if time_delay<0.2:#последний раз шарик видели 0.2 секунды назад 
            if baloon_twist.twist.linear.x > 0.8:
                setvel_forward = 1.5
            elif baloon_twist.twist.linear.x > 0.8:
                setvel_forward = 0.0
            else:
                setvel_forward = -0.5
            setvel.angular.z = baloon_twist.twist.angular.z*4
            if baloon_twist.twist.angular.y<0:
                setvel.linear.z=0.5
            elif baloon_twist.twist.angular.y>0.2:
                setvel.linear.z=-0.25
            else:
                setvel.linear.z=0
        else:#шарик потерян из виду
            setvel.angular.z=setvel_forward=setvel.linear.z=0
        print setvel, setvel_forward
        rate.sleep()

    set_mode(0,"AUTO.LAND")
    print 'Landing.......\r'
    setvel_timer.shutdown()
    rospy.sleep(5)

if __name__=="__main__":
    main()