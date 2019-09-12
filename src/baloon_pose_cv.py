#!/usr/bin/env python
# coding=UTF-8
# zuza baloon seeker
import rospy
from sensor_msgs.msg import CameraInfo, Image
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
#import tf.transformations as t
from geometry_msgs.msg import TwistStamped, Quaternion, Point, PoseStamped, TransformStamped
import copy
from std_msgs.msg import ColorRGBA
import tf2_ros
import math 
import yaml

my_cam_info=None
my_fps=0
my_last_fps=0
my_time_fps=time.time()
bridge = CvBridge()
mtx=None
dist=None

def raspicam_loop_cv(): 
    global my_fps, my_last_fps, my_time_fps, mtx, dist
    # initialize the camera and grab a reference to the raw camera capture
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, my_cam_info.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, my_cam_info.height)
    camera.set(cv2.CAP_PROP_FPS, rospy.get_param('/baloon_detector/framerate'))
    # allow the camera to warmup
    time.sleep(0.1)

    color_blue = (255,255,0)
    color_red = (0,0,255)
    color_green = (0,255,0)
    color_white = (255,255,255)
    color_yellow = (0,255,255)
    color_black = (0,0,0)

    # считываем значения бегунков
    invert = rospy.get_param('/baloon_detector/invert')
    h1 = rospy.get_param('/baloon_detector/h1')
    s1 = rospy.get_param('/baloon_detector/s1')
    v1 = rospy.get_param('/baloon_detector/v1')
    h2 = rospy.get_param('/baloon_detector/h2')
    s2 = rospy.get_param('/baloon_detector/s2')
    v2 = rospy.get_param('/baloon_detector/v2')
    real_ballon_r = rospy.get_param('/baloon_detector/real_ballon_r')

    # формируем начальный и конечный цвет фильтра
    h_min = np.array((h1, s1, v1), np.uint8)
    h_max = np.array((h2, s2, v2), np.uint8)

    while not rospy.is_shutdown(): 
      ret, image_raw = camera.read()
      # calculate FPS
      cur_time2 = time.time()
      if cur_time2-my_time_fps>5:
          my_last_fps=my_fps
          my_fps=1
          my_time_fps=cur_time2
      else:
          my_fps+=1
      
      image_hsv = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV )
      # накладываем фильтр на кадр в модели HSV
      thresh = cv2.inRange(image_hsv, h_min, h_max)
      if invert>0:
          cv2.bitwise_not(thresh,thresh)
      #find contours
      _, contours0, hierarchy = cv2.findContours( thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      max_area = 0
      max_center_x=None
      max_center_y=None
      max_baloon_r=None
      max_yaw_shift=None
      for cnt in contours0:
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        center = (int(rect[0][0]),int(rect[0][1]))
        area = int(rect[1][0]*rect[1][1])
        if area > 100:
            baloon_r = int(math.sqrt(area)/2)
            yaw_shift = int( (center[0] - my_cam_info.width/2) * 130 / my_cam_info.width) # 130 degree camera for my_cam_info.width pics image
            cv2.circle(image_raw, center, baloon_r , color_red, 1)
            if area>max_area:
                max_area=area
                max_center_x = center[0]
                max_center_y = center[1]
                max_baloon_r = baloon_r
                max_yaw_shift = yaw_shift
      #draw main baloon
      if max_area>0:
        cv2.circle(image_raw, (max_center_x,max_center_y), max_baloon_r+2 , color_blue, 2)
        cv2.putText(image_raw, "D=%d" % int(max_baloon_r*2), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_red, 1)
        cv2.putText(image_raw, "angle=%d '" % max_yaw_shift, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_green, 1)
        max_distance = real_ballon_r/math.tan(max_baloon_r/float(my_cam_info.width)*(math.pi*160.0/360.0)) #distance to baloon
        #print max_distance
        cv2.putText(image_raw, "%f m" % max_distance, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_blue, 1)
        cv2.line(image_raw, (max_center_x, 1), (max_center_x, my_cam_info.height-1) , color_white, 1)
        cv2.line(image_raw, (1,max_center_y), (my_cam_info.width-1, max_center_y) , color_white, 1)
        # post baloon Twist
        tws = TwistStamped()
        tws.header.stamp = rospy.Time.now()
        tws.header.frame_id = 'fcu'
        tws.twist.linear.x = max_distance
        tws.twist.angular.z = float(-max_yaw_shift)/360.0*math.pi#yaw
        tws.twist.angular.y = float( (max_center_y - my_cam_info.height/2) * 130.0 / my_cam_info.height) /360.0*math.pi #pitch
        publisher_baloontwist.publish(tws)

      #draw target
      cv2.line(image_raw, (my_cam_info.width/2-10, my_cam_info.height/2-10), (my_cam_info.width/2+10, my_cam_info.height/2+10) , color_yellow, 1)
      cv2.line(image_raw, (my_cam_info.width/2+10, my_cam_info.height/2-10), (my_cam_info.width/2-10, my_cam_info.height/2+10) , color_yellow, 1)
      #Draw FPS on image
      cv2.putText(image_raw,"fps: "+str(my_last_fps/5),(120,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,0),1,cv2.LINE_AA)
      # публикуем 1 кадр из 5, дабы уменьшить тормоза по вайфаю
      if my_fps%5==0: 
        try:
          publisher_image_raw_result.publish(bridge.cv2_to_imgmsg(image_raw,"bgr8"))
          publisher_image_raw_filter.publish(bridge.cv2_to_imgmsg(thresh, "mono8"))
        except CvBridgeError as e:
          print(e)

def publish_dummy_vp(event):
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'local_origin'
    ps.pose.orientation.w = 1;
    publisher_vp.publish(ps);

def callback_handle_pose(mavros_pose):
    rospy.loginfo("Got mavros pose, stop publishing zeroes.")
    dummy_timer.shutdown() 
    handle_pose_sub.unregister()

def process_camera_info_yaml():
    global mtx, dist, my_cam_info
    my_cam_info = CameraInfo()
    filename = rospy.get_param('/baloon_detector/camera_info_url')
    with open(filename, 'r') as ymlfile:
        cfg = yaml.load(ymlfile)
    my_cam_info.width = cfg['image_width']
    my_cam_info.height= cfg['image_height']
    my_cam_info.K = cfg['camera_matrix']['data']
    my_cam_info.D = cfg['distortion_coefficients']['data']

    mtx=np.zeros((3,3))
    dist=np.zeros((1,5))
    for i in range(3):
      for j in range(3):
        mtx[i,j]=my_cam_info.K[i*3+j]
    for i in range(5):
      dist[0,i]=my_cam_info.D[i]
    print mtx, dist   

if __name__ == '__main__':
    rospy.init_node('baloon_detector')
    ## init pose publishing from FCU
    publisher_vp = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    dummy_timer=rospy.Timer(rospy.Duration(0.5), publish_dummy_vp)
    handle_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback_handle_pose)
    
    ## init camera
    process_camera_info_yaml()
    print my_cam_info
    rospy.loginfo("image translation starting.........")
    publisher_image_raw_filter = rospy.Publisher('baloon_detector/image_filter', Image, queue_size=1)
    publisher_image_raw_result = rospy.Publisher('baloon_detector/image_result', Image, queue_size=1)
    publisher_baloontwist= rospy.Publisher('baloon_detector/twist', TwistStamped ,queue_size=1)
    raspicam_loop_cv()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
