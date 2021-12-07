#!/bin/python3
import cv2
import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Quaternion
import math

THRESH = 40     #threshold for image

def increase_brightness(image, value=10):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    image = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return image

product_loc_dict = dict()

def round_nearest2(x, a):
    return round(round(x / a) * a, -int(math.floor(math.log10(a))))

def get_pose(msg):
    #print(f"msg {msg}")
    global loc_product
    loc_product = Pose()
    loc_product = msg.pose.pose

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    # This has been rounded to the nearest to 0.03, as our dijkstra grid system is implemented with 0.03 stepping stone.
    global xy_tuple 
    xy_tuple = (round_nearest2(x, 0.03), round_nearest2(y, 0.03))

def img_callback(imgdata):
    img_bridge = CvBridge()
    try:
        imagecv = img_bridge.imgmsg_to_cv2(imgdata, "bgr8")       #convert rosimage to cvimage
        imagecv = increase_brightness(imagecv, value=5)
        imagecv = cv2.cvtColor(imagecv, cv2.COLOR_BGR2GRAY)
        imgbw = cv2.threshold(imagecv, THRESH, 255, cv2.THRESH_BINARY)[1]        #thresholding
        median_blur= cv2.medianBlur(imgbw, 5)       #to remove salt and pepper noise
        qrdata = decode(imgbw)      #extracting qr code information
        if qrdata:
            qrinfo = qrdata[0].data
            if qrinfo not in product_loc_dict.keys():
                product_loc_dict[qrinfo] = list()
                product_loc_dict[qrinfo].append(xy_tuple)
                print(product_loc_dict)
                
    except CvBridgeError as e:
        rospy.logerr(e)

rospy.init_node('qr_scan_node')
sub = rospy.Subscriber("/turtlebot3_burger/camera1/image_raw", Image, callback=img_callback)  # Subscriber object which will listen "LaserScan" type messages from the "/scan" Topic and call the "callback" function each time it reads something from the Topic
#Since QR information is present we record the loc
odom_sub = rospy.Subscriber('/odom', Odometry, callback=get_pose)
rospy.spin() # Loops infinitely until someone stops the program execution