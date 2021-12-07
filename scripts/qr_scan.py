#!/bin/python3
import cv2
import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Quaternion

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
loc_product = Pose()

def get_pose(msg):
    loc_product = msg.pose.pose

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
            #print(f"qrdata {qrdata}")
            qrinfo = qrdata[0].data
            #print(qrinfo)
            #type(qrinfo)
            if qrinfo not in product_loc_dict.keys():
                product_loc_dict[qrinfo] = list()
                product_loc_dict[qrinfo].append(loc_product)
                print(product_loc_dict)
            #Since QR information is present we record the loc
            odom_sub = rospy.Subscriber('/odom', Odometry, callback=get_pose)
                
    except CvBridgeError as e:
        rospy.logerr(e)
    

rospy.init_node('qr_scan_node')
sub = rospy.Subscriber("/turtlebot3_burger/camera1/image_raw", Image, callback=img_callback)  # Subscriber object which will listen "LaserScan" type messages from the "/scan" Topic and call the "callback" function each time it reads something from the Topic
rospy.spin() # Loops infinitely until someone stops the program execution