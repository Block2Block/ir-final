#!/bin/python3
import cv2
import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry

THRESH = 50     #threshold for image

product_loc_dict = dict()
loc_product = ""

def get_pose(msg):
    loc_product = msg.pose.pose

def img_callback(imgdata):

    try:
        imagecv = CvBridge.imgmsg_to_cv2(imgdata)       #convert rosimage to cvimage
        imgbw = cv2.threhold(imagecv, THRESH, 255, cv2.THRESH_BINARY)[1]        #thresholding
        qrdata = decode(imgbw)      #extracting qr code information
        if qrdata:
            qrinfo = qrdata[0].data
            if qrinfo not in product_loc_dict.keys():
                product_loc_dict[qrinfo] = list()
            
            #Since QR information is present we record the loc
            odom_sub = rospy.Subscriber('/odom', Odometry, callback=get_pose)
            product_loc_dict[qrinfo].append(loc_product)
            print(product_loc_dict)

    except CvBridgeError as e:
        rospy.logerr(e)
    

rospy.init_node('qr_scan_node')
sub = rospy.Subscriber("/turtlebot3_burger/camera1/image_raw", Image, callback=img_callback)  # Subscriber object which will listen "LaserScan" type messages from the "/scan" Topic and call the "callback" function each time it reads something from the Topic
rospy.spin() # Loops infinitely until someone stops the program execution