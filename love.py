# -*- coding: utf-8 -*-

import cv2
import numpy #library for mathematical functions
import rospy
from cv2 import namedWindow
from cv2 import destroyAllWindows, startWindowThread
from numpy import mean
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan #Used to check for collisions
from cv_bridge import CvBridge, CvBridgeError #conversion of ROS data to OpenCV format
from geometry_msgs.msg import Twist


class main:

    def __init__(self):
        rospy.loginfo("Starting Node")
        namedWindow("Image Window", 1)
        namedWindow("Green Isolation", 1)
        self.laser = rospy.Subscriber("/turtlebot_1/scan", LaserScan, self.avoid)
        self.bridge = CvBridge()
        startWindowThread()
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
                                          Image, self.callback)
        self.pub = rospy.Publisher("turtlebot_1/cmd_vel", Twist, queue_size = 1)                    
        
    
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        #sperating the green channel
        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((0, 0, 0)),
                                 numpy.array((0, 255, 0)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((25, 50, 10)),
                                 numpy.array((100, 255, 255)))

        bgr_contours, hierachy = cv2.findContours(bgr_thresh.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)

        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 2)
                
        cv2.imshow("Image Window", cv_image)        
        greenImg = cv2.bitwise_and(hsv_img,hsv_img,mask = hsv_thresh)
        
        cv2.imshow("Green Isolation", greenImg)        
        imageleft, imageright = numpy.hsplit(greenImg, 2)
        
        namedWindow("right", 1)     
        cv2.imshow("right", imageright)
        
        namedWindow("left", 1)   
        cv2.imshow("left", imageleft)
        
        rightIntensity = mean(imageright)        
        
        leftIntensity = mean(imageleft)
        
        
        #send_velocities is sent 0, saying that green is not detected
        if (leftIntensity>rightIntensity):
            self.send_velocities(0)
        else: #send_velocities is sent 1, as green is detected and action needs to be taken
            if (leftIntensity<rightIntensity):
                self.send_velocities(1)
            else:
                self.send_velocities(2)

            
    def send_velocities(self,x):
        r = rospy.Rate(10) #set frequency of commands
        rospy.loginfo("Sending commands")
        twist_msg = Twist()
        
        if (self.scan > 1): #Check if object detected
         
            if (x == 0):
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = 0.1
            else:
                #if green is in view, begin to twist
                if(x == 1):
                    twist_msg.angular.z = 1
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = -2
                else: #resume search for green colour again
                    twist_msg.linear.x = 0.2
                    twist_msg.angular.z = 0.1
                
        else:
            twist_msg.linear.x = 0;
            twist_msg.angular.z = -1;
                 
        self.pub.publish(twist_msg) # Sending the command
        r.sleep()

    #object collision is handled, to ensure that it does not bump into the environment       
    def avoid(self, data):
        self.scan = min(data.ranges)
            

            
main()
rospy.init_node('main', anonymous=True)
love = main()
## Calling the function
rospy.spin()
destroyAllWindows()