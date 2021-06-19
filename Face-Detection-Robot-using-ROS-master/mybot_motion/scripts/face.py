#!/usr/bin/env python
from __future__ import print_function # package used to import print function

import time # time module to display time
import roslib  # contains common data structures
import sys # system module
import rospy #python client library for ROS
import cv2   # opencv module 
from std_msgs.msg import String # representing primitive data types and other basic message constructs, such as multiarrays
from sensor_msgs.msg import Image # #package for Camera module integrated with robot
from cv_bridge import CvBridge, CvBridgeError # package to convert opencv image into ros supporting image

face_classifier=cv2.CascadeClassifier('/home/smartbridge/project/src/mybot_motion/scripts/haarcascade_frontalface_default.xml') #specify the xml file path

class image_converter: # main class

  def __init__(self): # init method
    self.image_pub = rospy.Publisher("mybot/camera/face",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("mybot/camera/image_raw",Image,self.callback)
 
  def callback(self,data): #call_back method
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    """(rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    cv2.imshow("face detect", cv_image)
    cv2.waitKey(3)"""
    rospy.loginfo("image")
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    faces=face_classifier.detectMultiScale(gray,1.3,5)
    #cv2.imshow("face detect", cv_image)
    #cv2.waitKey(3)

    for(x,y,w,h) in faces:
        #print('rect')
        cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,0,255), 2)
        cv2.putText(cv_image, 'Face', (x,y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        cv2.imshow('Face detection', cv_image)
        cv2.waitKey(3)
        picname=time.strftime("%H-%M-%S")
        image=picname+".png"
        cv2.imwrite(image, cv_image)
        #rospy.loginfo('print rectangle')
        try:
           self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
           print(e)

def main(args): #main function
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
