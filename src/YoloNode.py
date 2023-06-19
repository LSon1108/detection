#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np
from ultralytics import YOLO
# Instantiate CvBridge
model = YOLO('yolov8n.pt')
bridge =CvBridge()

def search(list):
    for r in list:
        if r[4]==0:
            print("co nguoi trong khung hinh") 

def image_callback(msg):
    
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = model(cv2_img,conf=0.25)
        annotated_frame = results[0].plot()
        
        
            # box=boxes[0]

        for r in results:
        
                boxes = r.boxes
                list=[] 
                for box in boxes:
                       
                    b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                    c = box.cls

                    x1,y1,x2,y2=box.xyxy[0]
                    x1=int(x1)
                    x2=int(x2)
                    y1=int(y1)
                    y2=int(y2)
                    c=int(c)
                    res=[x1,y1,x2,y2,c]
                    list.append(res)
                    search(list)
                    # print("x1 = "+ str(x1) +" y1 = "+str(y1)+" x2 = "+str(x2)+" y2= "+str(y2)+str(c))    
        #except:
        #   print("no detection") 
            
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # cv2.imshow('camera_image', cv2_img)
        cv2.imshow('camera_image', annotated_frame)
        cv2.waitKey(1)
def subImage():
    rospy.init_node('YOLO')
    # Define your image topic
    image_topic = "/sensors/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    print("Received an image!")
    subImage()
