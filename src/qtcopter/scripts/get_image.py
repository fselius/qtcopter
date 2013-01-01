#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv2

rospy.init_node('get_image')
img = rospy.wait_for_message('/image', Image)
print type(img), type(img.data), dir(img)
print dir(img.data)
cv2.imwrite('image.jpg', img.data)
print 'wrote to image.jpg'


