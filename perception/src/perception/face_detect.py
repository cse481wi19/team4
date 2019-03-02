#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import PIL
from std_msgs.msg import String
import face_recognition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("head_camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    face_loc = face_recognition.face_locations(cv_image)

    print("I found {} face(s) in this photograph.".format(len(face_loc)))

    for face_location in face_loc:

    #     # Print the location of each face in this image
      top, right, bottom, left = face_location
      print("A face is located at pixel location Top: {}, Left: {}, Bottom: {}, Right: {}".format(top, left, bottom, right))

    #     # You can access the actual face itself like this:
      face_image = cv_image[top:bottom, left:right]
      pil_image = PIL.Image.fromarray(face_image)
      pil_image.show()
   
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)