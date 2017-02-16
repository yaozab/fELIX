import rospy
import roslib
import cv2
import sys
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class image_stream:
  def __init__(self):
    # initlize subscriber
    self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_color/", Image, self.rgb_callback)
    self.depth_image_sub = rospy.Subscriber("/camera/depth/image/", Image, self.depth_callback)
    self.bridge = CvBridge()
    print("initilized")
    cv2.namedWindow("rgb")
    cv2.namedWindow("depth")

  def rgb_callback(self,data):
    try:
      cv_rgbimage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #print("showing rgb image")
    cv2.imshow("rgb", cv_rgbimage)
    cv2.waitKey(1)

  def depth_callback(self,data):
    try:
      cv_depthimage = self.bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
      print(e)

    #print("showing depth image")
    # normalize the depth image
    copy = cv_depthimage.copy()
    depth_norm = cv2.normalize(cv_depthimage,copy, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
    #print(depth_norm)
    
    cv2.imshow("depth", depth_norm)
    cv2.waitKey(1)

def main(args):
  ic = image_stream()
  rospy.init_node('image_stream', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
