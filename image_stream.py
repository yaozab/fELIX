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
    # initilize publishers
    self.image_pub = rospy.Publisher("image_stream", Image)
    # initlize subscriber
    self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.rgb_callback)
    self.depth_image_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)
    self.bridge = CvBridge()
    print("initilized")
    cv2.namedWindow("rgb")
    cv2.namedWindow("depth")
  def rgb_callback(self,data):
    try:
      cv_rgbimage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    print("showing rgb image")
    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #    cv2.circle(cv_image, (50,50), 10, 255)
    #print(cv_rgbimage)
    cv2.imshow("rgb", cv_rgbimage)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_rgbimage, "bgr8"))
    except CvBridgeError as e:
      print(e)
  def depth_callback(self,data):
    try:
      cv_depthimage = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)

    print("showing depth image")
    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #    cv2.circle(cv_image, (50,50), 10, 255)
    #depth_array = np.array(cv_depthimage, dtype=np.float32)
    depth_norm = cv2.normalize(cv_depthimage, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    print(depth_norm)
    cv2.imshow("depth", depth_norm)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(depth_norm, "32FC1"))
    except CvBridgeError as e:
      print(e)

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
