import rospy
import roslib
import cv2
import sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_stream:
	def __init__(self):
		# initilize publishers
		self.image_pub = rospy.Publisher("image_stream", Image)
		# initlize subscriber
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)
		self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)
		self.bridge = CvBridge()
        print("initilized")

	def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #print("showing image")
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
