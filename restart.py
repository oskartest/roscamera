#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])
steers=0
image_rgb=0
def img_callback(data):
    global cv_image, image_rgb , steer
    cv_image = bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(7,2+5),0)
    height, width, channels = cv_image.shape[:3]
    x_margin, y_margin = 0.3, 0.75
    _, binary_img = cv2.threshold(blur, 200 , 255, cv2.THRESH_BINARY)
    image_rgb = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2RGB)
    l_y, l_x = binary_img[int(y_margin*height): height, 0 : int(x_margin * width)].nonzero()
    r_y, r_x = binary_img[int(y_margin*height): height, int((1-x_margin) * width) : width].nonzero()
    leftx = np.average(l_x) if len(l_x) else 0
    rightx = np.average(r_x) + (1 - x_margin) * width if len(r_x) else width
    midx = int((leftx + rightx)/2)
    cv2.circle(image_rgb, (int(leftx), int(0.75 * height)), 10, (0,0,255), -1)
    cv2.circle(image_rgb, (int(rightx), int(0.75 * height)), 10, (0,0,255), -1)
    cv2.circle(image_rgb, (int(midx), int(0.75 * height)), 10, (0,0,255), -1)
    steer = -np.arctan2(midx - (width/2), height*y_margin)
    # steerss = round(math.degrees(steer),3)
    # steers = 2.5+(steerss*(12.5-2.5)/180.0)

rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, img_callback)

def main():
    while not rospy.is_shutdown():
	cv2.imshow("display",image_rgb)
        cv2.waitKey(33)
	
        if cv_image.size != (640*480*3):
            continue
        
        rate = rospy.Rate(30) # 10hz
        pub = rospy.Publisher('/steers', Float32, queue_size=10) #topic
        
        rospy.loginfo(steer)
        pub.publish(steer)
        rate.sleep()
if __name__ == '__main__':
    rospy.init_node('cam_tune', anonymous=True)
    main()
    rospy.spin()
