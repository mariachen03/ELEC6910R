#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import Image
from nav_msgs.msg import MapMetaData, OccupancyGrid


# face recognition
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sys
import operator

cand0 = cv2.imread('/home/svenja/catkin_ws/src/rpl_project/src/candidates/pic01.png')
cand1 = cv2.imread('/home/svenja/catkin_ws/src/rpl_project/src/candidates/pic02.png')
cand2 = cv2.imread('/home/svenja/catkin_ws/src/rpl_project/src/candidates/pic03.png')
cand3 = cv2.imread('/home/svenja/catkin_ws/src/rpl_project/src/candidates/pic04.png')
cand4 = cv2.imread('/home/svenja/catkin_ws/src/rpl_project/src/candidates/pic05.png')

cands = [cand0, cand1, cand2, cand3, cand4]
cand_names = ['Obama', 'Avril', 'Zhang', 'Orlando', 'Cartoon']

def comp_dist(x0, x1):
    if x0.shape != x1.shape:
        raise Exception("The shapes of two compared images should be same!")

    diff = np.power(x0-x1, 2)
    dist = np.sum(np.sum(np.sum(diff)))
    dist = dist / x0.size
    return dist

cascPath = "/home/svenja/catkin_ws/src/rpl_project/src/haarcascade_frontalface_default.xml"

# Create the haar cascade
faceCascade = cv2.CascadeClassifier(cascPath)


# call back
vel_msg = Twist()
vel_msg.linear.x = 1.0
vel_msg.linear.y = 1.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0

bridge = CvBridge()

policy = True

# position_pre = Point()
# position_pre.x = 1000.0 # a big number
# position_pre.y = 1000.0 # a big number
# position_pre.z = 0.0

def turn90():
    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.angular.z = 1.0

def turn_normal():
    vel_msg.linear.x = 1.0
    vel_msg.linear.y = 1.0
    vel_msg.angular.z = 0.0

def pose_callback(data):
    pass
    # print(data.pose.pose.position)
    # position = data.pose.pose.position
    # diff = (position.x-position_pre.x)*(position.x-position_pre.x) + (position.y-position_pre.y)*(position.y-position_pre.y)
    # # print(diff)
    # if(diff<0.0001):
    #     turn90()
    #
    # position_pre.x = position.x
    # position_pre.y = position.y

def image_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # (rows,cols,channels) = cv_image.shape

    # if cols > 60 and rows > 60:
    #     cv2.circle(cv_image, (50,50), 10, 255)

    if cv_image is None:
        return

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Detect faces in the image
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30)
        #flags = cv2.CV_HAAR_SCALE_IMAGE
    )

    print("Found {0} faces!".format(len(faces)))

    for (x, y, w, h) in faces:
        sample = cv_image[y:y+h,x:x+w]

        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        dist_list = []
        for cand in cands:
            resized_cand = cv2.resize(cand, (w, h))
            dist = comp_dist(resized_cand, sample)
            dist_list.append(dist)
            print(dist)

        index, value = min(enumerate(dist_list), key=operator.itemgetter(1))

        # print((x, y, w, h), ' is: ', index)
        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(cv_image,cand_names[index], (x, y+h), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(10)

def mapmetadata_callback(data):
    pass

def map_callback(data):
    pass

def drive():
    rospy.init_node('driver', anonymous=True)
    # pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
    # rate = rospy.Rate(10) # 10hz

    # listen to feedback from the car
    rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber("/vrep/image", Image, image_callback)
    rospy.Subscriber("/map_metadata", MapMetaData, mapmetadata_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    # while not rospy.is_shutdown():
    #     global policy
    #     if(policy):
    #         turn90()
    #     else:
    #         turn_normal()
    #     policy = not policy
    #     pub.publish(vel_msg)
    #     rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        drive()
    except rospy.ROSInterruptException:
        pass
