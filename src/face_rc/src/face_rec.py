#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import face_recognition
import pickle
import os

#initial the face_recognition
# Load a sample picture and learn how to recognize it.
pkl_names = open(sys.path[0]+'/face_names.pkl','rb')
pkl_encodings = open(sys.path[0]+'/face_encodings.pkl','rb')
known_face_names = pickle.load(pkl_names)
known_face_encodings = pickle.load(pkl_encodings)
# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
#self.process_this_frame = True
process = True
memory_name = "Unknown"
names = ["U","U","U","U","U","U","U","U","U","U"]
# Create the cv_bridge object
bridge = CvBridge()

node_name = "Face_recognition"
rospy.init_node(node_name)


def image_callback(ros_image):

    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError, e:
        print e
    # Convert the image to a numpy array since most cv2 functions
    # require numpy arrays.
    frame = np.array(frame, dtype=np.uint8)
    # Process the frame using the process_image() function
    display_image = process_image(frame)

    frame_msg = bridge.cv2_to_imgmsg(display_image, "bgr8")
    image_pub.publish(frame_msg)

def process_image(frame):
    frame = frame[120:360, 160:480]
    # Resize frame of video to 1/4 size for faster face recognition processing
    rgb_small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    #rgb_small_frame = small_frame#[:, :, ::-1]
    global process
    global face_locations
    global face_encodings
    global memory_name
    if process:
        face_locations = face_recognition.face_locations(rgb_small_frame)
        #self.face_locations = face_recognition.face_locations(rgb_small_frame,model = 'cnn')
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding,tolerance=0.39)
            name = "Unknown"
            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]
                del names[0]
                names.append(name)
                count  = names.count(name)
                if (memory_name != name and count>6):
                    memory_name = name
                    rospy.loginfo(name)
                    result_pub.publish(name)
            face_names.append(name)
    #self.process_this_frame = not self.process_this_frame
    process = not process
    for (top, right, bottom, left)in face_locations:
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
    return frame


# Subscribe to the camera image
image_sub = rospy.Subscriber("/camera/image", Image, image_callback, queue_size=1)
image_pub = rospy.Publisher("GUI/face_result",Image,queue_size=10)
result_pub = rospy.Publisher('face_result', String, queue_size=10)
rospy.loginfo("Waiting for image topics...")
rospy.wait_for_message("/camera/image", Image)
rospy.loginfo("Ready.")

def main(args):
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

