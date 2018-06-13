#!/usr/bin/env python
import rospy
import sys
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import face_recognition


class FaceRecognition():
    def __init__(self):
        self.node_name = "Face_recognition"
        rospy.init_node(self.node_name)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        #cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        #cv.MoveWindow(self.cv_window_name, 1000, 50)
        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image
        self.image_sub = rospy.Subscriber("/camera/process", Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher("GUI/face_result",Image,queue_size=10)
        self.result_pub = rospy.Publisher('face_result', String, queue_size=10)
        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("/camera/process", Image)
        rospy.loginfo("Ready.")

        #initial the face_recognition
	# Load a sample picture and learn how to recognize it.
        self.mqh_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/MQH.png")
        self.mqh_face_encoding = face_recognition.face_encodings(self.mqh_image)[0]
        self.rc_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/RC.jpg")
        self.rc_face_encoding = face_recognition.face_encodings(self.rc_image)[0]
        # Load a sample picture and learn how to recognize it.
        self.sh_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/SH")
        self.sh_face_encoding = face_recognition.face_encodings(self.sh_image)[0]
        # Load a sample picture and learn how to recognize it.
        self.syz_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/SYZ")
        self.syz_face_encoding = face_recognition.face_encodings(self.syz_image)[0]
        # Load a second sample picture and learn how to recognize it.
        self.zc_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/ZC")
        self.zc_face_encoding = face_recognition.face_encodings(self.zc_image)[0]
        # Load a second sample picture and learn how to recognize it.
        self.zwp_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/ZWP")
        self.zwp_face_encoding = face_recognition.face_encodings(self.zwp_image)[0]
        # Load a second sample picture and learn how to recognize it.
        self.zwj_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/ZWJ")
        self.zwj_face_encoding = face_recognition.face_encodings(self.zwj_image)[0]
        # Load a sample picture and learn how to recognize it.
        self.sjh_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/SJH")
        self.sjh_face_encoding = face_recognition.face_encodings(self.sjh_image)[0]
        # Load a sample picture and learn how to recognize it.
        self.lhb_image = face_recognition.load_image_file("/home/sun/catkin_ITF/src/face_rc/src/images/LHB")
        self.lhb_face_encoding = face_recognition.face_encodings(self.lhb_image)[0]
        # Create arrays of known face encodings and their names
        self.known_face_encodings = [self.mqh_face_encoding,self.rc_face_encoding,self.sjh_face_encoding,self.lhb_face_encoding,self.sh_face_encoding,self.zc_face_encoding,self.zwp_face_encoding,self.zwj_face_encoding,self.syz_face_encoding]
        self.known_face_names = ["MQH","RC","SJH","LHB","SH","ZC","ZWP","ZWJ","SYZ"]

        # Initialize some variables
        self.face_locations = []
        self.face_encodings = []
        self.face_names = []
        self.process_this_frame = True
        self.memory_name = "Unknown"
        self.names = ["U","U","U","U","U","U","U","U","U","U"]

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        # Convert the image to a numpy array since most cv2 functions
        # require numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        # Process the frame using the process_image() function
        display_image = self.process_image(frame)
        frame_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
        self.image_pub.publish(frame_msg)
        # Display the image.
        #cv2.imshow(self.node_name, display_image) show on the GUI no independent window 2018.6.12
        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    def process_image(self, frame):
        frame = frame[120:360, 160:480]
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame#[:, :, ::-1]
        if self.process_this_frame:
            self.face_locations = face_recognition.face_locations(rgb_small_frame)
            self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)
            self.face_names = []
            for face_encoding in self.face_encodings:
                #matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding,tolerance=0.39)
                name = "Unknown"
                if True in matches:
                    first_match_index = matches.index(True)
                    name = self.known_face_names[first_match_index]
                    del self.names[0]
                    self.names.append(name)
                    count  = self.names.count(name)
                    if (self.memory_name != name and count>6):
                        self.memory_name = name
                        rospy.loginfo(name)
                        self.result_pub.publish(name)
                self.face_names.append(name)
        self.process_this_frame = not self.process_this_frame
        for (top, right, bottom, left)in self.face_locations:
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
        return frame


def main(args):
    try:
        FaceRecognition()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

