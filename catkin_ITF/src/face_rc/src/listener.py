#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import sys

class listener():
    def __init__(self):
        self.node_name = 'result_listener'
        rospy.init_node(self.node_name)
        self.door_memory = "U"
        self.numbers = ["U","U","U","U","U","U","U","U","U","U","U","U","U","U","U","U","U","U","U","U"]
        #self.memory_name = "Unknown"
        #self.memory_door = "U"
        rospy.Subscriber("face_result",String,self.face_callback)
        rospy.Subscriber("door_result",String,self.door_callback);
        #self.pub_2 = rospy.Publisher()
        self.pub = rospy.Publisher('speech/tts_topic', String, queue_size=1)

    def face_callback(self,data):
        rospy.loginfo("The result is : %s",data.data)
        #p1_data = self.process_max(data)
        p2_data = self.process_string(data)
        self.pub.publish(p2_data)

    def door_callback(self,data):
        self.process_max(data)
        if (self.numbers.count(data.data)>12 and self.door_memory != data.data):
            self.door_memory  = data.data
            print(self.door_memory)
            self.pub.publish(data.data)
            rospy.loginfo("I send door result: %s",data.data)

    #process data and output max number name
    def process_max(self,data):
        del self.numbers[0]
        self.numbers.append(data.data)


    def process_string(self,data):
	if data.data == "MQH":
            data.data = "你好啊孟老师"
	if data.data == "RC":
            data.data = "你好啊任老师"
        if data.data == "SYZ":
            data.data = "你好啊孙玉哲"
        if data.data == "ZWP":
            data.data = "你好啊翟文鹏"
        if data.data == "SH":
            data.data = "你好啊申毫"
        if data.data == "ZWJ":
            data.data = "你好啊郑文键"
        if data.data == "ZC":
            data.data = "你好啊张驰"
        if data.data == "SJH":
            data.data = "你好啊史佳豪"
        if data.data == "LHB":
            data.data = "你好啊李宏彬"
        return data

def main(args):
    try:
        listener()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."


if __name__ == '__main__':
    main(sys.argv)
