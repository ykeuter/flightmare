#!/usr/bin/env python
import rospy
from flightros.msg import Test
import numpy as np
from rospy.numpy_msg import numpy_msg

def callback(data):
    # shape = [d.size for d in data.layout.dim]
    arr = data.weights[0].data
    shape = [d.size for d in data.weights[0].layout.dim]
    arr = arr.reshape(shape)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", arr)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", numpy_msg(Test), callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()