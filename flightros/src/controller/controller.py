#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class Controller:
    def __init__(self):
        self._pub = None
        
    def run(self):
        self._pub = rospy.Publisher('cmd', String, queue_size=1)
        rospy.Subscriber("rgb", String, self.img_callback)
        rospy.spin()

    def img_callback(self, msg):
        self._pub.publish("test")

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    c = Controller()
    c.run()
