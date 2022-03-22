#!/usr/bin/env python
import rospy
from flightevo.controller import Controller

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    c = Controller()
    c.run()
