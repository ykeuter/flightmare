#!/usr/bin/env python
import rospy
from flightevo.trainer import Trainer

if __name__ == '__main__':
    rospy.init_node('trainer', anonymous=True)
    t = Trainer()
    t.run()
