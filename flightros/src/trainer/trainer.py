#!/usr/bin/env python
import rospy
import os

from flightevo.trainer import Trainer

if __name__ == '__main__':
    cfg = os.path.join(os.path.dirname(__file__), "neat.cfg")
    rospy.init_node('trainer', anonymous=True)
    t = Trainer()
    t.run(cfg)
