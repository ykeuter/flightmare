#!/usr/bin/env python
import rospy
import math

class Trainer:
    EPOCH_T = 5

    def __init__(self):
        self._state_sub = None
        self._reset_pub = None
        self._geno_type_pub = None
        self._is_resetting = False
        self._population = None
        self._current_reward = 0
        self._current_agent = None

    def _reset(self):
        self._is_resetting = True
        # set reward
        self._current_agent = self._get_next_agent()
        self._current_reward = 0
        # reset simulator
        # reset controller

    def _get_next_agent(self):
        pass

    def _next_generation(self):
        pass

    def state_callback(self, msg):
        if self._is_resetting:
            return
        # update reward
        # if T > EPOCh_T
        # reset

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


if __name__ == '__main__':
    rospy.init_node('trainer', anonymous=True)
    t = Trainer()
    t.run()
