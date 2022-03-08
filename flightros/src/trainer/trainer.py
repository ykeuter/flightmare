#!/usr/bin/env python
import rospy
from flightros.msg import State
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Twist

class Trainer:
    EPOCH_LENGTH = 5

    def __init__(self):
        self._reset_sim = None
        self._is_resetting = False
        self._population = None
        self._current_reward = 0
        self._current_agent = None
        self._last_time = 0

    def run(self):
        rospy.wait_for_service('add_two_ints')
        self._reset_sim = rospy.ServiceProxy('reset_sim', ResetSim)
        rospy.Subscriber("state", State, self.state_callback)
        rospy.spin()

    def _reset(self):
        self._is_resetting = True
        # set reward
        self._current_agent = self._get_next_agent()
        self._current_reward = 0
        # reset controller
        try:
            self._reset_sim()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def _gen_new_state(self):
        return ResetSim.Request(
            Pose(Point(0, 0, 7), Quaternion(1, 0, 0, 0)),
            Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        )

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


if __name__ == '__main__':
    rospy.init_node('trainer', anonymous=True)
    t = Trainer()
    t.run()
