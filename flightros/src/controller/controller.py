#!/usr/bin/env python
import rospy
from flightros.msg import Cmd
from sensor_msgs.msg import Image

class Controller:
    def __init__(self):
        self._pub = None
        
    def run(self):
        self._pub = rospy.Publisher('cmd', Cmd, queue_size=1)
        rospy.Subscriber("rgb", Image, self.img_callback)
        rospy.spin()

    def img_callback(self, msg):
        rospy.loginfo(
            (
                "height ({}): {}\n" +
                "width ({}): {}\n" +
                "encoding ({}): {}\n" +
                "is_bigendian ({}): {}\n" +
                "step ({}): {}\n" +
                "data ({}): {}\n"
            ).format(
                type(msg.height), msg.height,
                type(msg.width), msg.width,
                type(msg.encoding), msg.encoding,
                type(msg.is_bigendian), msg.is_bigendian,
                type(msg.step), msg.step,
                type(msg.data), max(msg.data)
            )
        )
        self._pub.publish(Cmd(0, [3, 3, 3, 3]))

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    c = Controller()
    c.run()
