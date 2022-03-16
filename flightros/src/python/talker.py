#!/usr/bin/env python
# license removed for brevity
import rospy
from flightros.msg import Test
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from rospy.numpy_msg import numpy_msg
import numpy as np

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', numpy_msg(Test), queue_size=10)
    rate = rospy.Rate(1) # 1hz
    arr = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
    while not rospy.is_shutdown():
        m1 = Float32MultiArray(
            data=arr.reshape(-1),
            layout=MultiArrayLayout(
                dim=[MultiArrayDimension(size=s) for s in arr.shape]
            )
        )
        t = Test()
        t.weights = (m1, )
        pub.publish(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass