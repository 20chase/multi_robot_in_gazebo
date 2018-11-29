import rospy
import tf
import time

from nav_msgs.msg import Odometry


class OdomPublisher(object):
    def __init__(self):
        rospy.init_node("pub_odom")
        self.rate = rospy.Rate(30)
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

    def run(self):
        time.sleep(1)
        while not rospy.is_shutdown():
	    pass
#            print self.p_x, " -- ", self.p_y, " -- ", self.o_z, " -- ", self.o_w


    def odom_cb(self, data):
        self.p_x = data.pose.pose.position.x
        self.p_y = data.pose.pose.position.y
        self.o_z = data.pose.pose.orientation.z
        self.o_w = data.pose.pose.orientation.w
        self.br.sendTransform((self.p_x, self.p_y, 0),
                        (0, 0, self.o_z, self.o_w),
                        rospy.Time.now(),
                        "base_footprint",
                        "odom")

if __name__ == "__main__":
    pub = OdomPublisher()
    pub.run()
