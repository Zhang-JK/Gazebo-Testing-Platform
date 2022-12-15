import rospy

import tf
from nav_msgs.msg import Odometry

def handle_odom(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     msg.child_frame_id,
                     "odom")

if __name__ == '__main__':
    rospy.init_node('odom2tf')
    rospy.Subscriber('/real_pos', Odometry, handle_odom)
    rospy.spin()