#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import TransformStamped
from tf import transformations as t

class gt_broadcaster():
    def __init__(self):
        self.sss = rospy.Subscriber("base_pose_ground_truth", Odometry, gt_callback)
        self.tf_broadcaster = f.TransformBroadcaster()


def gt_callback(msg):
    br = tf.TransformBroadcaster()
    trans = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    rot = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
    inv_transform = t.inverse_matrix(transform)
    inv_trans = t.translation_from_matrix(inv_transform)
    inv_rot = t.quaternion_from_matrix(inv_transform)
    br.sendTransform(inv_trans,
                     inv_rot,
                     msg.header.stamp,
                     "map",
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('ground_truth_tf_broadcaster')
    rospy.Subscriber("base_pose_ground_truth", Odometry, gt_callback)
    rospy.spin()

