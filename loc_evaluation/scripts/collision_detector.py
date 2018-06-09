#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
import numpy as np
import tf
from geometry_msgs.msg import TransformStamped
from tf import transformations as t

class CollisionDetector():
    def __init__(self):
        self.gt_sub1 = rospy.Subscriber("/rob_1/base_pose_ground_truth", Odometry, self.gt_callback1)
        self.gt_sub2 = rospy.Subscriber("/rob_2/base_pose_ground_truth", Odometry, self.gt_callback2)
        self.gt_sub3 = rospy.Subscriber("/rob_3/base_pose_ground_truth", Odometry, self.gt_callback3)
        self.gt_sub4 = rospy.Subscriber("/rob_4/base_pose_ground_truth", Odometry, self.gt_callback4)
        self.rob_poses = [None, None, None, None]
        self.coll_dist = 2 * 0.62
        self.crit_coll_dist = 0.7

    def gt_callback1(self, msg):
        self.rob_poses[0] = msg
        self.checkCollision(0)

    def gt_callback2(self, msg):
        self.rob_poses[1] = msg
        self.checkCollision(1)

    def gt_callback3(self, msg):
        self.rob_poses[2] = msg
        self.checkCollision(2)

    def gt_callback4(self, msg):
        self.rob_poses[3] = msg
        self.checkCollision(3)

    def checkCollision(self, i):
        #print range(len(self.rob_poses))
        for j in range(len(self.rob_poses)):
            #print i, j
            #print self.rob_poses[j]
            if j != i and self.rob_poses[j] is not None:
                #print i, j
                d = self.getDistance(self.rob_poses[i], self.rob_poses[j])
                rospy.logdebug('rob ' + str(i) + " : " + str(j) + ", d:" + str(d) + 'time:' + str(rospy.get_time()))
                if d<= self.crit_coll_dist:
                    rospy.logerr('Coll rob ' + str(i) + " : " + str(j) + ", d:" + str(d) + ', time:' + str(rospy.get_time()))
                elif d <= self.coll_dist:
                    rospy.logwarn('Coll rob ' + str(i) + " : " + str(j) + ", d:" + str(d) + ', time:' + str(rospy.get_time()))

    def getDistance(self, rob1, rob2):
        (x1, y1) = self.getPosFromOdom(rob1)
        (x2, y2) = self.getPosFromOdom(rob2)
        d = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return d

    def getPosFromOdom(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        return (x,y)


if __name__ == '__main__':
    rospy.init_node('collision_detector')
    cd = CollisionDetector()
    rospy.spin()
