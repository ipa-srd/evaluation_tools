#!/usr/bin/env python
import sys
import rospy
import yaml
import psutil
import os
import rospkg
import optparse
import tf
import math
import PyKDL
import numpy as np
from nav_msgs.msg import Odometry

class PerformanceMeasure():
    def __init__(self):
        proc_name = rospy.get_param("~process_name", "")
        self.process = self.get_process(proc_name)
        self.count = 0
        self.curr_cpu = 0
        self.cpu_sum = 0
        self.mem_sum = 0

    def get_process(self, name):
        proc_id = 0
        for proc in psutil.process_iter():
            try:
                if name in proc.name():
                    proc_id = proc.pid
            except psutil.NoSuchProcess:
                pass
        if proc_id == 0:
            rospy.logerr("No process named " + name)
        else:
            rospy.loginfo("looking after CPU percentage of process " + name + " with id " + str(proc_id))
        process = psutil.Process(proc_id)
        return process

    def update(self):
        self.count +=  1
        self.curr_cpu = self.process.cpu_percent()
        self.cpu_sum += self.curr_cpu
        self.mem_sum += self.process.memory_percent()

        avg_cpu = self.cpu_sum/self.count
        avg_mem = self.mem_sum/self.count
        if (self.count%4 == 0):
            rospy.loginfo("Current CPU percentage is: " +  str(self.curr_cpu))
            rospy.loginfo("Average CPU percentage is: " + str(avg_cpu))
            rospy.loginfo("Average Memory percentage is: " + str(avg_mem))

    def get_avg_cpu_load(self):
        return self.cpu_sum/self.count

class LocErrorMeasure:
    def __init__(self):
        self.root_frame = rospy.get_param("~root_frame", "map")
        self.measured_frame = rospy.get_param("~measured_frame", "base_link")
        self.groundtruth_topic = rospy.get_param("~groundtruth_topic", "base_pose_ground_truth")
        print "gttopic:", self.groundtruth_topic

        self.freq = 5

        self.delta_trans = []
        self.delta_rot =  []
        self.max_pos_error = rospy.get_param("~max_pos_error", 2.0)
        self.max_ang_error = rospy.get_param("~max_ang_error", 2.0)
        self.fails = 0
        self.fail_time = rospy.get_time()
        self.fail_timeout = 5 #sec to wait until new fail is counted
        self.count = 0
        self.listener = tf.TransformListener()
        self.latest_gt_time = rospy.get_time()

        self.sub = rospy.Subscriber(self.groundtruth_topic, Odometry, self.gt_callback)

    def gt_callback(self, msg):
        if (rospy.get_time() - self.latest_gt_time) < (1. / self.freq):
            return
        self.latest_gt_time = rospy.get_time()

        trans_gt = []
        rot_loc = PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1)
        rot_gt = PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1)
        try:
            self.listener.waitForTransform(self.root_frame,
                                           self.measured_frame,
                                           msg.header.stamp,
                                           rospy.Duration(0.2))
            (trans_loc, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        trans_gt.append(msg.pose.pose.position.x)
        trans_gt.append(msg.pose.pose.position.y)
        rot_gt = PyKDL.Rotation.Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        rot_loc = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
        delta_trans = math.sqrt((trans_gt[0] - trans_loc[0])**2 + (trans_gt[1] - trans_loc[1])**2)
        self.delta_trans.append(delta_trans)
        delta_rot = rot_gt * rot_loc.Inverse()
        delta_yaw = abs(delta_rot.GetRPY()[2] * 180/math.pi)
        self.delta_rot.append(delta_yaw)

        self.count = self.count + 1
        if delta_trans > self.max_pos_error: #or delta_rot > self.max_ang_error:
            if rospy.get_time() > self.fail_time + self.fail_timeout:
                self.fails += 1
                self.fail_time = rospy.get_time()

        self.count += 1
        if (self.count%4 == 0):
            rospy.loginfo("Pos error: " +  str(delta_trans))
            rospy.loginfo("Ang error: " + str(delta_yaw))
            rospy.loginfo("Loc fail count:" + str(self.fails))

    def get_result(self):
        data = {}
        data["trans"] = self.delta_trans
        data["rot"] = self.delta_rot
        if len(data["trans"]) > 0:
            data["max_trans"] = max(data["trans"])
            data["max_rot"] = max(data["rot"])
        #data["avg_trans"] = np.mean(data["trans"])
        #data["avg_rot"] = np.mean(data["rot"])
        data["loc_fails"] = self.fails
        return data

if __name__ == '__main__':
    rospy.init_node('loc_performance_observer')

    log_path = rospy.get_param("~log_path", "~")
    file_name = rospy.get_param("~file_name", "")

    rospack = rospkg.RosPack()
    rospack.list()
    pkg_path =rospack.get_path("loc_evaluation")
    data_path = pkg_path + "/data/"
    file_path = data_path + file_name + '.yaml'

    if os.path.exists(file_path):
        os.remove(file_path)
        rospy.logdebug("Removed log file")

    rate = rospy.Rate(1)

    pm = PerformanceMeasure()
    lem = LocErrorMeasure()
    while not rospy.is_shutdown():
        pm.update()
        rate.sleep()
        avg_cpu = pm.get_avg_cpu_load()
        data = lem.get_result()
        data.update({"cpu_load":avg_cpu})
        with open(file_path, 'w+') as f:
            yaml.dump(data, f)
