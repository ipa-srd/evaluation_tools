#!/usr/bin/env python
import sys
import rospy

import psutil
import os
import rospkg
from optparse import OptionParser

def getProcessIDs(name):
    proc_id = 0
    for proc in psutil.process_iter():
        try:
            if name in proc.name():
                proc_id = proc.pid
        except psutil.NoSuchProcess:
            pass

    return proc_id

if __name__ == '__main__':
    rospy.init_node('measure_cpu', anonymous=True)

    parser = OptionParser(usage="%prog -p or --process + process name", prog=os.path.basename(sys.argv[0]))

    parser.add_option("-p", "--process", dest="process", default=0, help="Process name")
    parser.add_option("-f", "--file", dest="file", default="cpu_mean", help="file name")
    (options, args) = parser.parse_args()
    if not isinstance(options.process, basestring):
        parser.error("Please specify a process name using -p or --process")
        sys.exit()

    process_ID = getProcessIDs(options.process)
    file = str(options.file)

    rospack = rospkg.RosPack()
    rospack.list()
    path =rospack.get_path("loc_evaluation")
    path = path + "/data/"
    file_path = path + file + '.txt'

    if os.path.exists(file_path):
        os.remove(file_path)
        print "removed output file"

    if process_ID == 0:
        parser.error("No process named " + options.process)
    else:
        print "looking after CPU percentage of Process " + str(options.process) + " with ID " + str(process_ID)

    process = psutil.Process(process_ID)

    count = 0
    cpu = 0
    mem = 0
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rate.sleep()
        curr_cpu = process.cpu_percent()
        cpu = cpu + curr_cpu
        mem = mem + process.memory_percent()
        count = count + 1
        if (count%10 == 0):
            print "current CPU percentage is: " +  str(curr_cpu)
            print "average CPU percentage is: " + str(cpu/count)
            print "average Memory percentage is: " + str(mem/count)
            with open(file_path, 'w+') as f:
                f.write("cpu_mean: "+str(cpu/count)+"\n");
                f.write("mem_mean: "+str(mem/count));

