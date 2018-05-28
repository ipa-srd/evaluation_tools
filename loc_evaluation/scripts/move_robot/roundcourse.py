#!/usr/bin/python
import rospy
from simple_script_server import *

class roundcourse_run():
    def __init__(self):
        self.sss = simple_script_server()
        self.station0 = [0, 0, 0.0]
        self.station1 = [22, 0, 0.0]
        self.station2 = [22, 26, 0.0]
        self.station3 = [0, 26, 0]
        #self.station4 = [-9.2, -37, 0]
        #self.station5 = self.station3
        #self.station6 = self.station0
        self.goals = []
        self.goals.append(self.station0)
        self.goals.append(self.station1)
        self.goals.append(self.station2)
        self.goals.append(self.station3)
        #self.goals.append(self.station4)
        #self.goals.append(self.station5)
        #self.goals.append(self.station6)

    def main(self):

        i = 0
        for goal in self.goals:
            print 'go to station', i 
            self.sss.move("base", goal)
            i += 1
        

if __name__ == '__main__':
    rospy.init_node("roundcourse")
    rr = roundcourse_run()
    num_rounds = 20
    j = 0
    while j < num_rounds:
        rr.main()
        j += 1
