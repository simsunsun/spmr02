#! /usr/bin/env python
from __future__ import division
from ast import AsyncFunctionDef

import rospy
import std_msgs.msg
from math import pi, asin, acos, trunc
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from std_msgs.msg import Float32, Bool

from diff_drive import logW as log
from diff_drive import env

import math
import heapq

class NextGoalNode:
    def __init__(self):

        # self.s_start = (10*10, 30*10)
        # self.s_goal = (10*10, 10*10)

        self.target_x = 0
        self.target_y = 0
        self.yaw = 0
        self.atGoal = 0
        self.d = 100

        self.heuristic_type = "euclidean"
        self.Env = env.Env()  # class Env
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.e = 2.5

        self.g = dict()

        self.g = dict()                # Cost to come
        self.OPEN = dict()             # priority queue / OPEN set
        self.CLOSED = set()            # CLOSED set
        self.INCONS = {}               # INCONSISTENT set
        self.PARENT = dict()           # relations
        self.path = []                 # planning path
        self.visited = []              # order of visited nodes

        self.position = []
        self.position_seed = []

    def inGoal (self, meg):
        self.atGoal = meg
        # print(self.atGoal, type(self.atGoal))
        return self.atGoal

    def distance (self, meg):
        self.d = meg.data
        # print(self.d, type(self.d))
        return self.d

    # def on_odometry(self, newPose):
    #     self.pose = self.get_angle_pose(newPose.pose.pose)

    def init_postion(self):
        self.target_x = 0
        self.target_y = 0
        self.yaw = 0

    def Goal_position(self, start_x, start_y, goal_x, goal_y):
        self.s_start = (start_x*20, start_y*20)
        self.s_goal = (goal_x*20, goal_y*20)


    def send_Next_position(self, quat, desiredx, desiredy):
        # create PoseStamped
        next_quat = PoseStamped()
        next_quat.header = std_msgs.msg.Header()
        next_quat.header.stamp = rospy.Time.now()
        next_quat.header.frame_id = "base_link" 
        # next_quat.header.frame_id = "" 
        next_quat.pose = Pose()
        next_quat.pose.position.x    = desiredx 
        next_quat.pose.position.y    = desiredy 
        next_quat.pose.position.z    = 0 
        next_quat.pose.orientation.x = quat[0] 
        next_quat.pose.orientation.y = quat[1] 
        next_quat.pose.orientation.z = quat[2] 
        next_quat.pose.orientation.w = quat[3] 
        # send PoseStamped
        # print(next_quat)
        self.Next_poi.publish(next_quat)

    def main(self):
        rospy.init_node('Path_Finder')

        self.Next_poi = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

        rospy.Subscriber('goal_achieved', Bool, self.inGoal)
        rospy.Subscriber('diff_drive_go_to_goal/distance_to_goal', Float32, self.distance)

        self.node_name = rospy.get_name()
        rospy.loginfo("{0} started".format(self.node_name))
        rate = rospy.get_param('~rate', 10)
        self.rate = rospy.Rate(rate)

        self.init_postion()
        
        # i=0
        for i in range(0, 1):
            self.move(self.position)
            # self.move(self.position_back)
            # i = i + 1
            # print("No. : ", i)
        # print("check -main-")
        # rospy.spin()     ==================================================
    
    def move(self, position):
        
        for pose in position:
            x = pose[0]
            y = pose[1]
            self.yaw = math.atan2(y-self.target_y, x-self.target_x)
            angle = [0, 0, self.yaw]
            quat = quaternion_from_euler(angle[0],angle[1],angle[2])
            self.target_x = -x
            self.target_y = y
            for i in range(0, 2):
                self.send_Next_position(quat, -x, y)
                self.rate.sleep()
                
            print("move!!!!  x, y : ", -x," , ",y)
            
            log.log_write("{0} {1} {2} {3}\n".format(rospy.get_time(),self.d, -x, y))

            while True:
                # rospy.loginfo('wait')
                
                if 1.0 >= self.d:                                # set
                    # print("distance :" , self.d)
                    break
                self.rate.sleep()

    def init(self):
        """
        initialize each set.
        """

        self.g[self.s_start] = 0.0
        self.g[self.s_goal] = math.inf
        self.OPEN[self.s_start] = self.f_value(self.s_start)
        self.PARENT[self.s_start] = self.s_start

    def searching(self):
        self.init()
        self.ImprovePath()
        self.path.append(self.extract_path())

        # while self.update_e() > 1:                                          # continue condition
        #     self.e -= 0.4                                                   # increase weight
        #     self.OPEN.update(self.INCONS)
        #     self.OPEN = {s: self.f_value(s) for s in self.OPEN}             # update f_value of OPEN set

        #     self.INCONS = dict()
        #     self.CLOSED = set()
        #     self.ImprovePath()                                              # improve path
        #     self.path.append(self.extract_path())

        # print("path : ", self.path)
        for i in self.path:
            i.reverse()
            # print("check reversed : ", i)
            for i_ in i:
                i = list(i_)
                # print("..........", i)
                some_list = []
                for ii in i:
                    li = ii/20
                    some_list.append(li)
                    some_list.reverse()
                self.position_seed.append(some_list)

#  **************smooth*********************************************

        i = 0; a = 1; seed_list = []
        seed_x  = self.position_seed[i][0]
        seed_y  = self.position_seed[i][1]
        seed_list = [ seed_x , seed_y ]

        self.position.append(seed_list)

        while self.position_seed:
            
            try:
                if self.position_seed[i+a][0] == None:
                    print("out")
            except:
                break

            seed_xx = self.position_seed[i+a][0]
            seed_yy = self.position_seed[i+a][1]
            xyl = math.sqrt(math.pow(seed_xx - seed_x , 2) +
                            math.pow(seed_yy - seed_y , 2))
            # print("checkcheckcheckcheck : " ,xyl)
            while xyl < 0.5:
                # print("check : " ,xyl)

                try:
                    if self.position_seed[i+a][0] == None:
                        print("out")
                except:
                    break

                seed_xx = self.position_seed[i+a][0]
                seed_yy = self.position_seed[i+a][1]
                xyl = math.sqrt(math.pow(seed_xx - seed_x , 2) +
                                math.pow(seed_yy - seed_y , 2))
                seed_list_add = [seed_xx,seed_yy]
                a = a + 1
                # print("a : ", a)
                # print("i : ", i)
            
            try:
                self.position.append(seed_list_add)
                i = i + a; a = 1
                seed_x  = self.position_seed[i][0]
                seed_y  = self.position_seed[i][1]
            except:
                break

#  ***********************************************************

        # print("position_seed : ", self.position_seed,type(self.position_seed))
        print("position : ", self.position,type(self.position))

        return self.path, self.visited, self.position

# 5 10 5 27
# 5 27 7 27
# 7 27 15 27
# 15 27 20 27
# 20 27 20 20
# 20 20 14 10
# 14 10 14 5
# 14 5 23 5
# 23 5 5 10

# 5 10 20 24
# 20 24 5 10


    def ImprovePath(self):
        """
        :return: a e'-suboptimal path
        """

        visited_each = []

        while True:
            s, f_small = self.calc_smallest_f()

            if self.f_value(self.s_goal) <= f_small:
                break

            self.OPEN.pop(s)
            self.CLOSED.add(s)

            for s_n in self.get_neighbor(s):
                if s_n in self.obs:
                    continue

                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g or new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    visited_each.append(s_n)

                    if s_n not in self.CLOSED:
                        self.OPEN[s_n] = self.f_value(s_n)
                    else:
                        self.INCONS[s_n] = 0.0

        self.visited.append(visited_each)

    def calc_smallest_f(self):
        """
        :return: node with smallest f_value in OPEN set.
        """

        s_small = min(self.OPEN, key=self.OPEN.get)

        return s_small, self.OPEN[s_small]

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return {(s[0] + u[0], s[1] + u[1]) for u in self.u_set}

    def update_e(self):
        v = float("inf")

        if self.OPEN:
            v = min(self.g[s] + self.h(s) for s in self.OPEN)
        if self.INCONS:
            v = min(v, min(self.g[s] + self.h(s) for s in self.INCONS))

        return min(self.e, self.g[self.s_goal] / v)

    def f_value(self, x):
        """
        f = g + e * h
        f = cost-to-come + weight * cost-to-go
        :param x: current state
        :return: f_value
        """

        return self.g[x] + self.e * self.h(x)

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = self.PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def h(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type                                # heuristic type
        goal = self.s_goal                                                  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

# ===================round r===========================================
        s_1_1=list(s_start) ; s_1_2=list(s_end)
        s_2_1=list(s_start) ; s_2_2=list(s_end)
        s_3_1=list(s_start) ; s_3_2=list(s_end)
        s_4_1=list(s_start) ; s_4_2=list(s_end)
        s_5_1=list(s_start) ; s_5_2=list(s_end)
        s_6_1=list(s_start) ; s_6_2=list(s_end)
        s_7_1=list(s_start) ; s_7_2=list(s_end)
        s_8_1=list(s_start) ; s_8_2=list(s_end)
        
        obco = 20

        self.obs2=list(self.obs)
        s1_1_tu=(s_1_1[0]-obco,s_1_1[1])    
        s2_1_tu=(s_2_1[0]-obco,s_2_1[1]+obco)  
        s3_1_tu=(s_3_1[0],s_3_1[1]+obco)    
        s4_1_tu=(s_4_1[0]+obco,s_4_1[1]+obco)  
        s5_1_tu=(s_5_1[0]+obco,s_5_1[1])    
        s6_1_tu=(s_6_1[0]+obco,s_6_1[1]-obco)  
        s7_1_tu=(s_7_1[0],s_7_1[1]-obco)    
        s8_1_tu=(s_8_1[0]-obco,s_8_1[1]-obco) 



        if s1_1_tu in self.obs or s2_1_tu in self.obs or s3_1_tu in self.obs or s4_1_tu in self.obs or\
            s5_1_tu in self.obs or s6_1_tu in self.obs or s7_1_tu in self.obs or s8_1_tu in self.obs:
                return True
# ===============================================================================

# ===============round r ignore==============================================
        # if s_start in self.obs or s_end in self.obs:
        #     return True

        # if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
        #     if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
        #         s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
        #         s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
        #     else:
        #         s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
        #         s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

        #     if s1 in self.obs or s2 in self.obs:
        #         return True
# ===============================================================================

        return False

def node_move(a, b, c, d):
    node = NextGoalNode()
    node.Goal_position(b, a, d, c)
    node.searching()
    node.main()

if __name__ == '__main__':
    try:
        for i in range(0, 100000001):
            position = open("/home/tang/catkin_ws/src/diff_drive/nodes/Goal_position.txt", 'r')
            no = 0
            for line in position:
                no = no + 1
                line = line.replace("\n", "")
                line = list(line.split(" "))
                lline = []
                for i in line:
                    i = float(i)
                    lline.append(i)
                # print("start and Goal position :", lline)
                print("NO : ", no, "=> ")
                node_move(lline[0], lline[1], lline[2], lline[3])
            position.close()
        print("END")
    except rospy.ROSInterruptException:
        pass
