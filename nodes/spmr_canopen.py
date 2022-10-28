#! /usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

import src

from diff_drive import controller

class ControllerNode:

    def __init__(self):
        self.controller = controller.Controller()
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0

    def main(self):
        self.leftPub = rospy.Publisher('lwheel_desired_rate',
                                       Int32, queue_size=1)
        self.rightPub = rospy.Publisher('rwheel_desired_rate',
                                        Int32, queue_size=1)

        rospy.init_node('diff_drive_canopen')
        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        rospy.Subscriber("/test_mr/cmd_val", Twist, self.twistCallback)

        self.ticksPerMeter = float(rospy.get_param('~ticks_per_meter', 76.433))
        self.wheelSeparation = float(rospy.get_param('~wheel_separation', 0.566))
        self.maxMotorSpeed = int(rospy.get_param('~max_motor_speed', 1000))
        self.rate = float(rospy.get_param('~rate', 10))
        self.timeout = float(rospy.get_param('~timeout', 0.5))

        self.controller.setWheelSeparation(self.wheelSeparation)
        self.controller.setTicksPerMeter(self.ticksPerMeter)
        self.controller.setMaxMotorSpeed(self.maxMotorSpeed)

        rate = rospy.Rate(self.rate)
        self.lastTwistTime = rospy.get_time()

        while not rospy.is_shutdown():
            print("check")
            self.publish()
            rate.sleep()


    def publish(self):

        if rospy.get_time() - self.lastTwistTime < self.timeout:
            speeds = self.controller.getSpeeds(self.linearVelocity, self.angularVelocity)

            print("left", speeds.left)
            print("right", speeds.right)

            self.leftPub.publish(int(speeds.left))
            self.rightPub.publish(int(speeds.right))

            src.LW_Move(speeds.left / 2)
            src.RW_Move(speeds.right / 2)
            # src.Monitor_Val()


        else:
            self.leftPub.publish(0)
            self.rightPub.publish(0)
            print('test')


    def twistCallback(self, twist):
        self.linearVelocity = twist.linear.x
        self.angularVelocity = twist.angular.z
        self.lastTwistTime = rospy.get_time()

        # print("input x", twist.linear.x)
        # print("input z", twist.angular.z)

src.Dir_Op()

if __name__ == '__main__':
    try:
        node = ControllerNode()
        # src.Syn_STOP()
        src.Dir_Op()
        node.main()
    except rospy.ROSInterruptException:
        pass
