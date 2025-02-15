#!/usr/bin/env python3

from time import sleep
from typing import List, Tuple

import numpy as np
import rospy, time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

from rrp_ik_client import get_rrp_ik
    

ROSPY_RATE = 50


# PID Controller Class
class Controller:
    def __init__(self, p=0.0, i=0.0, d=0.0, set_point=0.0):
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.set_point = set_point
        self.previous_error = 0
        self.prev_time = rospy.get_time()
        self.previous_derivative = 0.0

        self.P_term = 0.0
        self.I_term = 0.0
        self.D_term = 0.0
        self.integral_limit = 10  # To prevent integral windup
        self.alpha = 0.8

    def update(self, current_value):
        
        current_time = rospy.get_time()
        dt = current_time - self.prev_time
        if dt <= 0: 
            return 0.0
        
        error = self.set_point - current_value

        self.P_term = self.Kp * error
        self.I_term += self.Ki * error * dt
        # Anti-windup
        self.I_term = max(min(self.I_term, self.integral_limit), -self.integral_limit)

        # Derivative term with filtering
        raw_derivative = (error - self.previous_error) / dt
        self.D_term = self.Kd * ((1 - self.alpha) * self.previous_derivative + self.alpha * raw_derivative)

        self.previous_error = error
        self.previous_derivative = self.D_term
        self.prev_time = current_time

        return self.P_term + self.I_term + self.D_term

    def update_set_point(self, set_point):
        self.set_point = set_point
        self.previous_error = 0
        self.I_term = 0
        self.prev_time = rospy.get_time()

    def set_pid(self, p=0.0, i=0.0, d=0.0):
        self.Kp = p
        self.Ki = i
        self.Kd = d


class RRP_bot:
    def __init__(self, positions: List[Tuple]):
        rospy.init_node("rrp_bot_move")
        rospy.loginfo("Press Ctrl + C to terminate")

        self.poses = positions
        self.des_states = None
        self.actual_states = []

        self.jnt_pub1 = rospy.Publisher("/rrp/joint1_effort_controller/command", Float64, queue_size=10)
        self.jnt_pub2 = rospy.Publisher("/rrp/joint2_effort_controller/command", Float64, queue_size=10)
        self.jnt_pub3 = rospy.Publisher("/rrp/joint3_effort_controller/command", Float64, queue_size=10)
        self.joint_error_pub = rospy.Publisher("/rrp/joint_error", Point, queue_size=10)

        self.orient_control1 = Controller()
        self.orient_control2 = Controller()
        self.pos_control3 = Controller()

        rospy.Subscriber("/rrp/joint_states", JointState, self.subs_callback)

        self.logging_counter = 0
        self.trajectory1 = []
        self.trajectory2 = []
        self.trajectory3 = []

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            np.savetxt("trajectory1.csv", np.array(self.trajectory1), fmt="%f", delimiter=",")
            np.savetxt("trajectory2.csv", np.array(self.trajectory2), fmt="%f", delimiter=",")
            np.savetxt("trajectory3.csv", np.array(self.trajectory3), fmt="%f", delimiter=",")

    def subs_callback(self, data):
        self.actual_states = data.position

        # Log every 25th data point
        self.logging_counter += 1
        if self.logging_counter >= 25:  
            self.logging_counter = 0
            self.trajectory1.append(self.actual_states[0])
            self.trajectory2.append(self.actual_states[1])
            self.trajectory3.append(self.actual_states[2])

    def control(self, kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3):
        self.orient_control1.set_pid(kp1, ki1, kd1)
        self.orient_control2.set_pid(kp2, ki2, kd2)
        self.pos_control3.set_pid(kp3, ki3, kd3)

        self.orient_control1.update_set_point(self.des_states[0])
        self.orient_control2.update_set_point(self.des_states[1])
        self.pos_control3.update_set_point(self.des_states[2])

        rate = rospy.Rate(ROSPY_RATE)

        while not rospy.is_shutdown():
            if len(self.actual_states) < 3:
                continue  # Skip until we receive valid joint states

            effort1 = self.orient_control1.update(self.actual_states[0])
            effort2 = self.orient_control2.update(self.actual_states[1])
            effort3 = self.pos_control3.update(self.actual_states[2])

            # Store last effort for smooth stopping
            self.jnt_pub1.last_effort = effort1
            self.jnt_pub2.last_effort = effort2
            self.jnt_pub3.last_effort = effort3

            self.jnt_pub1.publish(effort1)
            self.jnt_pub2.publish(effort2)
            self.jnt_pub3.publish(effort3)
            self.joint_error_pub.publish(Point(self.orient_control1.previous_error, self.orient_control2.previous_error, self.pos_control3.previous_error))

            reached_1 = self.is_close("orientation", self.des_states[0], self.actual_states[0])
            reached_2 = self.is_close("orientation", self.des_states[1], self.actual_states[1])
            reached_3 = self.is_close("position", self.des_states[2], self.actual_states[2])

            if reached_1:
                self.stop(self.jnt_pub1)

            if reached_2:
                self.stop(self.jnt_pub2)

            if reached_3:
                self.stop(self.jnt_pub3)

            if reached_1 and reached_2 and reached_3:
                print("All joints reached")
                sleep(1.5)
                break
                
            rate.sleep()

    def run(self):
        print("running rrp control")
        for idx, pose in enumerate(self.poses):
            print(f"goint to the pose no {idx}: {pose}")
            self.des_states = get_rrp_ik(pose[0],pose[1],pose[2])
            self.control(0.25, 0.0129, 0.12, 0.25, 0.0129, 0.12, 5.5, 10, 0.6)
    
    @staticmethod
    def is_close(entity, desired, actual, orient_tol=0.15, pos_tol=0.05):
        error = abs(desired - actual)
        tol = orient_tol if entity == "orientation" else pos_tol
        return True if error < tol else False
    
    @staticmethod
    def stop(jnt_pub):
        jnt_pub.publish(0.0)
        jnt_pub.publish(0.0)
        jnt_pub.publish(0.0)


if __name__ == "__main__":

    positions = [(0.0, 0.77, 0.34), (-0.345, 0.425, 0.24),(0.67, -0.245, 0.14), (0.77, 0.0, 0.39) ]
    RRP_bot(positions)