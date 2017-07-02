#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

#Define some functions to calculate rotations
def rot_x(q):
    R_x = Matrix([[1, 0     , 0      ],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])
    return R_x

def rot_y(q):
    R_y = Matrix([[cos(q) , 0      , sin(q)],
                  [0      , 1      , 0     ],
                  [-sin(q), 0      , cos(q)]])

    return R_y

def rot_z(q):

    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q) , 0],
                  [0     , 0      , 1]])

    return R_z


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #joint angles
            r, p, y = symbols("r p y")
            q = symbols('q') #Symbol for intrinsic rotations

            # Modified DH params
            s = {alpha0:0    , a0:0     , d1:0.75 ,
                 alpha1:-pi/2, a1:0.35  , d2:0    , q2:q2-pi/2,
                 alpha2:0    , a2:1.25  , d3:0    ,
                 alpha3:-pi/2, a3:-0.054, d4:1.50 ,
                 alpha4:pi/2 , a4:0     , d5:0    ,
                 alpha5:-pi/2, a5:0     , d6:0    ,
                 alpha6:0    , a6:0     , d7:0.303, q7:0}


            ###Homogenous Transforms
            #Link 0 (Base) to Link 1
            t0_1 = Matrix([[cos(q1)            , -sin(q1)           , 0           , a0              ],
                           [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                           [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0), cos(alpha0) , cos(alpha0)*d1 ],
                           [0                  , 0                  , 0           , 1              ]])

            t0_1 = t0_1.subs(s)

            #Link 1 to Link 2
            t1_2 = Matrix([[cos(q2)            , -sin(q2)           , 0           , a1              ],
                           [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                           [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1), cos(alpha1) , cos(alpha1)*d2 ],
                           [0                  , 0                  , 0           , 1              ]])

            t1_2 = t1_2.subs(s)

            #Link 2 to Link 3
            t2_3 = Matrix([[cos(q3)            , -sin(q3)           , 0           , a2              ],
                           [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                           [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2), cos(alpha2) , cos(alpha2)*d3 ],
                           [0                  , 0                  , 0           , 1              ]])

            t2_3 = t2_3.subs(s)

            #Link 3 to Link 4
            t3_4 = Matrix([[cos(q4)            , -sin(q4)           , 0           , a3              ],
                           [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                           [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3), cos(alpha3) , cos(alpha3)*d4 ],
                           [0                  , 0                  , 0           , 1              ]])

            t3_4 = t3_4.subs(s)

            #Link 4 to Link 5
            t4_5 = Matrix([[cos(q5)            , -sin(q5)           , 0           , a4              ],
                           [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                           [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4), cos(alpha4) , cos(alpha4)*d5 ],
                           [0                  , 0                  , 0           , 1              ]])

            t4_5 = t4_5.subs(s)

            #Link 5 to Link 6
            t5_6 = Matrix([[cos(q6)            , -sin(q6)           , 0           , a5              ],
                           [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                           [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5), cos(alpha5) , cos(alpha5)*d6 ],
                           [0                  , 0                  , 0           , 1              ]])

            t5_6 = t5_6.subs(s)

            #Link 6 to Link 7(Gripper)
            t6_g = Matrix([[cos(q7)            , -sin(q7)           , 0           , a6              ],
                           [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                           [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6), cos(alpha6) , cos(alpha6)*d7 ],
                           [0                  , 0                  , 0           , 1              ]])

            t6_g = t6_g.subs(s)

            #Composition of Homogenous Transforms
            t0_2 = t0_1 * t1_2
            t0_3 = t0_2 * t2_3
            t0_4 = t0_3 * t3_4
            t0_5 = t0_4 * t4_5
            t0_6 = t0_5 * t5_6
            t0_g = t0_6 * t6_g

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
            p_wrist = Matrix([[px],[py],[pz]])
            R0_g = simplify(rot_x(r) * rot_y(p) * rot_z(y))
            R0_g = R0_g.evalf(subs={r: roll, p: pitch, y: yaw})
            Wc = simplify(p_wrist - 0.303 * R0_g * Matrix([[1],[0],[0]]))

            xc = Wc[0,0]
            yc = Wc[1,0]
            zc = Wc[2,0]

            print("")
            print("")
            print("")
            print('Wrist center is ', Wc)

            joint_angle_1 = atan2(yc,xc)
            #joint_angle_2 = atan2(sqrt(xc**2 + yc**2),zc-0.75)
            #joint_angle_3 = atan2(sqrt(xc**2 + yc**2),zc)

            #########################################################
            #TODO ADDED SECOND WAY TO CALCULATE ANGLES TWO AND THREE#
            #########################################################
            armangle = atan2(zc-(1.94645), xc)
            xc = xc-0.054*(sin(armangle))
            zc = zc+0.054*(cos(armangle))
            # determine the length from origin to wrist centre looking top down to determine the X value for theta 2
            pxtheta = sqrt(yc*yc+xc*xc)
            # link 1 length is 1.25 link 2 length is 1.500
            l1 = 1.25 # a2
            l2 = 1.5 # d4
            # set joint 2 centre for theta 2 and 3 calculations
            pxtheta = pxtheta-0.35
            pztheta = zc - 0.75
            ## Calculateangle theta2 using law of cosines
            D=(pxtheta*pxtheta + pztheta*pztheta - l1*l1-l2*l2)/(2*l1*l2)
            # calculate theta 2
            theta2 = atan2(-sqrt(1-D*D),D)
            joint_angle_3= theta2
            # Using theta 2, calculate theta 1
            S1=((l1+l2*cos(theta2))*pztheta-l2*sin(theta2)*pxtheta)/(pxtheta*pxtheta + pztheta*pztheta)
            C1=((l1+l2*cos(theta2))*pxtheta+l2*sin(theta2)*pztheta)/(pxtheta*pxtheta + pztheta*pztheta)
            theta1=atan2(S1,C1)
            joint_angle_2 = theta1
            ############################################################################################
            ############################################################################################

            print("")
            print("")
            print('Joint 1 angle is ',joint_angle_1)
            print("")
            print('Joint 2 angle is ',joint_angle_2)
            print("")
            print('Joint 3 angle is ',joint_angle_3)

            #Correction rotation for joint angles 1- 3
            #joint_angle_3 = - (joint_angle_3 + pi/2)
            #joint_angle_2 = pi/2 - joint_angle_2 
                                    
            R0_3 = t0_3.extract([0,1,2],[0,1,2])
            R0_3 = R0_3.evalf(subs={q1:joint_angle_1, q2:joint_angle_2, q3:joint_angle_3})

            R3_6 = R0_3.inv() * R0_g

            ##Using R3_6 calculate the Eulers angles
            #Find important r:
            r21 = R3_6[1,0]
            r11 = R3_6[0,0]
            r31 = R3_6[2,0]
            r32 = R3_6[2,1]
            r33 = R3_6[2,2]

            #Calculate the angles
            ###TODO ADDED -PI/2 TO 6 AND 5
            joint_angle_6 = atan2(r21, r11) # rotation about Z-axis in radians (joint 6)
            joint_angle_5  = atan2(-r31, sqrt(r11*r11+r21*r21)) # rotation about Y-axis in radians (joint 5)
            joint_angle_4 = atan2(r32, r33) - pi/2 # rotation about X-axis in radians (joint 4)

            #TODO one angle is causing a collition
            print("")
            print("")
            print('Joint 1 angle is ',joint_angle_1)
            print("")
            print('NEW Joint 2 angle is ',joint_angle_2)
            print("")
            print('NEW Joint 3 angle is ',joint_angle_3)
            print("")
            print('Joint 4 angle is ',joint_angle_4)
            print("")
            print('Joint 5 angle is ',joint_angle_5)
            print("")
            print('Joint 6 angle is ',joint_angle_6)


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [joint_angle_1, joint_angle_2, joint_angle_3,
                                            joint_angle_4, joint_angle_5, joint_angle_6]
	    joint_trajectory_list.append(joint_trajectory_point)
        
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
