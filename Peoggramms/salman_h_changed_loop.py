#!/usr/bin/env python

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from numpy import array, matrix, cos, sin, pi, arccos, arctan2, sqrt
from numpy.linalg import inv
import time


def get_Table():

    # Define variables for joint angles
    theta1, theta2, theta3, theta4, theta5, theta6 = 0., 0., 0., 0., 0., 0.
    # Construct DH Table with measurements from 'kr210.urdf.xacro' file
    
    s = {'alpha0':     0,  'a0':      0,  'd1':  0.75,  'theta1':  theta1,
          'alpha1': -pi/2,  'a1':   0.35,  'd2':     0,  'theta2':  theta2,
          'alpha2':     0,  'a2':   1.25,  'd3':     0,  'theta3':  theta3,
          'alpha3': -pi/2,  'a3': -0.054,  'd4':  1.50,  'theta4':  theta4,
          'alpha4':  pi/2,  'a4':      0,  'd5':     0,  'theta5':  theta5,
          'alpha5': -pi/2,  'a5':      0,  'd6':     0,  'theta6':  theta6,
          'alpha6':     0,  'a6':      0,  'd7': 0.303,  'theta7':       0}
    
    return s

def H_Transformation(alpha, a, d, theta):
    
    A = matrix([[            cos(theta),           -sin(theta),           0,             a],
                [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])

    return A

def Rotation_Rx(theta):
    """Define matrix for rotation (roll) about x axis."""
    Rx = matrix([[1,          0,           0],
                 [0, cos(theta), -sin(theta)],
                 [0, sin(theta),  cos(theta)]])
    return Rx


def Rotation_Ry(theta):
    """Define matrix for rotation (pitch) about y axis."""
    Ry = matrix([[cos(theta),  0, sin(theta)],
                 [         0,  1,          0],
                 [-sin(theta), 0, cos(theta)]])
    return Ry


def Rotation_Rz(theta):
    """Define matrix for rotation (yaw) about z axis."""
    Rz = matrix([[cos(theta), -sin(theta), 0],
                 [sin(theta),  cos(theta), 0],
                 [         0,           0, 1]])
    return Rz



def get_Gripper_pose(geometry_msg):
    """
    Extract EE pose from received trajectory pose in an IK request message.
    NOTE: Pose is position (cartesian coords) and orientation (euler angles)
    Docs: https://github.com/ros/geometry/blob/indigo-devel/
          tf/src/tf/transformations.py#L1089
    

"""
    px = geometry_msg.position.x
    py = geometry_msg.position.y
    pz = geometry_msg.position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [geometry_msg.orientation.x, geometry_msg.orientation.y,
                    geometry_msg.orientation.z, geometry_msg.orientation.w])
     
    
    gripper_pose = (px, py, pz)
    orient = (roll, pitch, yaw)

    return gripper_pose, orient

 
def get_R0_EE(gripper_pose):
    """
    Compute EE Rotation matrix w.r.t base frame.
    Computed from EE orientation (roll, pitch, yaw) and describes the
    orientation of each axis of EE w.r.t the base frame
    """
    roll, pitch, yaw = gripper_pose[1]
    # Perform extrinsic (fixed-axis) sequence of rotations of EE about
    # x, y, and z axes by roll, pitch, and yaw radians respectively
    Rzyx = Rotation_Rz(yaw) * Rotation_Ry(pitch) * Rotation_Rx(roll)
    # Align EE frames in URDF vs DH params through a sequence of
    # intrinsic (body-fixed) rotations: 180 deg yaw and -90 deg pitch
    R_corr = Rotation_Rz(pi) * Rotation_Ry(-pi/2)
    # Account for this frame alignment error in EE pose
    R0_EE = Rzyx * R_corr

    return R0_EE


def get_Wc(s, R0_EE, gripper_pose):
    """
    Compute Wrist Center position (cartesian coords) w.r.t base frame.
    Keyword arguments:
    R_ee -- EE Rotation matrix w.r.t base frame
    ee_pose -- tuple of cartesian coords and euler angles describing EE
    Return values:
    Wc -- vector of cartesian coords of WC
    """
    px, py, pz = gripper_pose[0]
    # Define EE position as a vector
    Pxyz = matrix([[px], 
                   [py],
                   [pz]])
    # Get Col3 vector from Rxyz that describes z-axis orientation of EE

    # nx = R0_EE[0, 2]
    # ny = R0_EE[1, 2]
    # nz = R0_EE[2, 2]

    # N_ee = matrix ([[nx], 
     #               [ny],
     #               [nz]])


    N_EE = R0_EE[:, 2]

    
    # WC is a displacement from EE equal to a translation along
    # the EE z-axis of magnitude dG w.r.t base frame (Refer to DH Table)
   #  wcx = px - s['d7']*nx
    # wcy = py - s['d7']*ny
    # wcz = pz - s['d7']*nz
    
    Wc = Pxyz - s['d7']*N_EE

    return Wc


def get_joints1_2_3(s, Wc):
    """
    Calculate joint angles 1,2,3 using geometric IK method.
    NOTE: Joints 1,2,3 control position of WC (joint 5)
    """
    wcx, wcy, wcz = Wc[0], Wc[1], Wc[2]

    # theta1 is calculated by viewing joint 1 and arm from top-down
    theta1 = arctan2(wcy, wcx)

    # theta2,3 are calculated using Cosine Law on a triangle with edges
    # at joints 1,2 and WC viewed from side and
    # forming angles A, B and C repectively



    m = sqrt(wcx**2 + wcy**2)-s['a1']    
    w = arctan2(wcz - s['d1'], m)
    b = sqrt((wcz - s['d1'])**2 + m**2)    
    c = sqrt(s['d4']**2 + s['a3']**2)   
                  

    A = arccos((b**2 + s['a2']**2 - c**2) / (2*b*s['a2']))
    B = arccos((c**2 + s['a2']**2 - b**2) / (2*c*s['a2']))

    theta2 = pi/2 - A - w
    theta3 = pi/2 - B - arctan2(s['a3'], s['d4'])

         
    return theta1, theta2, theta3


def get_joints4_5_6(s, R0_EE, theta1, theta2, theta3):
    """
    Calculate joint Euler angles 4,5,6 using analytical IK method.
    NOTE: Joints 4,5,6 constitute the wrist and control WC orientation
    """
    # Compute individual transforms between adjacent links
    # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))
    T0_1 = H_Transformation(s['alpha0'], s['a0'], s['d1'], s['theta1'])
    T1_2 = H_Transformation(s['alpha1'], s['a1'], s['d2'], s['theta2'])
    T2_3 = H_Transformation(s['alpha2'], s['a2'], s['d3'], s['theta3'])
    
    T0_3 = T0_1 * T1_2 * T2_3            
                
    # Extract rotation components of joints 1,2,3 from their
    # respective individual link Transforms
    R0_1 = T0_1[:3, :3]
    R1_2 = T1_2[:3, :3]
    R2_3 = T2_3[:3, :3]
    # Evaluate the composite rotation matrix fromed by composing
    # these individual rotation matrices
    R0_3 = R0_1 * R1_2 * R2_3

    # R3_6 is the composite rotation matrix formed from an extrinsic
    # x-y-z (roll-pitch-yaw) rotation sequence that orients WC
    #R3_6 = T0_3[:3, :3].transpose() * R0_EE  # b/c R0_6 == R_ee = R0_3*R3_6
    R3_6 = inv(array(R0_3, dtype='float')) * R0_EE

    r21 = R3_6[1, 0]  # sin(theta5)*cos(theta6)
    r22 = R3_6[1, 1]  # -sin(theta5)*sin(theta6)
    r13 = R3_6[0, 2]  # -sin(theta5)*cos(theta4)
    r23 = R3_6[1, 2]  # cos(theta5)
    r33 = R3_6[2, 2]  # sin(theta4)*sin(theta5)

    # Compute Euler angles theta 4,5,6 from R3_6 by individually
    # isolating and explicitly solving each angle
    angles_pre = (0,0,0,0,0,0)
    if np.abs(r23) is not 1:
       theta5 = arctan2(sqrt(r13**2 + r33**2), r23)
       if sin(theta5) < 0:
           theta4 = arctan2(-r33, r13)
           theta6 = arctan2(r22, -r21)
       else:
           theta4 = arctan2(r33, -r13)
           theta6 = arctan2(-r22, r21)
    else:
        theta6 = angles_pre[5]
        if r23 == 1:
            theta5 = 0
            theta4 = -theta6 + arctan2(-r12, -r32)
       else:
            theta5 = 0
            theta4 = q6 - arctan2(r12, -r32)
           
    return theta4, theta5, theta6


def handle_calculate_IK(req):
    """Handle request from a CalculateIK type service."""
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        s = get_Table()
        # Initialize service response consisting of a list of
        # joint trajectory positions (joint angles) corresponding
        # to a given gripper pose
        joint_trajectory_list = []

        # To store coordinates for plotting (in plot_ee() function)
        #received_ee_points = []
        #fk_EE_points = []
        #EE_errors = []

        # For each gripper pose a response of six joint angles is computed
         loop_start_time = time.time()
         len_poses = len(req.poses)
        for x in xrange(0, len_poses):
            loop_current_time = time.time()
            joint_trajectory_point = JointTrajectoryPoint()

            # INVERSE KINEMATICS
            gripper_pose = get_Gripper_pose(req.poses[x])

            #received_ee_points.append(ee_pose[0])

            R0_EE = get_R0_EE(gripper_pose)
            Wc = get_Wc(s, R0_EE, gripper_pose)

            # Calculate angles for joints 1,2,3 and update dh table
            theta1, theta2, theta3 = get_joints1_2_3(s, Wc)
            s['theta1'] = theta1
            s['theta2'] = theta2-pi/2  # account for 90 deg constant offset
            s['theta3'] = theta3

            # Calculate angles for joints 4,5,6 and update dh table
            theta4, theta5, theta6 = get_joints4_5_6(s, R0_EE, theta1, theta2, theta3)
            s['theta4'] = theta4
            s['theta5'] = theta5
            s['theta6'] = theta6

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3,
                                                theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            def calculate_FK():
                """Calculate Forward Kinematics for verifying joint angles."""
                # Compute individual transforms between adjacent links
                # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))


                T0_1 = H_Transformation(s['alpha0'], s['a0'], s['d1'], s['theta1'])
                T1_2 = H_Transformation(s['alpha1'], s['a1'], s['d2'], s['theta2'])
                T2_3 = H_Transformation(s['alpha2'], s['a2'], s['d3'], s['theta3'])
                T3_4 = H_Transformation(s['alpha3'], s['a3'], s['d4'], s['theta4'])
                T4_5 = H_Transformation(s['alpha4'], s['a4'], s['d5'], s['theta5'])
                T5_6 = H_Transformation(s['alpha5'], s['a5'], s['d6'], s['theta6'])
                T6_EE = H_Transformation(s['alpha6'], s['a6'], s['d7'], s['theta7'])


                
                # Create overall transform between base frame and EE by
                # composing the individual link transforms
                T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
                fk_EE = [T0_EE[0, 3], T0_EE[1, 3], T0_EE[2, 3]]
                fk_EE_points.append([(fk_EE[0].item(0)),
                                     (fk_EE[1].item(0)),
                                     (fk_EE[2].item(0))])
                error_x = abs(fk_EE[0] - EE_pose[0][0])
                error_y = abs(fk_EE[1] - EE_pose[0][1])
                error_z = abs(fk_EE[2] - EE_pose[0][2])
                EE_errors.append([(error_x.item(0)),
                                  (error_y.item(0)),
                                  (error_z.item(0))])
            # NOTE: Uncomment following line to compute FK for plotting EE
            #calculate_FK()

        print "Total time:", round(time.time() - loop_start_time, 4)

        rospy.loginfo("Number of joint trajectory points:" +
                      " %s" % len(joint_trajectory_list))


        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    """Initialize IK_server ROS node and declare calculate_ik service."""
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
