from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    
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
    theta4 = arctan2(r33, -r13)
    theta5 = arctan2(sqrt(r13**2 + r33**2), r23)
    theta6 = arctan2(-r22, r21)

    return theta4, theta5, theta6

def handle_IK():

    s = get_Table()
    
    gripper_pose = get_Gripper_pose(req.poses[x])
    R0_EE = get_R0_EE(gripper_pose)
    Wc = get_Wc(s, R0_EE, gripper_pose)
    theta1, theta2, theta3 = get_joints1_2_3(s, Wc)
    theta4, theta5, theta6 = get_joints4_5_6(s, R0_EE, theta1, theta2, theta3)

    return Wc, theta1, theta2, theta3, theta4, theta5, theta6


    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    Wc, theta1, theta2, theta3, theta4, theta5, theta6 = handle_IK()
    s = get_Table()
    
    s['theta1'] = theta1
    s['theta2'] = theta2-pi/2  # account for 90 deg constant offset
    s['theta3'] = theta3
    s['theta4'] = theta4
    s['theta5'] = theta5
    s['theta6'] = theta6


    T0_1 = H_Transformation(s['alpha0'], s['a0'], s['d1'], s['theta1'])
    T1_2 = H_Transformation(s['alpha1'], s['a1'], s['d2'], s['theta2'])
    T2_3 = H_Transformation(s['alpha2'], s['a2'], s['d3'], s['theta3'])
    T3_4 = H_Transformation(s['alpha3'], s['a3'], s['d4'], s['theta4'])
    T4_5 = H_Transformation(s['alpha4'], s['a4'], s['d5'], s['theta5'])
    T5_6 = H_Transformation(s['alpha5'], s['a5'], s['d6'], s['theta6'])
    T6_EE = H_Transformation(s['alpha6'], s['a6'], s['d7'], s['theta7'])

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
    your_wc = [Wc[0], Wc[1], Wc[2]]                    # <--- Load computed Wc value
    your_ee = [T0_EE[0, 3], T0_EE[1, 3], T0_EE[2, 3]]# <--- Load your calculated end effector value
    
      ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
