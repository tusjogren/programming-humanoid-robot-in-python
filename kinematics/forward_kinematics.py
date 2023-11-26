'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
                        # Defining the chains for the legs
                        'LLeg': [
                            'LHipYawPitch', 'LHipRoll', 'LHipPitch',
                            'LKneePitch', 'LAnklePitch', 'LAnkleRoll'
                        ],
                        'RLeg': [
                            'RHipYawPitch', 'RHipRoll', 'RHipPitch',
                            'RKneePitch', 'RAnklePitch', 'RAnkleRoll'
                        ],

                        # Defining the chains for the arms
                        'LArm': [
                            'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw',
                            'LElbowRoll'
                        ],
                        'RArm': [
                            'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw',
                            'RElbowRoll'
                        ],

                        # Defining the chain for the head
                        'Head': ['HeadYaw', 'HeadPitch']
                    }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''
        Calculate the local transformation matrix of one joint.

        :param str joint_name: The name of the joint.
        :param float joint_angle: The angle of the joint in radians.
        :return: A 4x4 transformation matrix representing the joint's local transformation.
        :rtype: numpy.ndarray
        '''
        # Initialize a 4x4 identity matrix.
        T = identity(4)

        # Calculate cosine and sine of the joint angle for rotation matrix.
        cos = np.cos(joint_angle)
        sin = np.sin(joint_angle)

         # Determine the type of rotation based on the joint name and set the appropriate rotation matrix elements.
        if 'Roll' in joint_name:  # Rotation around X-axis
            T[1,1] = cos
            T[1,2] =-sin
            T[2,1] = sin
            T[2,2] = cos
        if 'Pitch' in joint_name:  # Rotation around Y-axis
            T[0,0] = cos
            T[0,2] = sin
            T[2,0] =-sin
            T[2,2] = cos
        if 'Yaw' in joint_name:  # Rotation around Z-axis
            T[0,0] = cos
            T[0,1] =-sin
            T[1,0] = sin
            T[1,1] = cos
        
        # Set the translation components (X, Y, Z) based on the joint name.
        # These values are specific to the robot's design and structure.
        X, Y, Z = (0, 0, 0)

        if 'HeadYaw' in joint_name:
            Z = 126.5
        elif 'ElbowYaw' in joint_name:
            X = 105.0
            Y = 15.0
        elif 'ShoulderPitch' in joint_name:
            Y = 98.0
            Z = 100.0
        elif 'WristYaw' in joint_name:
            X = 55.95
        elif 'HipYawPitch' in joint_name:
            Y = 50.0
            Z = -85.0
        elif 'AnklePitch' in joint_name:
            Z = -102.9
        elif 'KneePitch' in joint_name:
            Z = -100.0
        
        # If it's a right side joint, invert the Y-axis translation.
        if joint_name.startswith('R'):
            Y = -Y

        # Set the translation elements in the transformation matrix.
        T[0,3] = X
        T[1,3] = Y
        T[2,3] = Z

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)

                # Multiply the current cumulative transformation matrix (T) with the local transformation (Tl) 
                # of the current joint. This updates T to include the transformation of this joint, 
                # effectively accumulating the transformations along the kinematic chain.
                T = np.dot(T,Tl)

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
