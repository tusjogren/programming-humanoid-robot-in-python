'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from math import atan2
import numpy as np
from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from scipy.optimize import fmin
from numpy.linalg import norm

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def _end_effector_error(self, joint_angles, joint_names, target):
        Te = np.identity(4)
        for joint_name, angle in zip(joint_names, joint_angles):
            Te = np.dot(Te, self.local_trans(joint_name, angle))
        End = np.matrix(self.from_transform(Te))
        return norm(target - End)

    def _optimize_joint_angles(self, joint_names, target):
        initial_angles = [self.perception.joint[name] for name in joint_names]
        return fmin(lambda angles: self._end_effector_error(angles, joint_names, target), initial_angles)
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_names = self.chains[effector_name]
        target = np.matrix(self.from_transform(transform))
        optimized_angles = self._optimize_joint_angles(joint_names, target)
        return {name: angle for name, angle in zip(joint_names, optimized_angles)}

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        self.keyframes = (
            self.chains[effector_name],
            [[0., 3.]] * len(joint_angles),
            [[
                [self.perception.joint[name], [3, 0, 0], [3, 0, 0]],
                [angle, [3, 0, 0], [3, 0, 0]]
            ] for name, angle in joint_angles.items()]
        )

    def from_transform_np(self, t):
        # Using NumPy's arctan2 function for calculating Euler angles
        theta_X = np.arctan2(t[2, 1], t[2, 2])
        theta_Y = np.arctan2(t[0, 2], t[0, 0])
        theta_Z = np.arctan2(t[1, 0], t[1, 1])

        # Extracting the translation components directly using NumPy slicing
        translation = t[:3, 3]

        return [*translation, theta_X, theta_Y, theta_Z]

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 5
    T[-1, 2] = -260
    agent.set_transforms('LLeg', T)
    agent.run()
