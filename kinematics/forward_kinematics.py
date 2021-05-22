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
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np
from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       #dont know if we need wrist yaw + hand 
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'], 
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'], 
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       }
                       

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)

        s = np.sin(joint_angle)
        c = np.cos(joint_angle)
        
        # YOUR CODE HERE
        #Implemented 3D coordinate Transformation on slides 9+10
        
        # Z == Roll
        Rx_theta = np.array([[1 , 0, 0], 
                            [0, c ,-s],
                            [0, s , c] ])
        
        # Y == Pitch
        Ry_theta = np.array([[ c, 0, s], 
                            [ 0, 1, 0],
                            [-s, 0, c] ])

        # Z == Yaw
        Rz_theta=  np.array([[ c,-s, 0], 
                            [ s, c, 0],
                            [ 0, 0, 1] ])
        
        # TODO: Regard special joints, where both yaw + pitch are included

        if 'Roll' in joint_name:
            T[0:3,0:3] = Rx_theta
        if 'Pitch' in joint_name:
            T[0:3,0:3] = Ry_theta
        if 'Yaw' in joint_name:
            T[0:3,0:3] = Rz_theta

        length = {'Head': [[0., 0., 126.5],[0., 0., 0.]],
                  'LArm': [[0., 98., 100.],[0., 0., 0.], [105., 15., 0.], [0., 0., 0.]],
                  'RArm': [[0., -98., 100.],[0., 0., 0.], [105., 15., 0.], [0., 0., 0.]],
                  'LLeg': [[0., 50., -85.],[0., 0., 0.], [0., 0., 0.], [0., 0., -100.], [0., 0., -102.9], [0., 0., 0.]],
                  'RLeg': [[0., -50., -85.],[0., 0., 0.], [0., 0., 0.], [0., 0., -100.], [0., 0., -102.9], [0., 0., 0.]]}

        #fill last row with joint length        
        for key in self.chains.keys():
            if joint_name in self.chains[key]:
                idx = self.chains[key].index(joint_name) #find index of current joint in the chain dict
                T[0,3] = length[key][idx][0]
                T[1,3] = length[key][idx][1]
                T[2,3] = length[key][idx][2]
    
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
                # YOUR CODE HERE
                
                T = np.dot(T,Tl)  #simply multiply new transformation matrix with old T

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
