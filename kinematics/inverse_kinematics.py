'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


from math import atan2
import numpy as np
from numpy import linalg



class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        # Tried to implement jacobian 

        # Orientaded on the code from inverse_kinematics_2d_jacobian.ipynb
        # Robot appears and moves
        # Not sure if the result should be like this
         
        lambda_ = 1
        max_step = 0.1

        joint_angles = np.ones(len(self.chains[effector_name]))
      

        def from_trans(m):
            '''get x, y, theta from transform matrix'''

            #decompose rotaton matrix, has the same functionallity like in the inverse_kinematics_2d_jacobian.ipynb
            theta_x, theta_y, theta_z = 0, 0, 0
            if m[0, 0] == 1:
                theta_x = atan2(m[2, 1], m[1, 1])
            elif m[1, 1] == 1:
                theta_y = atan2(m[0, 2], m[0, 0])
            elif m[2, 2] == 1:
                theta_z = atan2(m[1, 0], m[0, 0])
            return np.array([ m[3, 0], m[3, 1],  m[3, 2], theta_x, theta_y, theta_z])

        target = np.array(from_trans(transform)).T  
        
        for _ in range(1000):
            Ts = [identity(len(self.chains[effector_name]))]
            for name in self.chains[effector_name]:
                Ts.append(self.transforms[name])

            Te = np.array(from_trans(Ts[-1])).T

            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            
            T = np.array([from_trans(i) for i in Ts[0:-1]]).T
            J = Te - T
            dT = Te - T

            J[0, :] = dT[2, :] # x
            J[1, :] = dT[0, :] # y
            J[-1, :] = 1  # angular
            
            d_theta = lambda_ * (linalg.pinv(J) * e)
            joint_angles += np.asarray(d_theta.T)[0]
            
            if np.linalg.norm(d_theta) < 1e-4:
                break
        
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        joint_angles = self.inverse_kinematics(effector_name, transform)
        name = []
        time = []
        angle = []
        counter = 0
        for joint in self.chains[effector_name]:
            name.append(joint)     #append joint name
            time.append([0.0,5.0])    #append time
            angle.append([[self.perception.joint[joint], [0., 0., 0.]], [joint_angles[counter], [0., 0., 0.]]])
            #only the joint_angle is used by my interpolation method, therefore the other values should have no effect an can be set to 0 
            counter += 1
    
        self.keyframes = (name, time, angle) 
        print(name, time, angle)
        
 
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('RLeg', T)
    agent.run()
