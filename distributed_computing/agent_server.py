'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
from os import listdir

import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from keyframes import *
from SimpleXMLRPCServer import SimpleXMLRPCServer

import time
import threading
import numpy as np
import pickle

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
              
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        print("Get angle")
        return self.perception.joint[joint_name]
        # YOUR CODE HERE
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        print("Set angle")
        self.target_joints[joint_name] = angle
        # YOUR CODE HERE

    def get_posture(self):                       #simply used code from joint_control to detect posture
        '''return current posture of robot'''
        print("Get posture")
        self.posture = 'unknown'
        file = 'robot_pose.pkl'
        self.posture_classifier =  pickle.load(open(file))  # LOAD YOUR CLASSIFIER
        file = '../joint_control/robot_pose_data'
        current_angles = np.zeros((1,10))
        joints =['LHipYawPitch','LHipRoll','LHipPitch','LKneePitch','RHipYawPitch','RHipRoll','RHipPitch','RKneePitch'] 
        #use joints and angles given in learn_posture.ibpyn 
        for i in range(len(joints)):
            current_angles[0][i] = self.perception.joint[joints[i]]
        current_angles[0][8] = self.perception.imu[0]
        current_angles[0][9] = self.perception.imu[1]
        prediction = self.posture_classifier.predict(current_angles)
        posture = listdir(file)[prediction[0]]
        return posture 
        #return self.recognize_posture()    #not working

        
    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        print("Execute keyframes")
        self.set_time = False 

        if callable(keyframes):
            keys = keyframes()
            self.keyframes = keys
        else:
            self.keyframes = keyframes

        time.sleep(max(map(max,self.keyframes[1]))+4) #look for the max value in times array and wait so long
           
        return True

        
    def get_transform(self, name):
        '''get transform with given name
        '''
        print("Get transform")
        return self.transforms[name]    
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        print("Set transform")

        super(ServerAgent, self).set_transforms(effector_name, transform)  
        # YOUR CODE HERE

if __name__ == '__main__':
    agent = ServerAgent()
    print("Start Server")

    server = SimpleXMLRPCServer(("localhost", 8000), allow_none = True)
    server.register_instance(agent)

    print("XMLRPC-Server listening to http://localhost:8000.")
    print("Shut down with STRG+C")
    server.register_introspection_functions()
    server.register_multicall_functions()
    
    thread = threading.Thread(target=server.serve_forever)  
    thread.start()
    agent.run()
