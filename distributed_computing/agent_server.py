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
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

import time
from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading

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

    def get_posture(self):
        '''return current posture of robot'''
        print("Get posture")
        
        #return self.recognize_posture()
        return self.posture()
        # YOUR CODE HERE

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        print("Execute keyframes")
        
        self.keyframes = keyframes
        #self.angle_interpolation(keyframes, self.perception)
        #while True:
        #    if self.keyframes == {}:
        #        break
        #    print(len(keyframes))
        #    print(len(keyframes[1]))
        #    print(keyframes[1])
        #self.run() 

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
        self.transform[effector_name] = transform  
        # YOUR CODE HERE

if __name__ == '__main__':
    agent = ServerAgent()
    print("Start Server")
    # Create server
    server = SimpleXMLRPCServer(("localhost", 8000), allow_none = True)
    server.register_instance(agent)
    #server.register_instance(XmlrpcHandler)
    print("XMLRPC-Server listening to http://localhost:8000.")
    print("Shut down with STRG+C")
    server.register_introspection_functions()
    server.register_multicall_functions()
    
    #server.serve_forever()
    #tried to implement threading
    thread = threading.Thread(target=server.serve_forever)  
    thread.start()
    
    agent.run()
