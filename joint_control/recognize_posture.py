'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from os import listdir
import numpy as np
from keyframes import hello
from keyframes import rightBackToStand
import pickle

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        file = 'robot_pose.pkl'
        self.posture_classifier =  pickle.load(open(file))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        file = '../joint_control/robot_pose_data'

        current_angles = np.zeros((1,10))
        joints =['LHipYawPitch','LHipRoll','LHipPitch','LKneePitch','RHipYawPitch','RHipRoll','RHipPitch','RKneePitch'] 
        #use joints and angles given in learn_posture.ibpyn 
        for i in range(len(joints)):
            current_angles[0][i] = perception.joint[joints[i]]
        current_angles[0][8] = perception.imu[0]
        current_angles[0][9] = perception.imu[1]

        prediction = self.posture_classifier.predict(current_angles)
        
        posture = listdir(file)[prediction[0]]
        print(posture)
        return posture 

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
