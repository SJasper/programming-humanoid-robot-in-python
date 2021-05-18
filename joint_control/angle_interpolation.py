'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
#from keyframes import dance
from keyframes import rightBackToStand
from keyframes import leftBellyToStand
from keyframes import wipe_forehead
from scipy import interpolate
import numpy as np
from scipy.interpolate import CubicSpline

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.time = 0          #variable for saving start time
        self.set_time = False   #variable to check if starttime was set 
        
    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)
    

    
    def angle_interpolation(self, keyframes, perception):
        '''
        def bezier(k0,k1,k2,k3,t):
            #calculate all the terms needed for bezier interpolation described in the lecture
             # use the calculated points k0 -k4 

            first =  np.power(1 - t, 3) * k0
            second = 3 * t * np.power(1 - t, 2) * k1
            third = 3 * np.power(t , 2) * (1 - t) * k2
            fourth = np.power(t,3) * k3
                
            #calculate general cubic formular of bezier interpolation and use the calculated terms
            return first + second + third  + fourth 
        ''' 
        # tried bezier before, focussed on spline interpolaton  
              
        target_joints = {}          #key - angle value dictionary
        names, times, keys = keyframes      #get names of joints, list of times and angles belonging to each time 
        
        if not self.set_time:       #check if animation started 
            self.set_time = True    
            self.time = perception.time     #initialize start time if not started
        
        current_time = perception.time - self.time -4  #calculate current time from start time     
        
        if (current_time<0):      #skip the first timesteps to avoid executing angle interpolation in mid air
            return target_joints

        num_joints = len(names)     #calculate number of joints to move  
        for joint in range(num_joints): 

            joint_times = times[joint]  #get time list belonging to current joint 

            len_time_seq = len(times[joint]) 
            if (current_time <= joint_times[len_time_seq-1]):   #if the animation is running 
                joint_angles = []   
                for i in keys[joint]:
                    joint_angles.append(i[0])  #get joint angles from keyframes array
                if(len_time_seq<=3):    #avoid interpolation with splrep with less than 4 values
                    joint_name = names[joint] 
                    if names[joint] == "RHipYawPitch":
                        joint_name = "LHipYawPitch"
                    target_joints[joint_name] = np.interp(current_time, joint_times, joint_angles) #interpolate with numpy
                else:       #enough values to make cubic spline interpolation
                    spl = interpolate.splrep(joint_times, joint_angles)   # calculate interpolation function for time and angle values 
                    #f = CubicSpline(joint_times, joint_angles, bc_type='natural',extrapolate=True)
                    joint_name = names[joint] 
                    if names[joint] == "RHipYawPitch":
                        joint_name = "LHipYawPitch"
                    target_joints[joint_name] = interpolate.splev(current_time, spl)    #save angle calculated from spline and current time in target_joints 
                    #target_joints[joint_name] = f(current_time)
        return target_joints
        
    
if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    #agent.keyframes = wipe_forehead()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = dance()  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
