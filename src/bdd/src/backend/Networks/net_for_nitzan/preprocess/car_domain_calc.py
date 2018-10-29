'''
Created on Jul 27, 2017

@author: Sascha Hornauer
'''
from operator import div, add, mul, sub
import numpy as np
from kzpy3.data_analysis.trajectory_generator.util.path_follower import PID_Path_Following
import sys
from kzpy3.data_analysis.trajectory_generator.util.map_utils import get_corr_angle, \
    distance
from kzpy3.data_analysis.trajectory_generator.util import map_utils
from kzpy3.data_analysis.trajectory_generator.beacon_navigator import constants
# from kzpy3.data_analysis.trajectory_generator.thirdParty import pid_controller

# min_max_delta = 0.28*np.pi # Maximum steering angle delta
min_max_delta = np.pi / 4.  # Maximum steering angle, reduced by 5 deg to fit all situations
# Delta can be in between +- 50 deg according to manufacturer of the RR10 Bomber, 50 deg = 0.28 * pi

max_left_command = 100
max_right_command = 0
try:
    from angles import normalize as normalize_angle  # Adjust for different angles packages
except ImportError:
    from angles import normalize_angle  #  Adjust for different angles packages

def convert_delta_to_steer(delta_values):
    
        value_range = np.abs(max_left_command - max_right_command)
        
        norm_values = map(div, delta_values, [min_max_delta] * np.ones(len(delta_values)))
        norm_values = map(add, norm_values, [1.] * np.ones(len(delta_values)))
        norm_values = map(div, norm_values, [2.] * np.ones(len(delta_values)))
        norm_values = map(mul, norm_values, [value_range] * np.ones(len(delta_values)))
        
        return norm_values

def convert_steer_to_deltas(steer_values):
    
    # delta values are assumed to be in +pi/2,-pi/2
    
    if isinstance(steer_values, int):
        steer_values = [steer_values]
    
    steering_range = np.abs(max_left_command - max_right_command)
    
    steer_values = map(div, steer_values, [steering_range] * np.ones(len(steer_values)))
    steer_values = map(mul, steer_values, [2.0] * np.ones(len(steer_values)))
    steer_values = map(sub, steer_values, [1.0] * np.ones(len(steer_values)))
    steer_values = map(mul, steer_values, [-min_max_delta] * np.ones(len(steer_values)))
    
    return steer_values
    

class Car_Controller():    

    pid_follower = PID_Path_Following()
    
    # The own start can be used in the future to calculate the XTE
    def get_steering_value(self, own_start, own_position, own_heading, next_point, own_speed):
        if constants.debug: print "Next Point: " + str(next_point)
        if constants.debug: print "Own position " + str(own_position)
        if constants.debug: print "Own Start " + str(own_start)

            
        
        next_delta = self.pid_follower.get_delta(own_start, next_point, own_position, own_heading, own_speed)
        if constants.debug: print "Result clipped " + str(next_delta)
        steering_values = convert_delta_to_steer([next_delta])
        
        # Cut upper limit
        if constants.debug: print "Steering values: " + str(steering_values)
        steering_values_cut = [min(100., value) for value in steering_values]
        
        steering_values_cut = [np.mean(steering_values_cut)]
        
        return steering_values_cut, next_delta, next_point

