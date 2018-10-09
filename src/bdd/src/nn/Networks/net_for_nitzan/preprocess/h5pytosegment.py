import numpy as np
import h5py
import os
from multiprocessing import Pool
from time import sleep
from enum import Enum

import cv2
import numpy as np
from scipy.misc.pilutil import imresize

from trajectory_tools import get_heading
import math
import sys

        
current_goal_left = (0.6, 7.3)
current_goal_center = (2.1, 7.3)
current_goal_right = (3.4, 7.3)

extract_goal= False
extract_xyz = False

class GoalCode(Enum):
    left = 0
    center = 1
    right = 2
    no_goal = 3
    

def add_heading(raw_positions):
    
    # Number of positions to calculate the heading.
    # It has to be higher to be able to do interpolation when
    # the values are not changing
    number_of_pos = 8
    
    full_poses = []
    
    for i in range(len(raw_positions)):
        
        try:
            calc_over_pos = raw_positions[i:i+number_of_pos]
            heading = get_heading(calc_over_pos)
            if math.isnan(heading):
                # Little hack, if we have no point of reference and heading can
                # not be calculated assume heading is 0
                if not full_poses:
                    heading = 0
                else:
                    heading = full_poses[i-1][2]
                
            #print "Heading was {0}".format(heading)
            full_poses.append((raw_positions[i][0],raw_positions[i][1],heading))
            
        except IndexError:
            # If we reached the end of the sequence just copy the heading of the last entry
            for j in range(i,i+number_of_pos-1):
                full_poses.append((raw_positions[j][0],raw_positions[j][1],get_heading(full_poses[i-1])))
            #print "End of array reached {0}".format(i)
            
    return np.array(full_poses)
    
    


def flip(own_poses):
    
    max_x = 4.1
    max_y = 10.6
    
    flipped_pose = []
    
    for pose in own_poses:
        
        angle = pose[2]
        
        x = np.cos(angle)
        y = np.sin(angle)
        x = -x
        flipped_angle = np.arctan2(y,x)
        
        flipped_pose.append((max_x - pose[0], pose[1],flipped_angle))
    
    return np.array(flipped_pose)
    


def process(run_name):
    input_prefix = '/home/sascha/testmount/dest/FirstChunk/'
    output_prefix = '/home/sascha/secondDisk/segmentedDataNoXYGoal/'
    #print "Processing {0}".format(run_name)

    try:
        os.makedirs(os.path.join(output_prefix, run_name))
    except OSError:
        print "Directory already existing. Skipping run {0}".format(run_name)
        return
        
    try:
        
        f_meta = h5py.File(os.path.join(os.path.join(input_prefix, run_name), 'left_timestamp_metadata.h5py'), 'r')
        f_img = h5py.File(os.path.join(os.path.join(input_prefix, run_name), 'flip_images.h5py'), 'r')
        f_normal_img = h5py.File(os.path.join(os.path.join(input_prefix, run_name), 'original_timestamp_data.h5py'), 'r')
    except IOError:
        print "Error: Needed h5py file not found. Skipping run {0}".format(run_name)
        return
        
    try:
        rounded_state = np.round(f_meta['state'][:])  # TODO: export to h5py
    except KeyError:
        print "Error: Damaged run {0}. Skipping".format(run_name)
        return
        
    for left_ts_id in range(1, len(rounded_state) - 1):
        if not rounded_state[left_ts_id - 1] == 4 and not rounded_state[left_ts_id + 1] == 4 and rounded_state[left_ts_id] == 4:
            rounded_state[left_ts_id] = rounded_state[left_ts_id - 1]
            
    consecutive_seq_idx = np.zeros(len(f_meta['ts']))

    def is_valid_timestamp(state, motor, allow_state=[6], min_motor=53):
        return state in allow_state and motor > min_motor

    for left_ts_id in range(1, len(consecutive_seq_idx)):
        consecutive_seq_idx[left_ts_id] = int(is_valid_timestamp(rounded_state[left_ts_id], f_meta['motor'][left_ts_id]) and f_meta['ts'][left_ts_id] - f_meta['ts'][left_ts_id - 1] < 0.3)

    # find closest right idx to each left idx (in time)
    left_idx_to_right = []
    left_ts = f_img['left_image_flip']['ts'][:]
    right_ts = f_img['right_image_flip']['ts'][:]
    for left_ts_id in range(len(left_ts)):
        try:
            diffs = np.abs(left_ts[left_ts_id] - right_ts[max(0, left_ts_id - 10) : min(left_ts_id + 10, len(left_ts) - 1)])
            left_idx_to_right.append(np.argmin(diffs) + max(0, left_ts_id - 10))
        except Exception:
            consecutive_seq_idx[left_ts_id] = 0  # if there is a problem get rid of it
    
    

    def contiguous_regions(condition):
        """Finds contiguous True regions of the boolean array "condition". Returns
        a 2D array where the first column is the start index of the region and the
        second column is the end index."""

        # Find the indicies of changes in "condition"
        d = np.diff(condition)
        idx, = d.nonzero() 

        # We need to start things after the change in "condition". Therefore, 
        # we'll shift the index by 1 to the right.
        idx += 1

        if condition[0]:
            # If the start of condition is True prepend a 0
            idx = np.r_[0, idx]

        if condition[-1]:
            # If the end of condition is True, append the length of the array
            idx = np.r_[idx, condition.size]  # Edit

        # Reshape the result into two columns
        idx.shape = (-1, 2)
        return idx

    condition = consecutive_seq_idx.astype(bool)

    def save_h5py(state, motor, steer, left, right, time, seg_num, seg_length, goal, poses):
        output_dir = os.path.join(output_prefix, run_name, "seg_" + str(seg_num))
        
        try:
            os.makedirs(output_dir)
        except OSError:
            print "Directory already existing. Skipping run {0}".format(output_dir)
            return
        

        new_f_images = h5py.File(os.path.join(output_dir, "images.h5py"))
        new_f_images.create_dataset('left', (seg_length, 94, 168, 3), dtype='uint8')
        new_f_images['left'][:] = left
        new_f_images.create_dataset('right', (seg_length, 94, 168, 3), dtype='uint8')
        new_f_images['right'][:] = right
        new_f_images.create_dataset('ts', (seg_length,), dtype='int')
        new_f_images['ts'][:] = time
        
        new_f_metadata = h5py.File(os.path.join(output_dir, "metadata.h5py"))
        new_f_metadata.create_dataset('steer', (seg_length,), dtype='uint8')
        new_f_metadata['steer'][:] = steer.astype('uint8')
        new_f_metadata.create_dataset('motor', (seg_length,), dtype='uint8')
        new_f_metadata['motor'][:] = motor.astype('uint8')
        new_f_metadata.create_dataset('state', (seg_length,), dtype='uint8')
        new_f_metadata['state'][:] = state.astype('uint8')
        if extract_goal:
            new_f_metadata.create_dataset('goal', (seg_length,), dtype='uint8')
            new_f_metadata['goal'][:] = goal.astype('uint8')
        if extract_xyz:
            new_f_metadata.create_dataset('pose', (seg_length,3), dtype=np.dtype("f"))
            new_f_metadata['pose'][:] = poses.astype(np.dtype("f"))

    def decide_goal_type(goal_data):
        
        goal_list = []
        
        # This should ideally be a one-hot-vector though this has to be translated
        # in the training code since sequence of sequences are too hard to parse right
        # now
        for goal_entry in goal_data:
            
            goal_entry = (np.asscalar(goal_entry[0]), np.asscalar(goal_entry[1]))
            if (goal_entry == current_goal_left):
                # print "left"
                goal_list.append(GoalCode.left.value)
            elif (goal_entry == current_goal_center):
                # print "center"
                goal_list.append(GoalCode.center.value)
            elif (goal_entry == current_goal_right):
                # print "right"
                goal_list.append(GoalCode.right.value)
            else:
                goal_list.append(GoalCode.no_goal.value)
                # print "Goal was {0}".format(goal_entry)
        return np.array(goal_list)


    def flip_goallist(goal_list):
        
        if not goal_list:
            return None
        
        for i in range(len(goal_list)):
            
            goal = goal_list[i]
            
            if goal == GoalCode.left.value:
                goal_list[i] = GoalCode.right.value
            elif goal == GoalCode.right.value:
                goal_list[i] = GoalCode.left.value
        
        return np.array(goal_list)
    



# #print the start and stop indicies of each region where the absolute 
# values of x are below 1, and the min and max of each of these regions
    seg_num = 0
    print contiguous_regions(condition)
    for start, stop in contiguous_regions(condition):
        if stop - start > 110:
            
            state = rounded_state[start:stop]
            motor = f_meta['motor'][start:stop]
            steer = 99 - f_meta['steer'][start:stop]
  
            # My understanding, build a synchronized list of left and right camera
            # images, which is also flipped
            count = 0
            left = np.zeros((stop - start, 94, 168, 3), dtype='uint8')
            right = f_img['left_image_flip']['vals'][start:stop]  # notice the flip
            for left_ts_id in range(start, stop):
                left[count] = f_img['right_image_flip']['vals'][left_idx_to_right[left_ts_id]]
                count += 1
            time = np.array(list(range(len(left))))
            seg_length = len(left)
            
            if extract_goal:
            
                try:
                    goal_timestamps = f_normal_img['current_goal_x']['ts'][:]
                    pos_timestamps = f_normal_img['hedge_pos_x']['ts'][:]
                except KeyError:
                    print "Error: Goal Information not found. Skipping run {0}".format(run_name)
                    return
                    
                goal_list = []
                
                goal_ts_id = 0
                left_ts_id = start
                
                for left_ts_id in range(start, stop):
    
                     
                    while goal_ts_id < len(goal_timestamps)-1 and goal_timestamps[goal_ts_id] < left_ts[left_ts_id]:
                        goal_ts_id += 1
                      
    
                    goal_list.append((f_normal_img['current_goal_x']['vals'][goal_ts_id], f_normal_img['current_goal_y']['vals'][goal_ts_id]))
            else:
                goal = None
                    
            if extract_xyz:
                # Go again through all hedge positions
                position_list = []
                 
                pos_ts_id = 0
                left_ts_id = start
                
                goal_test = 0
                for left_ts_id in range(start, stop):
    
                    while pos_ts_id < len(pos_timestamps)-1 and pos_timestamps[pos_ts_id] < left_ts[left_ts_id]:
                        #print "increase pos a"
                        pos_ts_id += 1
    
                     
                    position_list.append((f_normal_img['hedge_pos_x']['vals'][pos_ts_id], f_normal_img['hedge_pos_y']['vals'][pos_ts_id]))
                 
               
                goal = decide_goal_type(goal_list)
                raw_positions = position_list
                own_poses = add_heading(raw_positions)
                flipped_poses = flip(own_poses)
            else:
                flipped_poses = None
                own_poses = None
        
            save_h5py(state, motor, steer, left, right, time, seg_num, seg_length, flip_goallist(goal), flipped_poses)

            seg_num += 1

            # Unflipped Images
            steer = f_meta['steer'][start:stop]
            time = np.array(list(range(len(left))))

            count = 0
            right = np.zeros((stop - start, 94, 168, 3), dtype='uint8')
            left = f_normal_img['left_image']['vals'][start:stop]
            for left_ts_id in range(start, stop):
                right[count] = f_normal_img['right_image']['vals'][left_idx_to_right[left_ts_id]]
                count += 1
            time = np.array(list(range(len(left))))

            save_h5py(state, motor, steer, left, right, time, seg_num, seg_length, goal,own_poses)

            seg_num += 1

            # print start, stop

if __name__ == '__main__':
    input_prefix = '/home/sascha/testmount/dest/FirstChunk/'
    run_names = next(os.walk(input_prefix))[1]
    # run_names = ['direct_racing_Tilden_27Nov16_12h20m21s_Mr_Blue']
    pool = Pool(processes=10)
    pool.map(process, run_names)
    #process(run_names[0])
