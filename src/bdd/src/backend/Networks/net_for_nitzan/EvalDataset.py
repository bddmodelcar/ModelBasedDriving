import numpy as np
import time
import h5py
import torch
import torch.utils.data as data
import matplotlib.pyplot as plt
import sys
from numpy.linalg import norm
from matplotlib.mlab import dist_point_to_segment, dist

# The constants are only once defined in Dataset
from Dataset import wall_max_x, wall_max_y, wall_min_x, wall_min_y, wall_a, wall_b, wall_c , \
    obstacle_left_a, obstacle_left_b, wall_d, obstacle_left_c, obstacle_left_d, \
    obstacle_right_a, obstacle_right_b, obstacle_right_c, obstacle_right_d

# Checking if the ros normalize angle or the other python normalize angle is installed
try:
    from angles import normalize_angle as normalize
except:
    from angles import normalize as normalize
 
from random import shuffle
import os

from enum import Enum


class GoalCode(Enum):
    left = 0
    center = 1
    right = 2
    no_goal = 3

# This could be cleverley calculated in a for loop though for efficiency
# and to avoid eventual bugs it's right now more hard-coded
row_one_angles = [(np.pi / 6),  0,  -(np.pi / 6)]
row_two_angles = [(np.pi / 4),  0,  -(np.pi / 4)]
point_distance = 1.0  # m
no_of_points = 5

def get_front_points(own_pos_x, own_pos_y, own_pos_phi):
    
    front_points = []
    
    for angle in row_one_angles:
        
        offset_x = np.cos(angle + own_pos_phi) * point_distance
        offset_y = np.sin(angle + own_pos_phi) * point_distance
    
        point_x = offset_x + own_pos_x
        point_y = offset_y + own_pos_y
        
        front_points.append([point_x, point_y])
    
    for angle in row_one_angles:    
        for second_angle in row_two_angles:
            
            offset_x = (np.cos(angle + own_pos_phi) * point_distance) + (np.cos(second_angle + own_pos_phi) * point_distance)
            offset_y = (np.sin(angle + own_pos_phi) * point_distance) + (np.sin(second_angle + own_pos_phi) * point_distance)
        
            point_x = offset_x + own_pos_x
            point_y = offset_y + own_pos_y
            
            front_points.append([point_x, point_y])
    
    return front_points     


def get_free_probabilities(front_end_points):
    '''
    Check if the front end points of the Dubins lanes are still within the area.
    In the application scenario, the office, only rectangles are sufficient to
    check because the obstacles are rectangles and the outer walls are also at
    a right angle. This simplifies detection if those points are in the area
    to simple checks and no arbitrary polygon check is implemented for now.
    
    A further simplification is made in the sense that the probability of a
    end-point being within an obstacle or outside is calculated by taking the
    perpendicular distance from the x and y axis instead of the 
    '''
    point_penalty = []
    
    for front_point in front_end_points:
        wall_penalty = get_outside_wall_penalty(front_point, wall_a, wall_b, wall_c, wall_d)
        if wall_penalty > 0.0:
            point_penalty.append(wall_penalty)
            # If this point has already a wall penalty, it can not be in an obstacle
            continue
        
        # To check if the point is not in the obstacle we substract it's probability of being
        # within from 1.0
        obstacle_penalty_left = get_inside_obstacle_penalty(front_point, obstacle_left_a, obstacle_left_b, obstacle_left_c, obstacle_left_d)
        if obstacle_penalty_left > 0.0:
            point_penalty.append(obstacle_penalty_left)
            # If that point is within the left obstacle it can not be in the right obstacle
            continue
        
        obstacle_penalty_right = get_inside_obstacle_penalty(front_point, obstacle_right_a, obstacle_right_b, obstacle_right_c, obstacle_right_d)
        if obstacle_penalty_right > 0.0:
            point_penalty.append(obstacle_penalty_right)
            # If that point is within the right obstacle it can not be in the left obstacle
            continue

        # The point is within the rectangle and not within an obstacle.
        point_penalty.append(0.0)

    return point_penalty
    
def get_distance_to_line(p1, p2, p3):
    '''
    Return the distance to the line, defined by point p1 and p2 of point p3
    This method was redesigned after the matplotlib method was found
    '''
    # print "Points {0},{1},{2}".format(p1,p2,p3)
    d = dist_point_to_segment(p3, p1, p2)
    return d

def get_outside_wall_penalty(query_point, p1, p2, p3, p4):
    
    query_point = np.array(query_point)
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    p4 = np.array(p4)
    
    # The p1-p4 information is redundant in terms of the definition of
    # the rectangle since an askew form is not supported
    min_x = min(p1[0], p2[0], p3[0], p4[0])
    min_y = min(p1[1], p2[1], p3[1], p4[1])
    
    max_x = max(p1[0], p2[0], p3[0], p4[0])
    max_y = max(p1[1], p2[1], p3[1], p4[1])    
    
    within_rectangle = min_x < query_point[0] < max_x and min_y < query_point[1] < max_y
    
    if within_rectangle:
        return 0.
    else:        
        dist_a = get_distance_to_line(p1, p2, query_point)
        dist_b = get_distance_to_line(p2, p3, query_point)
        dist_c = get_distance_to_line(p3, p4, query_point)
        dist_d = get_distance_to_line(p4, p1, query_point)
        
        # Normalization
        min_distance = min(dist_a, dist_b, dist_c, dist_d)
        if min_distance > point_distance:
            return 1.0
        else:        
            return min_distance / point_distance
        
def get_inside_obstacle_penalty(query_point, p1, p2, p3, p4):
    
    query_point = np.array(query_point)
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    p4 = np.array(p4)
    
    # The p1-p4 information is redundant in terms of the definition of
    # the rectangle since an askew form is not supported
    min_x = min(p1[0], p2[0], p3[0], p4[0])
    min_y = min(p1[1], p2[1], p3[1], p4[1])
    
    max_x = max(p1[0], p2[0], p3[0], p4[0])
    max_y = max(p1[1], p2[1], p3[1], p4[1])    
    
    within_rectangle = min_x < query_point[0] < max_x and min_y < query_point[1] < max_y
    
    if within_rectangle:
        dist_a = get_distance_to_line(p1, p2, query_point)
        dist_b = get_distance_to_line(p2, p3, query_point)
        dist_c = get_distance_to_line(p3, p4, query_point)
        dist_d = get_distance_to_line(p4, p1, query_point)
        
        # Normalization
        min_distance = min(dist_a, dist_b, dist_c, dist_d)
        if min_distance > point_distance:
            return 1.0
        else:        
            return min_distance / point_distance
    else:        
        return 0.0
        
class Dataset(data.Dataset):

    global wall_max_x
    global wall_max_y
    
    nframes = 2

    
    def __init__(self, data_folder_dir, stride=10):
        # print "Looking at {0} for files".format(os.path.join(data_folder_dir, 'processed_h5py'))
        self.runs = os.walk(data_folder_dir).next()[1]
        shuffle(self.runs)  # shuffle each epoch to allow shuffle False
        self.run_files = []

        # Initialize List of Files
        self.invisible = []
        self.visible = []
        self.total_length = 0 
        self.full_length = 0 

        h5py_max_open_files = 30  # Experiment
        open_files = 0
        
        for run in self.runs:
            
            segs_in_run = os.walk(os.path.join(data_folder_dir, run)).next()[1]
            shuffle(segs_in_run)  # shuffle on each epoch to allow shuffle False

            for seg in segs_in_run:
                if open_files < h5py_max_open_files:
                    images = h5py.File(os.path.join(data_folder_dir, run, seg, 'images.h5py'), 'r')
                    metadata = h5py.File(os.path.join(data_folder_dir, run, seg, 'metadata.h5py'), 'r')
                    images_file = images
                else:
                    images = os.path.join(data_folder_dir, run, seg, 'images.h5py')
                    metadata = os.path.join(data_folder_dir, run, seg, 'metadata.h5py')
                    images_file = h5py.File(images, 'r')

                open_files += 1

                length = len(images_file['left'])
                self.run_files.append({'images': images, 'metadata': metadata})
                self.visible.append(self.total_length)  # visible indicies

                # invisible is not actually used at all, but is extremely useful
                # for debugging indexing problems and gives very little slowdown
                self.invisible.append(self.full_length + 7)  # actual indicies mapped

                self.total_length += (length - (10 * stride - 1) - 7)
                self.full_length += length


        self.stride = stride

    def __getitem__(self, index):
        run_idx, t = self.create_map(index)
        
        run_img_obj = self.run_files[run_idx]['images']
        run_meta_obj = self.run_files[run_idx]['metadata']
        
        if isinstance(run_img_obj, str):        
            data_file = h5py.File(run_img_obj, 'r')
        else:
            data_file = run_img_obj
        
        if isinstance(run_meta_obj, str):    
            metadata_file = h5py.File(run_meta_obj, 'r')
        else:
            metadata_file = run_meta_obj
        
        list_camera_input = []

        for t_moment in range(self.nframes):            
            for camera in ('left', 'right'):                
                list_camera_input.append(torch.from_numpy(data_file[camera][t + t_moment - 1]))      
            
        camera_data = torch.cat(list_camera_input, 2)  
        camera_data = camera_data.float() / 255. - 0.5        
        camera_data = torch.transpose(camera_data, 0, 2)        
        camera_data = torch.transpose(camera_data, 1, 2)
        
        # The positioning system is slightly behind the visual data so there is a
        # small correction factor here
        meta_t = t+5
        
        own_pos_x = metadata_file['pose'][meta_t][0] / wall_max_x
        own_pos_y = metadata_file['pose'][meta_t][1] / wall_max_y
        own_pos_phi = ((metadata_file['pose'][meta_t][2] / np.pi) + 1) / 2.
        
        front_end_points = get_front_points(metadata_file['pose'][meta_t][0], metadata_file['pose'][meta_t][1], metadata_file['pose'][meta_t][2])
        
        obstacle_probability = get_free_probabilities(front_end_points)
        # This differes from the training dataset for visualization purposes
        truth_values = [own_pos_x, own_pos_y, own_pos_phi]
        truth_values.extend(obstacle_probability)
        
        truth = sum([truth_values], [])
        
        final_ground_truth = torch.FloatTensor(truth)
        
        if isinstance(run_img_obj, str):        
            data_file.close()
            
        if isinstance(run_meta_obj, str):    
            metadata_file.close()

        return camera_data, final_ground_truth

    def __len__(self):
        return self.total_length

    def create_map(self, global_index):
        for idx, length in enumerate(self.visible[::-1]):
            if global_index >= length:
                return len(self.visible) - idx - 1, global_index - length + 7


if __name__ == '__main__':
    
    own_x = 2.0
    own_y = 10.5
    own_phi = np.pi / 4.

    
    front_points = get_front_points(own_x, own_y, own_phi)
    data_points = np.array(front_points).T
    data_points = [np.append(data_points[0], own_x), np.append(data_points[1], own_y)]
    plt.axis('equal')
    plt.scatter(data_points[0], data_points[1])
    
    plt.plot([wall_a[0], wall_b[0], wall_c[0], wall_d[0], wall_a[0]], [wall_a[1], wall_b[1], wall_c[1], wall_d[1], wall_a[1]], 'k-', lw=2)
    plt.plot([obstacle_left_a[0], obstacle_left_b[0], obstacle_left_c[0], obstacle_left_d[0], obstacle_left_a[0]], [obstacle_left_a[1], obstacle_left_b[1], obstacle_left_c[1], obstacle_left_d[1], obstacle_left_a[1]], 'k-', lw=2)
    plt.plot([obstacle_right_a[0], obstacle_right_b[0], obstacle_right_c[0], obstacle_right_d[0], obstacle_right_a[0]], [obstacle_right_a[1], obstacle_right_b[1], obstacle_right_c[1], obstacle_right_d[1], obstacle_right_a[1]], 'k-', lw=2)
    
    print get_free_probabilities(front_points)
    
    plt.show()
    plt.pause(5000)

    
    
