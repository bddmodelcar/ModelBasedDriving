import numpy as np
import time
import h5py
import torch
import torch.utils.data as data
import matplotlib.pyplot as plt
import sys
from numpy.linalg import norm
 
from random import shuffle
import os

from enum import Enum

nframes = 10

class Dataset(data.Dataset):

    global wall_max_x
    global wall_max_y
    global nframes 
    
    def __init__(self, data_folder_dir, cpu_mode, stride=10):
        
        self.cpu_mode = cpu_mode
        # print "Looking at {0} for files".format(os.path.join(data_folder_dir, 'processed_h5py'))
        self.runs = os.walk(data_folder_dir).next()[1]
        #shuffle(self.runs)  # shuffle each epoch to allow shuffle False
        self.run_files = []
        self.nframes = nframes
        # Initialize List of Files
        self.invisible = []
        self.visible = []
        self.total_length = 0 
        self.full_length = 0 

        h5py_max_open_files = 3500  # Experiment
        open_files = 0
        
        for run in self.runs:
            
            segs_in_run = os.walk(os.path.join(data_folder_dir, run)).next()[1]
            #shuffle(segs_in_run)  # shuffle on each epoch to allow shuffle False
            #print segs_in_run[0:3]
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
                
                list_camera_input.append(torch.from_numpy(data_file[camera][t + t_moment - self.nframes]))
                        
        camera_data = torch.cat(list_camera_input, 2)     
        
        if not self.cpu_mode:   
            camera_data = camera_data.cuda().float() / 255. - 0.5
        else:
            camera_data = camera_data.cpu().float() / 255. - 0.5
        
                
        camera_data = torch.transpose(camera_data, 0, 2)        
        camera_data = torch.transpose(camera_data, 1, 2)
        
        # Get Ground Truth
        steer = []
        motor = []

        for i in range(0, self.stride * self.nframes, self.stride):
            steer.append(float(metadata_file['steer'][t + i]))
        for i in range(0, self.stride * self.nframes, self.stride):
            motor.append(float(metadata_file['motor'][t + i]))
        
        final_ground_truth = torch.FloatTensor(steer + motor).cuda() / 99.
        
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







    
    
