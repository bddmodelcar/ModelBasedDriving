"""Training and validation code for bddmodelcar."""
import sys
import traceback
import logging

from Parameters import ARGS
from EvalDataset import Dataset
import Utils
import numpy as np
from enum import Enum
import os
from nets.SqueezeNetDubins import SqueezeNet
from torch.autograd import Variable
import torch.nn.utils as nnutils
import torch
import torchvision
import matplotlib.pyplot as plt
import cv2
from torchvision.transforms import ToPILImage
from scipy.misc.pilutil import imresize

import pygame
import sys
import EvalDataset


class GoalCode(Enum):
    left = 0
    center = 1
    right = 2
    no_goal = 3


motor_gain = 1.
steer_gain = 1.
last_time = 0

verbose = False

nframes = 2 

# Some constants are only once defined in Dataset
from Dataset import wall_max_x, wall_max_y, wall_min_x, wall_min_y, wall_a, wall_b, wall_c , \
    obstacle_left_a, obstacle_left_b, wall_d, obstacle_left_c, obstacle_left_d, \
    obstacle_right_a, obstacle_right_b, obstacle_right_c, obstacle_right_d

map_width_meter = wall_max_x
map_height_meter = wall_max_y
linewidth = 1
white = (255,255,255)

size = 400,1000

pygame.init()
screen = pygame.display.set_mode(size)
screen.fill(white)

#pygame.draw.rect(screen, (255,0,0), (0,0,400,1000), 10)
def trans_to_pyg_cord(point_in_meter):
    
    pos_point_x = int((400 / wall_max_x) * point_in_meter[0])
    pos_point_y = 1000 - int((1000 / wall_max_y) * point_in_meter[1])
    
    scale = [[1.0,0.0],[0.0,1.0]]
    translation = [0,0]
    
    scaled_point = np.dot((pos_point_x,pos_point_y),scale)
    
    return (int(scaled_point[0])+translation[0], int(scaled_point[1])+translation[1])

def draw_map(open_cv_image, width, height, pos_val_x_true, pos_val_y_true, pos_val_phi_true, net_outputs=None, true_propabilities=None):
    
    pos_val_x_true = 2.0
    pos_val_y_true = 4.0
    pos_val_phi_true = np.pi/2.
    
    front_points = EvalDataset.get_front_points(pos_val_x_true, pos_val_y_true, pos_val_phi_true)
            
    # Draw network output
    screen.fill(white)
    
    # Draw ground truth
    pos_x_true, pos_y_true = trans_to_pyg_cord((pos_val_x_true, pos_val_y_true))
    
    phi_true_circ_d = 20
    phi_true_circ_x_true = int(np.cos(pos_val_phi_true) * phi_true_circ_d)
    phi_true_circ_y_true = int(np.sin(pos_val_phi_true) * phi_true_circ_d)
    
    if front_points:
        for point in front_points:
            
            pos_point_x , pos_point_y = trans_to_pyg_cord(point)
            
            pygame.draw.circle(screen, (255, 0, 0), (pos_point_x, pos_point_y), 10, 5)
            
            
    # Draw probabilities as lines if existent
    if net_outputs is not None:
        
        full_dist = EvalDataset.point_distance
        i = 0
        
        for output in net_outputs[0]:
            output = output.data[0]
            #print output
            part_dist = full_dist*(output)
            end_pos_x = (np.cos(EvalDataset.point_angles[i]+pos_val_phi_true) * part_dist)+pos_val_x_true
            end_pos_y = (np.sin(EvalDataset.point_angles[i]+pos_val_phi_true) * part_dist)+pos_val_y_true
            
            end_pos_x,end_pos_y = trans_to_pyg_cord((end_pos_x,end_pos_y))
            
            pygame.draw.line(screen, (0,255, 255), (pos_x_true,pos_y_true), (end_pos_x, end_pos_y),10)
            i += 1
            
            
    # Draw probabilities as lines if existent
    if true_propabilities is not None:
        
        full_dist = EvalDataset.point_distance
        i = 0
        #print true_propabilities
        for output in true_propabilities:
            
            part_dist = full_dist*(output)
            end_pos_x = (np.cos(EvalDataset.point_angles[i]+pos_val_phi_true) * part_dist)+pos_val_x_true
            end_pos_y = (np.sin(EvalDataset.point_angles[i]+pos_val_phi_true) * part_dist)+pos_val_y_true
            
            end_pos_x,end_pos_y = trans_to_pyg_cord((end_pos_x,end_pos_y))
            
            pygame.draw.line(screen, (0, 0, 255), (pos_x_true,pos_y_true), (end_pos_x, end_pos_y),5)
            i += 1
    
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((wall_a[0], wall_a[1])), trans_to_pyg_cord((wall_b[0], wall_b[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((wall_b[0], wall_b[1])), trans_to_pyg_cord((wall_c[0], wall_c[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((wall_c[0], wall_c[1])), trans_to_pyg_cord((wall_d[0], wall_d[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((wall_d[0], wall_d[1])), trans_to_pyg_cord((wall_a[0], wall_a[1])),5)
    
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((obstacle_left_a[0], obstacle_left_a[1])), trans_to_pyg_cord((obstacle_left_b[0], obstacle_left_b[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((obstacle_left_b[0], obstacle_left_b[1])), trans_to_pyg_cord((obstacle_left_c[0], obstacle_left_c[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((obstacle_left_c[0], obstacle_left_c[1])), trans_to_pyg_cord((obstacle_left_d[0], obstacle_left_d[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((obstacle_left_d[0], obstacle_left_d[1])), trans_to_pyg_cord((obstacle_left_a[0], obstacle_left_a[1])),5)
    
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((obstacle_right_a[0], obstacle_right_a[1])), trans_to_pyg_cord((obstacle_right_b[0], obstacle_right_b[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((obstacle_right_b[0], obstacle_right_b[1])), trans_to_pyg_cord((obstacle_right_c[0], obstacle_right_c[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((obstacle_right_c[0], obstacle_right_c[1])), trans_to_pyg_cord((obstacle_right_d[0], obstacle_right_d[1])),5)
    pygame.draw.line(screen, (200, 255, 0), trans_to_pyg_cord((obstacle_right_d[0], obstacle_right_d[1])), trans_to_pyg_cord((obstacle_right_a[0], obstacle_right_a[1])),5)
    
    # Draw position and orientation
    pygame.draw.circle(screen, (0, 0, 255), (pos_x_true + phi_true_circ_x_true, pos_y_true - phi_true_circ_y_true), 6, 5)
    pygame.draw.circle(screen, (0, 255, 0), (pos_x_true, pos_y_true), 10, 5)
    
    
                    
        
            
    pygame.display.update()


def main():

    cap = cv2.VideoCapture(1)

    #168*2
    #94
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1344);
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,376);
    
    while True:
        
        net = SqueezeNet()
        epoch = ARGS.epoch
        save_data = torch.load(os.path.join(ARGS.save_path, "epoch%02d.weights" % (epoch - 1,)), map_location=lambda storage, loc: storage)
        net.load_state_dict(save_data)
        
        net.eval()

        nframes = 2
        list_camera_input = []
        
        for t_moment in range(nframes):
            
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            # Our operations on the frame come here
            capture_one = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            #print frame.shape
            # Take only the left half
            left = capture_one[0:376,0:1344/2]
            
            # Take only the left half
            right = capture_one[0:376,1344/2:-1]
            
            # Resize to 168x94
            left = imresize(left,(94,168),'nearest')
            right = imresize(right,(94,168),'nearest')
                        
            for camera in (left,right):                
                list_camera_input.append(torch.from_numpy(camera))        
        
                camera_data = torch.cat(list_camera_input, 2)        
                camera_data = camera_data.float() / 255. - 0.5        
                camera_data = torch.transpose(camera_data, 0, 2)        
                camera_data = torch.transpose(camera_data, 1, 2)
                
        
        
        
        
        #camera_data = torch.unsqueeze(camera_data,0)
        
        outputs = net(Variable(camera_data))
        
        
        open_cv_image = np.array(left) 
        # Convert RGB to BGR 
        open_cv_image = open_cv_image[:, :, ::-1].copy()
        

        height,width,channels = open_cv_image.shape

        draw_map(open_cv_image, width, height, None, None, None, outputs, None)

        
        final_image = imresize(open_cv_image,8.0,'bilinear')
        
        cv2.imshow("Evaluation",final_image)
        
        cv2.waitKey(1)
    



if __name__ == '__main__':
    main()
