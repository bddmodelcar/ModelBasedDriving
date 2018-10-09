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
from nets.SqueezeNetWithPos import SqueezeNet
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


class GoalCode(Enum):
    left = 0
    center = 1
    right = 2
    no_goal = 3


motor_gain = 1.
steer_gain = 1.

# motor_gain = 1.
# steer_gain = 1.

last_time = 0

verbose = False

nframes = 2 

wall_max_x = Dataset.wall_max_x
wall_max_y = Dataset.wall_max_y

map_width_meter = wall_max_x
map_height_meter = wall_max_y
linewidth = 1

white = (255,255,255)

size = 400,1000

pygame.init()
screen = pygame.display.set_mode(size)
screen.fill(white)

#pygame.draw.rect(screen, (255,0,0), (0,0,400,1000), 10)


def draw_map(open_cv_image, width, height, pos_val_x, pos_val_y, pos_val_x_true, pos_val_y_true, pos_val_phi, pos_val_phi_true):
    
    # Draw network output
    screen.fill(white)
    pos_x = int((400/wall_max_x)*pos_val_x)
    pos_y = 1000-int((1000/wall_max_y)*pos_val_y)
    
    phi_circ_d = 20
    phi_circ_x = int(np.cos(pos_val_phi) * phi_circ_d)
    phi_circ_y = int(np.sin(pos_val_phi) * phi_circ_d)
        
    pygame.draw.circle(screen, (0,255,0), (pos_x+phi_circ_x,pos_y-phi_circ_y), 6, 5)
    pygame.draw.circle(screen, (255,0,0), (pos_x,pos_y), 10, 5)
    
    
    
    # Draw ground truth
    
    pos_x_true = int((400/wall_max_x)*pos_val_x_true)
    pos_y_true = 1000-int((1000/wall_max_y)*pos_val_y_true)
    
    phi_true_circ_d = 20
    phi_true_circ_x_true = int(np.cos(pos_val_phi_true) * phi_true_circ_d)
    phi_true_circ_y_true = int(np.sin(pos_val_phi_true) * phi_true_circ_d)
        
    pygame.draw.circle(screen, (0,0,255), (pos_x_true+phi_true_circ_x_true,pos_y_true-phi_true_circ_y_true), 6, 5)
    pygame.draw.circle(screen, (0,255,0), (pos_x_true,pos_y_true), 10, 5)
    
    pygame.display.update()

def main():
    logging.basicConfig(filename='training.log', level=logging.DEBUG)
    logging.debug(ARGS)  # Log arguments

    # Set Up PyTorch Environment
    # torch.set_default_tensor_type('torch.FloatTensor')
    #torch.cuda.set_device(ARGS.gpu)
    #torch.cuda.device(ARGS.gpu)

    net = SqueezeNet()
    criterion = torch.nn.MSELoss()
    optimizer = torch.optim.Adadelta(net.parameters())

    try:
        epoch = ARGS.epoch

        #print("Resuming")
        save_data = torch.load(os.path.join(ARGS.save_path, "epoch%02d.weights" % (epoch - 1,)), map_location=lambda storage, loc: storage)
        net.load_state_dict(save_data)

        #logging.debug('Starting training epoch #{}'.format(epoch))

        net.eval()
             
        val_dataset = Dataset('/media/sascha/rosbags/DatabaseWithPosGoal/processed_h5py/training/', ARGS.require_one, ARGS.ignore)
        val_data_loader = torch.utils.data.DataLoader(val_dataset,
                                                        batch_size=1,
                                                        shuffle=False, pin_memory=False)
 
        val_loss = Utils.LossLog()
        
        logging.debug('Starting validation epoch #{}'.format(epoch))
        for camera, truth in val_data_loader:
            
            
            to_img = ToPILImage()
            camera_data = camera[0].clone().cpu()
            
            camera_data = torch.squeeze(camera_data)[0:3]
            
            camera_data = (camera_data+0.5)
            
            camera_data = to_img(camera_data)
                        
            # Forward
            optimizer.zero_grad()
            
            
            
            outputs = net(Variable(camera))
            
            pos_val_x = outputs[0][0].data[0] * wall_max_x
            pos_val_y = outputs[0][1].data[0] * wall_max_y
            
            true_x = truth[0][0] * wall_max_x
            true_y = truth[0][1] * wall_max_y
            
            pos_val_phi = ((outputs[0][2].data[0]*2.)-1)*np.pi
            phi_true = ((truth[0][2]*2.)-1)*np.pi
            
            
            width, height =  camera_data.size
            
            
            
            camera_data.convert('RGB') 
            open_cv_image = np.array(camera_data) 
            # Convert RGB to BGR 
            open_cv_image = open_cv_image[:, :, ::-1].copy()
            
            bar_range = width / 2
            
#             # Network steering output
#             steer_val = outputs[0][9].data[0]
#             steer_val = steer_val - 0.5
#             minimum = int(min(width/2,(width / 2) + (steer_val * bar_range)))
#             maximum = int(max(width/2,(width / 2) + (steer_val * bar_range)))
#             open_cv_image[height-20:height-10,minimum:maximum] = [0,0,255]
#             
#             # Ground truth steering output
#             steer_val_truth = truth[0][9]
#             steer_val_truth = steer_val_truth - 0.5
#             minimum = int(min(width/2,(width / 2) + (steer_val_truth * bar_range)))
#             maximum = int(max(width/2,(width / 2) + (steer_val_truth * bar_range)))
            
#             open_cv_image[height-30:height-20,minimum:maximum] = [0,255,0]
            
            
            
            
            draw_map(open_cv_image,width, height, pos_val_x, pos_val_y, true_x, true_y, pos_val_phi,phi_true)
            
            final_image = imresize(open_cv_image,8.0,'bilinear')
            
            cv2.imshow("Evaluation",final_image)
            
            cv2.waitKey(1)
            
             
            
 
            loss = criterion(outputs, Variable(truth))
            
            # Logging Loss
            val_loss.add(loss.data[0])
 
        Utils.csvwrite('valloss.csv', [val_loss.average()])
 
        logging.debug('Finished validation epoch #{}'.format(epoch))
         

    except Exception:
        logging.error(traceback.format_exc())  # Log exception
        print "Exception occured " + str(traceback.format_exc())
        sys.exit(1)



if __name__ == '__main__':
    main()
