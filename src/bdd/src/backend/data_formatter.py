from collections import deque
import numpy as np
import torch
from torch.autograd import Variable


'''data_formatter.py
This file is meant to be edited 
by the user. Driver.py uses the
functions in this file to format
all input and output data so that
it fits whatever model is used.

Code which you may want to edit:
- format_input_data()
- format_output_data()

'''

class DataFormatter():

    num_frames = 10
    
    # Create scaling layer
    scale = torch.nn.AvgPool2d(kernel_size=3, stride=2, padding=1).cuda()

    left_imgs = deque(maxlen = num_frames)
    right_imgs = deque(maxlen = num_frames)
    
    input_data = []
    output_data = []
        
        
        
        
    @classmethod
    def enough_images_queued(cls):
        return len(cls.left_imgs) == cls.num_frames and len(cls.right_imgs) == cls.num_frames
        
        
        
        
    @classmethod
    def format_left_input(cls, left_img):
        img = np.frombuffer(left_img.data, dtype=np.uint8).reshape(left_img.height, left_img.width, -1)
        cls.left_imgs.append(torch.from_numpy(img))



        
    @classmethod
    def format_right_input(cls, right_img):
        img = np.frombuffer(right_img.data, dtype=np.uint8).reshape(right_img.height, right_img.width, -1)
        cls.right_imgs.append(torch.from_numpy(img))



        
    @classmethod
    def format_input_data(cls):
        camera_data = []
        for i in range(cls.num_frames):
            camera_data.append(cls.left_imgs[i])
            camera_data.append(cls.right_imgs[i])
            
        camera_data = torch.cat(camera_data, 2)
        camera_data = camera_data.cuda().float() / 255. - 0.5
        camera_data = torch.transpose(camera_data, 0, 2) 
        camera_data = torch.transpose(camera_data, 1, 2)
        camera_data = camera_data.unsqueeze(0)
        camera_data = Variable(camera_data)
        camera_data = cls.scale(camera_data)
        camera_data = cls.scale(camera_data)
        cls.input_data = camera_data
        return




    @classmethod
    def format_output_data(cls, output_data):
        # choose which timeframes of steer and throttle to use
        # or whether to do smoothing or whatnot
        output = output_data.detach().cpu().numpy()
        steer = output[0][:10]
        throttle = output[0][10:]
        
        steer = np.average(steer) * 100
        throttle = np.average(throttle) * 100
        cls.output_data = (steer,throttle)
        return

        
    
    
    @classmethod
    def get_input_data(cls):
        return cls.input_data
        
        
        
        
    @classmethod
    def get_output_data(cls):
        return cls.output_data



