"""Training and validation code for bddmodelcar."""
import sys
import traceback
import logging


from Parameters import ARGS
from Dataset import Dataset
import Utils
from argparse import ArgumentError

cpu_mode = ARGS.cpu_mode
log_loss_histgrm_mode = True
vizualize = False

if vizualize:
    import cv2

if vizualize:
    from torchvision.transforms import ToPILImage
    from scipy.misc.pilutil import imresize
    
    to_img = ToPILImage()

import h5py
import os

from nets.SqueezeNet10 import SqueezeNet
from torch.autograd import Variable
import torch.nn.utils as nnutils
import torch
import numpy as np

if log_loss_histgrm_mode:
    if not ARGS.data_logfile:
        raise ArgumentError(ARGS.data_logfile,'Logging mode was on but no hdf5 logfile name provided.')
        exit()
    h5py_log_file = ARGS.data_logfile

dataset_path = ARGS.data_path
steering_values = []
truth_values = []
loss_values = []

def meta_callback(data):
    global metadata_choice
    
    print "Choice is now " + str(data.data)
    metadata_choice = int(data.data)

def draw_outputs(open_cv_image, outputs, truth):
    
    steering = -(outputs[0][0:9].cpu().data - 0.5)
    truth_steering = -(truth[0][0:9].cpu().numpy() - 0.5)
    motor = outputs[0][10:20]
    
    height, width, channels = open_cv_image.shape
    
    scale_factor = 100
    
    v_space = 4
    h_pos = height / 10
    
    for i in range(len(steering)):
       
        h_pos += v_space
        
        from_pt = (int(width/2),h_pos)
        to_pt = (int(width/2+steering[i]*scale_factor),h_pos)
        
        cv2.line(open_cv_image,from_pt,to_pt,(0,0,255),thickness=4)
        
        h_pos += v_space
        
        from_pt = (int(width/2),h_pos)
        to_pt = (int(width/2+truth_steering[i]*scale_factor),h_pos)
        
        cv2.line(open_cv_image,from_pt,to_pt,(0,255,0),thickness=4)
        

def log_results(outputs, truth, loss_value):
    
    global steering_values
    global truth_values
    global loss_values
    
    # Come to terms with strange bug, trying to log empty tensors
    new_steering_values = torch.squeeze(outputs.cpu().data).numpy()
    
    if len(np.array(new_steering_values.shape)) < 2:
        print "Bug averted {}".format(np.array(new_steering_values).shape)
        return
    
    steering_values.append(torch.squeeze(outputs.cpu().data).numpy())
    truth_values.append(torch.squeeze(truth.cpu()).numpy())
    loss_values.append(loss_value.data.cpu().numpy())
    
    print "Size steering {}".format(np.array(steering_values).shape)
    print "Size loss {}".format(np.array(loss_values).shape)
    
def write_out_log(epoch_no):
    
    global steering_values
    global truth_values
    global loss_values
    
    h5py_file = h5py.File(h5py_log_file+"Epoch"+str(epoch_no)+".h5py", 'w')
    
    print "Size steering {}".format(np.array(steering_values).shape)
    print "Size loss {}".format(np.array(loss_values).shape)
    
    steering_dset = h5py_file.create_dataset("steering", np.array(steering_values).shape, dtype='f')
    truth_dset = h5py_file.create_dataset("truth", np.array(truth_values).shape, dtype='f')
    loss_dset = h5py_file.create_dataset("loss", np.array(loss_values).shape, dtype='f')
    
    steering_dset[...] = steering_values
    truth_dset[...] = truth_values
    loss_dset[...] = loss_values
    
    h5py_file.close()
    
    # Setting the lists to empty again is not necessary right now but if ever
    # more than one epoch should be compared here. This is useful 
    steering_values = []
    truth_values = []
    loss_values = []
    

def main():
    global metadata_choice
    logging.basicConfig(filename='training.log', level=logging.DEBUG)
    logging.debug(ARGS)  # Log arguments
 
    # Set Up PyTorch Environment
    # torch.set_default_tensor_type('torch.FloatTensor')
    if cpu_mode == False:
        torch.cuda.set_device(ARGS.gpu)
        torch.cuda.device(ARGS.gpu)
        net = SqueezeNet().cuda()
        criterion = torch.nn.MSELoss().cuda()
    elif cpu_mode == True:
        print "Warning. CPU Mode chosen"
        net = SqueezeNet().cpu()
        criterion = torch.nn.MSELoss().cpu()
    else:
        print "CPU mode parameter set to weird state: {}".format(cpu_mode)
        exit()
        
    
    optimizer = torch.optim.Adadelta(net.parameters())

    try:
        epoch = ARGS.epoch

        if not epoch == 0:
            import os
            print("Resuming")
            
            save_data = torch.load(os.path.join(ARGS.save_path, "epoch%02d.weights" % (epoch - 1,)), map_location=lambda storage, loc: storage)
            #save_data = torch.load(os.path.join(ARGS.save_path, "epoch%02d_full_db.weights" % (epoch - 1,)), map_location=lambda storage, loc: storage)
            
            net.load_state_dict(save_data)

        logging.debug('Starting training epoch #{}'.format(epoch))

        net.eval()  # Evaluation mode

        train_dataset = Dataset(dataset_path, cpu_mode)
        #train_dataset = Dataset("/media/sascha/rosbags/DatabaseWithPosGoal/processed_h5py/training", cpu_mode)
        
        train_data_loader = torch.utils.data.DataLoader(train_dataset,
                                                        batch_size=500,
                                                        shuffle=True, pin_memory=False)


        i = 0

        for camera, truth in train_data_loader:                
            
            if vizualize:
                depth_t1 = camera[0].clone().cpu()
                depth_t1 = torch.unsqueeze(depth_t1[0],0)
                depth_t1 = (depth_t1 + 0.5)
                depth_t1 = to_img(depth_t1)
               
                depth_t2 = camera[0].clone().cpu()
                depth_t2 = torch.unsqueeze(depth_t2[1],0)
                depth_t2 = (depth_t2 + 0.5)
                depth_t2 = to_img(depth_t2)
                            
            if cpu_mode == False:
                outputs = net(Variable(camera)).cuda()
            elif cpu_mode == True:
                outputs = net(Variable(camera)).cpu()
            
            loss = criterion(outputs, Variable(truth))
            
            if vizualize:
                open_cv_image_t1 = np.array(depth_t1)
                open_cv_image_t2 = np.array(depth_t2)
             
                open_cv_image = np.concatenate((open_cv_image_t1,open_cv_image_t2),0)
                
                #open_cv_image = open_cv_image[:, :, ::-1].copy()
                open_cv_image = cv2.cvtColor(open_cv_image,cv2.COLOR_GRAY2RGB)
                draw_outputs(open_cv_image, outputs, truth)
                
                final_image = imresize(open_cv_image, 4.0, 'bilinear')
            
                cv2.imshow("Evaluation", final_image)
                cv2.waitKey(1)
            
            if log_loss_histgrm_mode:
                log_results(outputs,truth, loss)
             
            # Debug for quick tests   
            if i >= 300:
                break

            if i % 10 == 0:
                print "Iteration " + str(i)
                logging.debug("Current iteration {}".format(i))
            #if i == Dataset.max_iterations:
                # Perform the exact same amount of iterations in every set. Important for outdoor data
                # Careful. This is dependent on the batchsize
            #    break    
            
            i += 1
            
        if log_loss_histgrm_mode:
            write_out_log(epoch)

    except Exception:
        logging.error(traceback.format_exc())  # Log exception
        print "Exception occured " + str(traceback.format_exc())
        sys.exit(1)

if __name__ == '__main__':
    main()
