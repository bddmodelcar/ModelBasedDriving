"""Training and validation code for bddmodelcar."""
import sys
import traceback
import logging

from Parameters import ARGS
from Dataset import Dataset
import Utils

from nets.SqueezeNet10 import SqueezeNet
from torch.autograd import Variable
import numpy as np
import torch.nn.utils as nnutils
import torch
from torchvision.transforms import ToPILImage
from PIL import Image
to_img = ToPILImage()
import time

def main():
    logging.basicConfig(filename='training.log', level=logging.DEBUG)
    logging.debug(ARGS)  # Log arguments

    # Set Up PyTorch Environment
    # torch.set_default_tensor_type('torch.FloatTensor')
    torch.cuda.set_device(ARGS.gpu)
    torch.cuda.device(ARGS.gpu)

    net = SqueezeNet().cuda()
    criterion = torch.nn.MSELoss().cuda()
    optimizer = torch.optim.Adadelta(net.parameters())

    try:
        epoch = ARGS.epoch

        if not epoch == 0:
            import os
            print("Resuming")
            save_data = torch.load(os.path.join(ARGS.save_path, "epoch%02d.weights" % (epoch - 1,)))
            net.load_state_dict(save_data)

        logging.debug('Starting training epoch {0} at {1}'.format(epoch,time.ctime()))

        net.train()  # Train mode

        train_dataset = Dataset(ARGS.data_path, False,10)
        train_data_loader = torch.utils.data.DataLoader(train_dataset,
                                                        batch_size=500,
                                                        shuffle=False, pin_memory=False)

        train_loss = Utils.LossLog()

        training_cycle = 0

        for camera, truth in train_data_loader:
            
            training_cycle += 1
            
            camera = camera.cuda()
            truth = truth.cuda()
            
            # Forward
            optimizer.zero_grad()
            
            outputs = net(Variable(camera)).cuda()

            loss = criterion(outputs, Variable(truth))

            # Backpropagate
            loss.backward()
            nnutils.clip_grad_norm(net.parameters(), 1.0)
            optimizer.step()

            # Logging Loss
            train_loss.add(loss.data[0])
            
            print "Iteration {0}".format(training_cycle)
            print "Loss {0}".format(loss.data[0])

        Utils.csvwrite('trainloss.csv', [train_loss.average()])

        logging.debug('Finished training epoch #{}'.format(epoch))
        
        logging.debug('Saving net #{}'.format(epoch))
        
        Utils.save_net("epoch%02d" % (epoch,), net)
            

    except Exception:
        logging.error(traceback.format_exc())  # Log exception
        print "Exception occured " + str(traceback.format_exc())
        sys.exit(1)

if __name__ == '__main__':
    main()
