#import rospy
#import bdd.msg as BDDMsg
#from sensor_msgs.msg import Image
import time
import torch
import os
from Networks.SqueezeNet10 import SqueezeNet


solver = None


# from kzpy3.2/.../bair_car/nodes/network.py
def init_model():

    global solver
    directory_path = os.path.dirname(__file__)
    #rel_path = 'Networks/net_for_nitzan/save/epoch00.weights'
    rel_path = 'Networks/nitzan_test_weights/1/test_weights_13.weights'
    #rel_path = 'Networks/nitzan_test_weights/2/tests_epoch13.weights'
    
    abs_path = os.path.join(directory_path, rel_path)

    # Initializes Solver
    solver = SqueezeNet().cuda()

    # Load PyTorch model
    save_data = torch.load(abs_path)
    solver.load_state_dict(save_data)
    solver.eval()

    
def run_inference(formatted_data, return_value_q):
    raw_output = solver(formatted_data)  # Run inference
    return_value_q.put(raw_output)
    return
    

	




    
