import rospy
import bdd.msg as BDDMsg
#import params # probably bad
from sensor_msgs.msg import Image
#import torch
import time


"""Neural Network class
Run a neural network based on the
parameters in the dictionary model_info.

This is framework agnostic; it does not
matter whether you use pytorch,
tensorflow, numpy, caffe, or raw C
to implement your neural network, you
just need to implement a simple python module
that can be called from output_controls and
returns a speed and direction given
image input. For less popular frameworks
you will be passed a numpy array and
must handle the conversion yourself


nn_runner: 
	-init node and nn, subs to car_data

image_callback: 
	-called when car_data received. 
	-Sends data to processing 
	-receives processed data and runs the model


May be useful to know about spinning and callbacks (e.g. they say not to run long tasks in callback)
https://answers.ros.org/question/53055/ros-callbacks-threads-and-spinning/
"""

# from kzpy3.2/.../bair_car/nodes/network.py
def init_model():
    
    #
    print('fake initialization of [NN]')
    pass
    #
    
    '''
    
   	#global solver, scale, nframes
   	
    #info = params.model_info
    #model = info['full  _path']
    
	
    # Load PyTorch model
    save_data = torch.load(weight_file_path)
    print("Loaded " + weight_file_path)
    # Initializes Solver
    solver = SqueezeNet().cuda()
    
    solver.load_state_dict(save_data['net'])
    solver.eval()
    nframes = solver.N_FRAMES
    
    # Create scaling layer
    scale = nn.AvgPool2d(kernel_size=3, stride=2, padding=1).cuda()
    
    '''
    

def input_from_driver(car_data, run_nn = False):
    # send new car_data to data_processor
    # new_queued_data = data_processor.add_new_data(car_data)
    
    if run_nn:
        print('fake running [NN]...') # or just sleep instead of a loop...
        time.sleep(1)
        
    #
        print("ran NN")
    return
    #
    
    
    #    run_model(new_queued_data)
    

def run_model(processed_data):
	# get camera_data and metadata from `processed_data` input

	raw_output = solver(some_input, Variable(metadata))  # Run the neural net
	return raw_output




#def image_callback(car_data):
	#TODO: send new car_data to data processing and receive back list of 10 images
#        raw_output = run_model(processed_data)
#        nn_raw_output.publish(raw_output with_some_message_type)




#def nn_runner():


#	rospy.init_node('nn_runner')

	# subscribe to car_data and discard any older data
#        rospy.Subscriber('/bdd/car_data', BDDMsg.some_msg, image_callback, queue_size = None)

	# init publisher for nn raw output
#	nn_raw_output = rospy.Publisher('nn_raw_output', BDDMsg.BDDControlsMsg, queue_size=None)

#	rospy.spin()




#if __name__ == '__main__':
#    try:
    	#nn_runner()
#    except rospy.ROSInterruptException:
#    	pass

    
