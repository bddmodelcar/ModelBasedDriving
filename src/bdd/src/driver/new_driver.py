#!/usr/bin/env python
import rospy
import bdd.msg as BDDMsg
from multiprocessing import Process
from sensor_msgs.msg import Image

# add parent of parent directory to path so files these files can be imported
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from nn.nn_runner import init_model, input_from_driver
from params import params



"""Main node
This is the driver ros node,
the main entry-point for car
control generation. For information
on actually sending controls to
the hardware and other hw/sw
interface code, see car.py and
the bdd_car module

The driver node is responsible for spawning
neural network processes for each
set of parameters as returned by
params.models (see the params module)

In addition to starting the
neural networks, this initializes the
zed camera and takes care of publishing
images to a ros topic, to which the
neural network processes should subscribe
"""




class Driver():

    current_process = Process()
    #net = None

    def __init__(self):
        rospy.logdebug('starting bdd driver node')
        print('starting [DRIVER] node')

        
        # initialize the NN that driver will call
        #info = params.model_info
        #model = info['full_path'] # don't know where to use this yet
        init_model() 

        Driver.control_pub = rospy.Publisher('controls', BDDMsg.BDDControlsMsg, queue_size=None)
        rospy.Subscriber('/zed/depth/depth_registered', Image, self.cam_callback, queue_size = 10)
        Driver.cam_callback(self, None)
        rospy.spin()
    
    
    
    
    def cam_callback(self, data):

        print('~~ received image ~~')

        # if no NN process running, start a new NN process
        if not Driver.current_process.is_alive():
            print('-*-*-*-*STARTING NEW PROCESS*-*-*-*-')
            Driver.current_process = Process(target = input_from_driver, args = (data, True))
            Driver.current_process.start()
            
        # else, don't start a new one, but record the new car data
        else:
            print('... NOT starting another ...')
            input_from_driver(data, False)
            
        
        print
        
        #
        #send_to_car(None) 
        #    




    # TODO: rename function since it's not a callback anymore
    def send_to_car(self, data):

        # TODO: Format the nn_output into steer and speed. Similar code to run_model in Network.py
        speed = 0 #data.speed
        direction = 0 #data.direction
        
        #TODO: make sure this works:
        Driver.control_pub.publish(BDDMsg.BDDControlsMsg(speed=speed, direction=direction))
        print('published controls on topics \'controls\' ')
        
        


if __name__ == '__main__':
    rospy.init_node('driver')
    try:
    	Driver()
    except rospy.ROSInterruptException:
    	pass
