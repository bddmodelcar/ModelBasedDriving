#!/usr/bin/env python
import rospy
import bdd.msg as BDDMsg
from multiprocessing import Process, Queue
from Queue import Empty # Regular Queue, not multiprocessing's Queue
from sensor_msgs.msg import Image

# add parent of parent directory to path so files these files can be imported
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from nn.nn_runner import init_model, process_input
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
    return_value_q = Queue()
    
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
    
    
    
    # invokes a neural network process if one is not currently running
    # if one is running, then only send data for recording to be used next time a NN will run
    def cam_callback(self, car_data):

        #print('~~ received image ~~')

        # if no NN process running, start a new NN process
        if not Driver.current_process.is_alive():
        
            # Queue is used for only one tuple of data: (speed, direction)
            #return_value_q = Queue()
            
            print('STARTING NEW PROCESS')
            Driver.current_process = Process(target = process_input, args = (car_data, True, Driver.return_value_q))
            Driver.current_process.start()
            Driver.current_process.join()
            
            try:
                self.send_to_car(Driver.return_value_q.get(False))
            except Empty:
                print('queue is empty')
                pass
            
        # else, don't start a new one, but record the new car data
        else:
            process_input(car_data, False, None)
            



    # publishes car controls on /controls topic
    def send_to_car(self, nn_output):

        # TODO: Format the nn_output into steer and speed. Similar code to run_model in Network.py
        speed = nn_output[0] #data.speed
        direction = nn_output[1] #data.direction
        Driver.control_pub.publish(BDDMsg.BDDControlsMsg(speed=speed, direction=direction))
        
        


if __name__ == '__main__':
    rospy.init_node('driver')
    try:
    	Driver()
    except rospy.ROSInterruptException:
    	pass
