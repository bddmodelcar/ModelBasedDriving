#!/usr/bin/env python
import rospy
import bdd.msg as BDDMsg
from multiprocessing import Process, Queue
from Queue import Empty # Regular Queue, not multiprocessing's Queue
from sensor_msgs.msg import Image

# add parent of parent directory to path so these files can be imported
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
"""




class Driver():

    current_process = Process()
    nn_output = Queue() # Queue used for getting return data from separate process
    formatter = DataFormatter() 
    
    #net = None

    def __init__(self):
        rospy.logdebug('starting bdd driver node')
        print('starting [DRIVER] node')

        
        # initialize the NN that driver will call
        #info = params.model_info
        #model = info['full_path'] # don't know where to use this yet
        init_model() 

        Driver.controls_pub = rospy.Publisher('controls', BDDMsg.BDDControlsMsg, queue_size=None)
        rospy.Subscriber('/zed/depth/depth_registered', Image, self.cam_callback, queue_size = 10)
        Driver.cam_callback(self, None)
        rospy.spin()
    
    
    '''
    Invokes a neural network inference process when new car_data arrives.
    But if there's already a NN busy inferring, then don't start another inference.
    '''
    def cam_callback(self, car_data):

        nn_input = formatter.format_input_data(car_data)

        # if no NN process currerntly running, start a new NN process
        if not Driver.current_process.is_alive():
        
            # Queue is used for only one tuple of return data: (speed, direction)
            print('STARTING NEW PROCESS')
            Driver.current_process = Process(target = process_input, args = (car_data, Driver.nn_output))
            Driver.current_process.start()
            Driver.current_process.join()
            
            try:
                car_controls = formatter.format_output_data(Driver.nn_output.get(False))
                self.send_to_car(car_controls)
            except Empty:
                print('queue is empty -- no data to send to car')
            



    # publishes car controls on "/controls" topic
    def send_to_car(self, nn_output):

        # TODO: Format the nn_output into steer and speed. Similar code to run_model in Network.py
        speed = nn_output[0] #data.speed
        direction = nn_output[1] #data.direction
        Driver.controls_pub.publish(BDDMsg.BDDControlsMsg(speed = speed, direction = direction))
        
        


if __name__ == '__main__':
    rospy.init_node('driver')
    try:
    	Driver()
    except rospy.ROSInterruptException:
    	pass
