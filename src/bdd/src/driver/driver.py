#!/usr/bin/env python
import rospy
import bdd.msg as BDDMsg
from multiprocessing import Process, Queue
from Queue import Empty # Regular Queue, not multiprocessing's Queue
from sensor_msgs.msg import Image

# add parent of parent directory to path so these files can be imported
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from backend.nn_runner import init_model, run_inference
from backend.data_formatter import DataFormatter
from params import params
import numpy as np
from rospy.numpy_msg import numpy_msg
import time
from threading import Thread


"""Main node
This is the driver ros node,
the main entry-point for car
control generation. The driver node 
is responsible for spawning neural network 
processes and sending commands to the car.

General order of operations:
1. Receives left and right images
2. Uses DataFormatter (DF) to store them
3. DF.format_input_data
4. DF.get_input_data
5. Runs inference process
6. DF.format_output_data
7. DF.get_output_data
8. Sends formatted output data to car
"""




class Driver():

    current_process = Thread()
    


    def __init__(self):
        print('starting [DRIVER] node')

        init_model()
        
        Driver.controls_pub = rospy.Publisher('controls', BDDMsg.BDDControlsMsg, queue_size = None)
        rospy.Subscriber('/zed/left/image_rect_color', numpy_msg(Image), self.left_callback, queue_size = 5)
        rospy.Subscriber('/zed/right/image_rect_color', numpy_msg(Image), self.right_callback, queue_size = 5)

    
    
    def left_callback(self, data):
        DataFormatter.format_left_input(data)
    
    
    def right_callback(self, data):    
        DataFormatter.format_right_input(data)
        
    '''
    Invokes a neural network inference process when new car_data arrives.
    But if there's already a NN busy inferring, then don't start another inference.
    '''    
    def run(self):
    
        while not DataFormatter.enough_images_queued():
            continue
    
        while not rospy.is_shutdown():
        
            # if no NN process currerntly running, start a new NN process
            if not Driver.current_process.is_alive():

		nn_output = Queue() # Queue used for getting return data: (steer, throttle)
                DataFormatter.format_input_data()
                car_data = DataFormatter.get_input_data()
                Driver.current_process = Thread(target = run_inference, args = (car_data, nn_output))
                Driver.current_process.start()
                Driver.current_process.join() # makes code below wait for nn_output. Doesn't block other funcs
                try:
                    raw_output = nn_output.get(True)
                except Empty:
                    print('queue is empty -- no data to send to car')
                else:
                    DataFormatter.format_output_data(raw_output)    
                    car_controls = DataFormatter.get_output_data()
                    self.send_to_car(car_controls)
        



    # publishes car controls on "/controls" topic
    def send_to_car(self, car_controls):
        steer = car_controls[0] # first part of output
        throttle = car_controls[1] # second part of output
        Driver.controls_pub.publish(BDDMsg.BDDControlsMsg(steer = steer, throttle = throttle))
        
        


if __name__ == '__main__':
    rospy.init_node('driver')
    try:
    	Driver().run()
    except rospy.ROSInterruptException:
    	exit()
