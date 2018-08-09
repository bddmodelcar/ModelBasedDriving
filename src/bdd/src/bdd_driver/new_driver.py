#!/usr/bin/env python
import rospy
from bdd_driver import neural_network
NeuralNetwork = neural_network.NeuralNetwork
#Aggregator = controls_aggregator.Aggregator
from params import params
import bdd.msg as BDDMsg
#from multiprocessing import Process

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



def cam_callback(data):

    # TODO: publish data to car_data
    car_data_pub.publish(data)




def nn_output_callback(data):

    # TODO: Format the nn_output into steer and speed. Similar code to run_model in Network.py
    speed = 0 #data.speed
    direction = 0 #data.direction
    control_pub.publish(BDDMsg.BDDControlsMsgToDriver(speed=speed, direction=direction))




def driver():
    rospy.init_node('driver')
    rospy.logdebug('starting bdd driver node')

    control_pub = rospy.Publisher('controls', BDDMsg.BDDControlsMsg, queue_size=None)
    car_data_pub = rospy.Publisher('car_data', <car_data_message_type>, queue_size = None)
    rospy.Subscriber('/zed/depth/depth_registered', sensor_msgs/Image, cam_callback, queue_size = None)
    rospy.Subscriber('/bdd/nn_raw_output', <some_msg_type>, nn_output_callback, queue_size = None)     




if __name__ == '__main__':
    try:
    	driver()
    except rospy.ROSInterruptException:
    	pass
