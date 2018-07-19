#!/usr/bin/env python
import rospy
from bdd_driver import neural_network, controls_aggregator
NeuralNetwork = neural_network.NeuralNetwork
Aggregator = controls_aggregator.Aggregator
from params import params
import bdd.msg as BDDMsg
from multiprocessing import Process

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

def main():
    rospy.init_node('bdd_driver')
    rospy.logdebug('starting bdd driver node')

    while not rospy.is_shutdown():
	
	img_sub = rospy.Subscriber("/zed/depth/depth_registered", sensor_msgs/Image, callback())

def callback(data):
    # Give data to preprocessing class to add new image to queue of 10 images
    # run nueral network
    # publish NN output to rostopic

if __name__ == '__main__':
    main()
