#!/usr/bin/env python
#import os
#import threading
#import time
import rospy
import bdd.msg as BDDMsg
import arduino_utils as utils
from params import params


class Car():
    def __init__(self):
        if not self.init_hardware():
            print('failed to initialized car hardware')
            exit(1)
        self.speed = 0 # -1 = max speed back, 0 = stop, 1 = max speed forward
        self.direction = 0 # -1 = left, 0 = straight, 1 = right
        self.state = 0

    def __del__(self):
        self.shutdown_hardware()

    def init_hardware(self):
        utils.init()

    def shutdown_hardware(self):
        utils.shutdown()

    def update_output(self):
        utils.write(speed=self.speed, direction=self.direction)


def controls_callback(controls_msg):
    car.speed = controls_msg.speed
    car.direction = controls_msg.direction
    car.state = controls_msg.state
    car.update_output()
    
def car_inputs():
    print('starting bdd car node')
    rospy.init_node('car_inputs')
    rospy.Subscriber('controls', BDDMsg.BDDControlsMsg, callback=controls_callback)
    rospy.spin()
    print('exiting bdd car node')

if __name__ == '__main__':
    
    try:
        car_inputs()
        car = Car()
    except rospy.ROSInterruptException: 
        pass
        
