#!/usr/bin/env python
#import os
#import threading
#import time
import rospy
import bdd.msg as BDDMsg
import arduino_utils as utils
from params import params


class CarControl():
    def __init__(self):
        if not self.init_hardware():
            print('failed to initialized car hardware')
            exit(1)
        self.speed = 0 # -1 = max speed back, 0 = stop, 1 = max speed forward
        self.direction = 0 # -1 = left, 0 = straight, 1 = right
        self.state = 0
        rospy.init_node('carControl')
        rospy.Subscriber('controls', BDDMsg.BDDControlsMsg, callback=self.controls_callback)
        rospy.spin()
        print('exiting bdd car node')

    def __del__(self):
        self.shutdown_hardware()

    def init_hardware(self):
        return utils.init()

    def shutdown_hardware(self):
        utils.shutdown()

    def update_output(self):
        utils.write(speed=self.speed, direction=self.direction)


    def controls_callback(self, controls_msg):
        print('/\/\/\/\/GOT SUM INPUT!\/\/\/\/', controls_msg.speed, controls_msg.direction)
        self.speed = controls_msg.speed
        self.direction = controls_msg.direction
        self.update_output()
    

if __name__ == '__main__':
    
    try:
        car = CarControl()
    except rospy.ROSInterruptException: 
        pass
        
