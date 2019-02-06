#!/usr/bin/env python

import serial
from params import params
import time
import rospy
import bdd.msg as BDDMsg
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import sys
from collections import deque

''' arduino_car.py
This file takes in commands from
driver.py and, depending on the state,
decides what commands to send to the 
arduino car. Commands are give either
by the neural network or by the human
via the remote controller. A running average
ensures that there are no sharp changes
in the commands being sent to the car.
Speed limiting is also implemented here.
'''

class ArduinoCar():

    arduino = None
    NN_data = None
    steer_calibration = deque(maxlen = 10)
    throttle_calibration = deque(maxlen = 10)
    steer_running_avg = deque(maxlen = params.running_avg_len)
    throttle_running_avg = deque(maxlen = params.running_avg_len)
    steer_offset = 0
    throttle_offset = 0
    img_num = 0
    
    #state_pub = rospy.Publisher('state', Int32, queue_size=5)
    #steer_used_pub = rospy.Publisher('steer_used', Int32, queue_size=5)
    #throttle_used_pub = rospy.Publisher('throttle_used', Int32, queue_size=5)
    car_info_pub = rospy.Publisher('car_info', BDDMsg.CarInfoMsg, queue_size=5)

    


    def __init__(self):
        rospy.init_node('arduino_car')
        rospy.Subscriber('controls', BDDMsg.BDDControlsMsg, callback = ArduinoCar.update_data_callback)
        rospy.Subscriber('/zed/left/image_rect_color', Image, callback = ArduinoCar.left_image_callback)
        ArduinoCar.connect_to_arduino()
        ArduinoCar.run()
    
    
    # used for pairing each steer/throttle command to the image it belongs to
    # left and right images use the same seq num so no need to get right img seq num too   
    @classmethod
    def left_image_callback(cls, data):
        cls.img_num = data.header.seq
        print(cls.img_num)
        

    @classmethod
    def update_data_callback(cls, data):
        cls.NN_data = data



    @classmethod
    def run(cls):
 
        prev_human_steer_time = 0
        prev_human_throttle_time = 0

        while not rospy.is_shutdown():
        
            serial_data = cls.get_serial_data()
            if not serial_data:
                print('waiting for serial data...is the remote turned on?')
                continue
                
            if not cls.NN_data:
                print('waiting for NN data...')
                time.sleep(1)
                continue
            print(serial_data)
            RC_button_pwm, RC_steer_uncalib_pwm, RC_throttle_uncalib_pwm = serial_data
            current_state = cls.convert_button_to_state(RC_button_pwm)
            RC_steer_pwm = RC_steer_uncalib_pwm + cls.steer_offset # true, calibrated values
            RC_throttle_pwm = RC_throttle_uncalib_pwm + cls.throttle_offset # true, calibrated values
            RC_steer_percent = cls.convert_pwm_to_percent_steer(RC_steer_pwm)
            RC_throttle_percent = cls.convert_pwm_to_percent_throttle(RC_throttle_pwm)
            
            # Neural Network Execution
            if current_state == "state_one":
            
                NN_steer_pwm = cls.convert_steer_to_pwm(cls.NN_data.steer)
                NN_throttle_pwm = cls.convert_throttle_to_pwm(cls.NN_data.throttle)
                    
                # NN Steering + Human Throttle
                if abs(RC_throttle_pwm - params.throttle_null_pwm) >= params.command_pwm_epsilon or time.time() - prev_human_throttle_time <= params.stay_in_state_time:
                    print('state_9 -- NNE/Human Throttle', NN_steer_pwm, RC_throttle_pwm)
                    if abs(RC_throttle_pwm - params.throttle_null_pwm) >= params.command_pwm_epsilon:
                        prev_human_throttle_time = time.time() # throttle not neutral -- definitley still in this state
                    cls.send_to_arduino(NN_steer_pwm, RC_throttle_pwm)    
                    cls.publish_data(9, cls.NN_data.steer, RC_throttle_percent)

                
                # NN Throttle + Human Steering
                elif abs(RC_steer_pwm - params.steer_null_pwm) >= params.command_pwm_epsilon or time.time() - prev_human_steer_time <= params.stay_in_state_time:
                    print('state_6 -- NNE/Human Steering', RC_steer_pwm, NN_throttle_pwm)
                    if abs(RC_steer_pwm - params.steer_null_pwm) >= params.command_pwm_epsilon:
                        prev_human_steer_time = time.time() # steer not neutral -- definitley still in this state
                    cls.send_to_arduino(RC_steer_pwm, NN_throttle_pwm)    
                    cls.publish_data(6, RC_steer_percent, cls.NN_data.throttle)
                
                
                # NN Steering + Throttle
                else:
                    print('state_one -- Neural Network Execution', NN_steer_pwm, NN_throttle_pwm)
                    cls.send_to_arduino(NN_steer_pwm, NN_throttle_pwm)
                    cls.publish_data(1, cls.NN_data.steer, cls.NN_data.throttle)

            # Human annotation mode
            elif current_state == "state_two":
                print('state_two -- human annotation')
                cls.send_to_arduino(RC_steer_pwm, RC_throttle_pwm)
                cls.publish_data(2, RC_steer_percent, RC_throttle_percent)

            # Human correction mode (human control)
            elif current_state == "state_three":
                print('state_three -- human control')
                cls.send_to_arduino(RC_steer_pwm, RC_throttle_pwm)
                cls.publish_data(3, RC_steer_percent, RC_throttle_percent)

            # calibration
            elif current_state == "state_four":
                print('state_four -- calibration')
                cls.send_to_arduino(steer = params.steer_null_pwm, throttle = params.throttle_null_pwm)
                cls.calibrate(RC_steer_uncalib_pwm, RC_throttle_uncalib_pwm)
                cls.publish_data(4, 0, 0)




    # sends pwm signals for steer and throttle. Arduino differentiates
    # steer/throttle signals by +10000 added to throttle pwm
    @classmethod
    def send_to_arduino(cls, steer, throttle):
        cls.update_running_avg(steer, throttle)
        steer = cls.deque_avg(cls.steer_running_avg)
        throttle = cls.deque_avg(cls.throttle_running_avg)
        throttle = cls.limit_speed(throttle)
        
        if cls.arduino:
            try:
                cls.arduino.reset_input_buffer() # seems to makes write() work for some reason
                cls.arduino.write(str(int(steer)) + "\n")
                cls.arduino.write(str(int(throttle) + 10000) + "\n")
            except serial.SerialTimeoutException:
                print('serial write() timeout exception')

        
        
    
    @classmethod
    def update_running_avg(cls, steer_pwm, throttle_pwm):
        cls.steer_running_avg.append(steer_pwm)
        cls.throttle_running_avg.append(throttle_pwm)
        
        
        
        
    @classmethod
    def connect_to_arduino(cls):
        print("finding arduino")
        cls.arduino = serial.Serial(params.arduino_path, params.baudrate, timeout = params.timeout, write_timeout = 0)
        time.sleep(5) # give arduino time to boot
        print("found arduino")





    @classmethod
    def get_serial_data(cls):
        line = cls.arduino.readline()
        # expected line format: "mse, <button_pwm>, <steer_pwm>, <throttle_pwm>"
        try:
            line = line.split(', ')
            if 'mse' == line[0]:
                data = line[1:4]
                data = [int(x) for x in data] #convert each element to a number
                return data
            else:
                return None
        except ValueError:
            print('error in get_serial_data')
            return None



    
    # running average of controller signals when untouched to get null steer/throttle commands
    @classmethod
    def calibrate(cls, steer_pwm, throttle_pwm):  
        cls.steer_calibration.append(steer_pwm)
        cls.throttle_calibration.append(throttle_pwm)
        steer_avg = cls.deque_avg(cls.steer_calibration)
        throttle_avg = cls.deque_avg(cls.throttle_calibration)
        cls.steer_offset = params.steer_null_pwm - steer_avg
        cls.throttle_offset = params.throttle_null_pwm - throttle_avg
        print(cls.steer_offset, cls.throttle_offset)
        
        
        
        
    @classmethod
    def deque_avg(cls, deque):
        if len(deque) == 0:
            return 0
        return sum(deque) / len(deque)
            
    
    @classmethod
    def limit_speed(cls, throttle_pwm):
        if throttle_pwm > params.forward_speed_limit:
            return params.forward_speed_limit
        elif throttle_pwm < params.backward_speed_limit:
            return params.backward_speed_limit
        return throttle_pwm
        
        


    @classmethod
    def publish_data(cls, state_num, steer, throttle):
        #cls.state_pub.publish(state_num)
        #cls.steer_used_pub.publish(steer)
        #cls.throttle_used_pub.publish(throttle)
        cls.car_info_pub.publish(BDDMsg.CarInfoMsg(state=state_num, steer=steer, throttle=throttle, imgNum=cls.img_num))




    @classmethod
    def convert_button_to_state(cls, button_pwm):
        if abs(button_pwm - params.state_one_pwm) <= params.state_pwm_epsilon:
            return "state_one"
        elif abs(button_pwm - params.state_two_pwm) <= params.state_pwm_epsilon:
            return "state_two"
        elif abs(button_pwm - params.state_three_pwm) <= params.state_pwm_epsilon:
            return "state_three"
        elif abs(button_pwm - params.state_four_pwm) <= params.state_pwm_epsilon:
            return "state_four"
            



    # covert NN output (0 - 99) into steer PWM (Pulse Width Modulation) signals
    @classmethod
    def convert_steer_to_pwm(cls, value):
        return cls.from_range_to_range(value,
                                       from_min = params.nn_min_output,
                                       from_max = params.nn_max_output,
                                       to_min = params.steer_min_pwm,
                                       to_max =params.steer_max_pwm)




    # convert steer PWM into NN output percentage (0 - 99)
    @classmethod
    def convert_pwm_to_percent_steer(cls, value):
        return cls.from_range_to_range(value,
                                       from_min = params.steer_min_pwm,
                                       from_max = params.steer_max_pwm,
                                       to_min = params.nn_min_output,
                                       to_max =params.nn_max_output)




    # covert NN output (0 - 99) into throttle PWM (Pulse Width Modulation) signals
    @classmethod
    def convert_throttle_to_pwm(cls, value):
        return cls.from_range_to_range(value,
                                       from_min = params.nn_min_output,
                                       from_max = params.nn_max_output,
                                       to_min = params.throttle_min_pwm,
                                       to_max =params.throttle_max_pwm)




    # convert throttle PWM into NN output percentage (0 - 99)
    @classmethod
    def convert_pwm_to_percent_throttle(cls, value):
        return cls.from_range_to_range(value,
                                       from_min = params.throttle_min_pwm,
                                       from_max = params.throttle_max_pwm,
                                       to_min = params.nn_min_output,
                                       to_max =params.nn_max_output)




    # utility function for converting a value from one range of values to another
    @classmethod
    def from_range_to_range(cls, value, from_min, from_max, to_min, to_max):
        from_span = from_max - from_min
        to_span = to_max - to_min

        # convert value into a 0-1 range (float)
        value_scaled = float(value - from_min) / from_span

        # convert 0-1 range into a value in the 'to range'
        return to_min + (value_scaled * to_span)
            
            


if __name__ == '__main__':
    try:
        ArduinoCar()
    except rospy.ROSInterruptException:
        print('ropsy interrupt exception')

    except KeyboardInterrupt:
        print('shutdown requested...')

    except Exception as e:
        print('Exception', e)
        print('Exception type', type(e))

    finally:
        print('exiting car node...')
        for i in range(10):
            ArduinoCar.send_to_arduino(steer = params.steer_null_pwm, throttle = params.throttle_null_pwm)
        ArduinoCar.arduino.close()
        sys.exit(0)
        

