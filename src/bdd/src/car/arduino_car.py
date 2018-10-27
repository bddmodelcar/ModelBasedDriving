#!/usr/bin/env python

import serial
from params import params
import time
import rospy
import bdd.msg as BDDMsg
from std_msgs.msg import Int32
import sys
from collections import deque



class ArduinoCar():

    arduino = None
    NN_data = None
    steer_calibration = deque(maxlen = 10)
    throttle_calibration = deque(maxlen = 10)
    steer_running_avg = deque(maxlen = params.running_avg_len)
    throttle_running_avg = deque(maxlen = params.running_avg_len)
    steer_offset = 0
    throttle_offset = 0

    state_pub = rospy.Publisher('state', Int32, queue_size=5)
    steer_used_pub = rospy.Publisher('steer_used', Int32, queue_size=5)
    throttle_used_pub = rospy.Publisher('throttle_used', Int32, queue_size=5)

    def __init__(self):
        rospy.init_node('arduino_car')
        rospy.Subscriber('controls', BDDMsg.BDDControlsMsg, callback = ArduinoCar.update_data_callback)
        ArduinoCar.connect_to_arduino()
        ArduinoCar.run()
        
        

    @classmethod
    def update_data_callback(cls, data):
        cls.NN_data = data



    @classmethod
    def run(cls):
 

        while not rospy.is_shutdown():
        
            serial_data = cls.get_serial_data()
            if not serial_data:
                print('waiting for serial data...')
                continue
                
            if not cls.NN_data:
                print('waiting for NN data...')
                time.sleep(1)
                continue
            
            RC_button_pwm, RC_steer_pwm, RC_throttle_pwm = serial_data
            current_state = cls.convert_button_to_state(RC_button_pwm)
            
            # Neural Network Execution
            if current_state == "state_one":
                NN_steer_pwm = cls.convert_steer_to_pwm(cls.NN_data.steer)
                NN_throttle_pwm = cls.convert_throttle_to_pwm(cls.NN_data.throttle)
                print('state_one -- Neural Network Execution', NN_steer_pwm, NN_throttle_pwm)
                cls.send_to_arduino(NN_steer_pwm, NN_throttle_pwm)
                cls.publish_data(1, cls.NN_data.steer, cls.NN_data.throttle)

            # Human annotation mode
            elif current_state == "state_two":
                print('state_two -- human annotation')
                RC_steer_pwm += cls.steer_offset # calibrated values
                RC_throttle_pwm += cls.throttle_offset
                print("human sending", RC_steer_pwm, RC_throttle_pwm)
                cls.send_to_arduino(RC_steer_pwm, RC_throttle_pwm)
                steer = cls.convert_pwm_to_NN_steer(RC_steer_pwm)
                throttle = cls.convert_pwm_to_NN_throttle(RC_throttle_pwm)
                cls.publish_data(2, steer, throttle)

            # Human correction mode (human control)
            elif current_state == "state_three":
                print('state_three -- human control')
                RC_steer_pwm += cls.steer_offset # calibrated values
                RC_throttle_pwm += cls.throttle_offset
                cls.send_to_arduino(RC_steer_pwm, RC_throttle_pwm)
                steer = cls.convert_pwm_to_NN_steer(RC_steer_pwm)
                throttle = cls.convert_pwm_to_NN_throttle(RC_throttle_pwm)
                cls.publish_data(3, steer, throttle)

            # calibration
            elif current_state == "state_four":
                print('state_four -- calibration')
                cls.send_to_arduino(steer = params.steer_null_pwm, throttle = params.throttle_null_pwm)
                cls.calibrate(RC_steer_pwm, RC_throttle_pwm)
                cls.publish_data(4, None, None)




    # sends pwm signals for steer and throttle. Arduino differentiates
    # steer/throttle signals by +10000 added to throttle pwm
    @classmethod
    def send_to_arduino(cls, steer, throttle):
        print('Commands:', steer, throttle)
        cls.update_running_avg(steer, throttle)
        steer = cls.deque_avg(cls.steer_running_avg)
        throttle = cls.deque_avg(cls.throttle_running_avg)
        print('Commands:', steer, throttle)
        throttle = cls.limit_speed(throttle)
        print('Commands:', steer, throttle)
        
        try:
            cls.arduino.reset_input_buffer() # seems to makes write() work for some reason
            cls.arduino.write(str(int(steer)) + "\n")
            cls.arduino.write(str(int(throttle) + 10000) + "\n")
        except serial.SerialTimeoutException:
            print('serial write() timeout exception')
            #cls.arduino.reset_input_buffer()

        
        
    
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
        line = line.split(', ')
        if 'mse' == line[0]:
            data = line[1:]
            try:
                data = [int(x) for x in data] #convert each element to a number
                return data
            except ValueError:
                return None
        else:
            return None


    ''' ### UNUSED ###  
    @classmethod
    def controller_calibration_prompt(cls, RC_steer_pwm, RC_throttle_pwm):
        prompt = str()
        if abs(RC_steer_pwm - params.steer_null_pwm) < params.calibration_epsilon and abs(RC_throttle_pwm - params.throttle_null_pwm) < params.calibration_epsilon:
            prompt = '4 - Calibration \tOK\tOK'
            
        elif abs(RC_steer_pwm - params.steer_null_pwm) > params.calibration_epsilon and abs(RC_throttle_pwm - params.throttle_null_pwm) < params.calibration_epsilon:
            prompt = '4 - Calibration STEER: ' + str(RC_steer_pwm - params.steer_null_pwm) + '\tOK'
            
        elif abs(RC_steer_pwm - params.steer_null_pwm) < params.calibration_epsilon and abs(RC_throttle_pwm - params.throttle_null_pwm) > params.calibration_epsilon:
            prompt = '4 - Calibration \tOK' + '\tTHROTTLE: ' + str(RC_throttle_pwm - params.throttle_null_pwm)
            
        else:
            prompt = '4 - Calibration STEER: ' + str(RC_steer_pwm - params.steer_null_pwm) + '\tTHROTTLE: ' + str(RC_throttle_pwm - params.throttle_null_pwm)


        print(prompt)
    
    '''
    
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
        #print(deque)
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
        cls.state_pub.publish(state_num)
        cls.steer_used_pub.publish(steer)
        cls.throttle_used_pub.publish(throttle)
        



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
    def convert_pwm_to_NN_steer(cls, value):
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
    def convert_pwm_to_NN_throttle(cls, value):
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
        print('exiting car node')

    except KeyboardInterrupt:
        print('shutdown requested...')
        
        
    # catch write time out exception and make program continue
        
    except Exception as e:
        print('Exception', e)
        print('Exception type', type(e))
        print('exiting car node')

        
    finally:
        print('finally exiting...')
        for i in range(10):
            ArduinoCar.send_to_arduino(steer = params.steer_null_pwm, throttle = params.throttle_null_pwm)
            ArduinoCar.arduino.flush()
        ArduinoCar.arduino.close()
        sys.exit(0)
        

