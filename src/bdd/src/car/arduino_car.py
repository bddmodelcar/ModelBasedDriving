#!/usr/bin/env python

import serial
from params import params
import time
import rospy
import bdd.msg as BDDMsg
from std_msgs.msg import Int32





arduino = None
NN_data = None




def setup():
    connect_to_arduino()
    rospy.init_node('arduino_car')
    rospy.Subscriber('controls', BDDMsg.BDDControlsMsg, callback = callback_func)
    state_pub = rospy.Publisher('state', Int32, queue_size=5) 
    steer_used_pub = rospy.Publisher('steer_used', Int32, queue_size=5) 
    throttle_used_pub = rospy.Publisher('throttle_used', Int32, queue_size=5) 
    run() #rospy.spin()

    print('exiting car node')
    arduino.close()
    arduino = None	

def callback_func(data):
    global NN_data
    NN_data = data


def run():

    while not rospy.is_shutdown():

	    # convert NN_data to pwm
	    global NN_data
	    if not NN_data:
	        #print('no NN data')
	        continue
	    NN_steer_pwm = convert_steer_to_pwm(NN_data.steer)
	    NN_throttle_pwm = convert_throttle_to_pwm(NN_data.throttle)
	    serial_data = get_serial_data()
	    if not serial_data:
	        print('no serial data')
	        continue
	    RC_button_pwm, RC_steer_pwm, RC_throttle_pwm = serial_data
	    current_state = convert_button_to_state(RC_button_pwm)

	    # based on state choose what to do with all the data
	    if current_state == "state_one":
	        print('state_two -- Neural Network Execution', NN_steer_pwm, NN_throttle_pwm)
        	# Neural Network Execution
        	send_to_arduino(NN_steer_pwm, NN_throttle_pwm)
        	#publish_data(4, NN_data.steer, NN_data.throttle)

	    elif current_state == "state_two":
	        # Human annotation mode
	        print('state_two -- human annotation')

	        # take in human commands for training
	        pass


	    elif current_state == "state_three":
	        # Human correction mode (human control)
	        print('state_three -- human control')
	        send_to_arduino(RC_steer_pwm, RC_throttle_pwm)
	        steer = convert_pwm_to_NN_steer(RC_steer_pwm)
	        throttle = convert_pwm_to_NN_throttle(RC_throttle_pwm)
	        #publish_data(3, steer, throttle)
		

	    elif current_state == "state_four":	
	        # calibration
	        controller_calibration_prompt(RC_steer_pwm, RC_throttle_pwm)
	        pass




def connect_to_arduino():
	global arduino
	print("finding arduino")
	arduino = serial.Serial(params.arduino_path, params.baudrate, timeout = params.timeout)
	time.sleep(5) # give arduino time to boot
	print("found arduino")
	pass
	send_to_arduino(steer = params.steer_min_pwm, throttle = params.throttle_null_pwm)
	time.sleep(1)
	send_to_arduino(steer = params.steer_max_pwm, throttle = params.throttle_null_pwm)
	time.sleep(1)
	send_to_arduino(steer = params.steer_null_pwm, throttle = params.throttle_null_pwm)




def get_serial_data():
	line = arduino.readline()

	# expected line format: "mse, <button_pwm>, <steer_pwm>, <throttle_pwm>"
	line = line.split(', ')
	#print(line)
	if "mse" == line[0]:
		data = line[1:]
		data = [int(x) for x in data] #convert each element to a number
		#print(data)
		return data
	else:
	    return None




# covert NN output (0 - 99) into steer PWM (Pulse Width Modulation) signals
def convert_steer_to_pwm(value):
	return from_range_to_range(value, 
		from_min = params.nn_min_output, 
		from_max = params.nn_max_output, 
		to_min = params.steer_min_pwm,
		to_max =params.steer_max_pwm)




# convert steer PWM into NN output percentage (0 - 99)
def convert_pwm_to_NN_steer(value):
	return from_range_to_range(value, 
		from_min = params.steer_min_pwm, 
		from_max = params.steer_max_pwm, 
		to_min = params.nn_min_output,
		to_max =params.nn_max_output)




# covert NN output (0 - 99) into throttle PWM (Pulse Width Modulation) signals
def convert_throttle_to_pwm(value):
	return from_range_to_range(value, 
		from_min = params.nn_min_output, 
		from_max = params.nn_max_output, 
		to_min = params.throttle_min_pwm,
		to_max =params.throttle_max_pwm)




# convert throttle PWM into NN output percentage (0 - 99)
def convert_pwm_to_NN_throttle(value):
	return from_range_to_range(value, 
		from_min = params.throttle_min_pwm, 
		from_max = params.throttle_max_pwm, 
		to_min = params.nn_min_output,
		to_max =params.nn_max_output)




# utility function for converting a value from one range of values to another
def from_range_to_range(value, from_min, from_max, to_min, to_max):
	from_span = from_max - from_min
	to_span = to_max - to_min

	# convert value into a 0-1 range (float)
	value_scaled = float(value - from_min) / from_span

	# convert 0-1 range into a value in the 'to range'
	return to_min + (value_scaled * to_span)




def convert_button_to_state(button_pwm):
	if abs(button_pwm - params.state_one_pwm) <= params.state_pwm_epsilon:
		return "state_one"
	elif abs(button_pwm - params.state_two_pwm) <= params.state_pwm_epsilon:
		return "state_two"
	elif abs(button_pwm - params.state_three_pwm) <= params.state_pwm_epsilon:
		return "state_three"
	elif abs(button_pwm - params.state_four_pwm) <= params.state_pwm_epsilon:
		return "state_four"




# sends pwm signals for steer and throttle. Arduino differentiates
# steer/throttle signals by +10000 added to throttle pwm
def send_to_arduino(steer, throttle): 
	arduino.write(str(steer) + "\n")
	arduino.write(str(throttle + 10000) + "\n")




def publish_data(state_num, steer, throttle):
	state_pub.publish(state_num)
	steer_used_pub.publish(steer)
	throttle_used_pub.publish(throttle)
	
	
	
	
def controller_calibration_prompt(RC_steer_pwm, RC_throttle_pwm):
    prompt = str()
    if abs(RC_steer_pwm - params.steer_null_pwm) < 7 and abs(RC_throttle_pwm - params.throttle_null_pwm) < 7:
        prompt = '4 - Calibration \tOK\tOK'
    elif abs(RC_steer_pwm - params.steer_null_pwm) > 7 and abs(RC_throttle_pwm - params.throttle_null_pwm) < 7:
        prompt = '4 - Calibration STEER: ' + str(RC_steer_pwm - params.steer_null_pwm) + '\tOK'
    elif abs(RC_steer_pwm - params.steer_null_pwm) < 7 and abs(RC_throttle_pwm - params.throttle_null_pwm) > 7:
        prompt = '4 - Calibration \tOK' + '\tTHROTTLE: ' + str(RC_throttle_pwm - params.throttle_null_pwm)
    else:
        prompt = '4 - Calibration STEER: ' + str(RC_steer_pwm - params.steer_null_pwm) + '\tTHROTTLE: ' + str(RC_throttle_pwm - params.throttle_null_pwm)
         
    
    print(prompt)



if __name__ == '__main__':
    try:
    	setup()
    except rospy.ROSInterruptException:
    	pass
