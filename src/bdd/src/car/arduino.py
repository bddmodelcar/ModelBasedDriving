#!/usr/bin/env python

import serial
from params import params
import time
import rospy





arduino = None



def setup():
	connect_to_arduino()
	rospy.init_node('carControl')
    rospy.Subscriber('controls', BDDMsg.BDDControlsMsg, callback = run)
    state_pub = rospy.Publisher('state', Int32, queue_size=5) 
	steer_used_pub = rospy.Publisher('steer_used', Int32, queue_size=5) 
	throttle_used_pub = rospy.Publisher('throttle_used', Int32, queue_size=5) 
    rospy.spin()

    print('exiting car node')
    arduino.close()
    arduino = None	




def run(NN_data):

	# convert NN_data to pwm
	NN_steer_pwm = convert_steer_to_pwm(NN_data.steer)
	NN_throttle_pwm = convert_throttle_to_pwm(NN_data.throttle)
	RC_button_pwm, RC_steer_pwm, RC_throttle_pwm = get_serial_data()
	current_state = convert_button_to_state(RC_button_pwm)

	# based on state choose what to do with all the data
	if current_state == "state_one":
		# calibration
		pass

	elif current_state == "state_two":
		# Human annotation mode

		# take in human commands for training
		pass


	elif current_state == "state_three":
		# Human correction mode (human control)
		send_to_arduino(RC_steer_pwm, RC_throttle_pwm)
		steer = convert_pwm_to_NN_steer(RC_steer_pwm)
		throttle = convert_pwm_to_NN_throttle(RC_throttle_pwm)
		publish_data(3, steer, throttle)
		

	elif current_state == "state_four":
		# Neural Network Execution
		send_to_arduino(NN_steer_pwm, NN_throttle_pwm)
		publish_data(4, NN_data.steer, NN_data.throttle)




def connect_to_arduino():
	print("finding arduino")
	arduino = serial.Serial(params.arduino_path, params.baudrate, timeout = params.timeout)
	time.sleep(5) # give arduino time to boot
	send_to_arduino(steer = steer_null_pwm, throttle = throttle_null_pwm)




def get_serial_data():
	line = arduino.readline()

	# expected line format: "mse, <button_pwm>, <steer_pwm>, <throttle_pwm>"
	line = line.split(', ')
	if "mse" == line[0]:
		return line[1:]




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
	arduino.write(steer + "\n")
	arduino.write(throttle + 10000 + "\n")




def publish_data(state_num, steer, throttle):
	state_pub.publish(state_num)
	steer_used_pub.publish(steer)
	throttle_used_pub.publish(throttle)




if __name__ == '__main__':
    try:
    	setup()
    except rospy.ROSInterruptException:
    	pass
