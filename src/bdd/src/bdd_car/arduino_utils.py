import os
import serial
from autocomm_params import params

controller = False # becomes the serial object

def find_controller():
    acm_ports = [os.path.join('/dev', dev) for dev in os.listdir('/dev') if 'ttyACM' in p]
    for port in acm_ports:
        try:
            ser = serial.Serial(port, baudrate=params.baudrate, timeout=params.timeout)
            for _ in xrange(100):
                line = ser.readline()
                if 'mse' in line:
                    return ser
            ser.close()
        except:
            pass
    print('could not find motor controller')
    return False


# speed is a floating point value between -1 and 1
def pwm_from_speed(speed):
    return int(min(params.speed_pwm_max, max(params.speed_pwm_min, (params.speed_pwm_null + params.speed_pwm_gain * speed))))

# direction is a floating point value between -1 and 1
def pwm_from_direction(direction):
    return int(min(params.direction_pwm_max, max(params.direction_pwm_min, (params.direction_pwm_null + params.direction_pwm_gain * direction))))

def control_str(speed, direction):
    speed_pwm = pwm_from_speed(speed)
    direction_pwm = pwm_from_direction(direction)
    return '(' + str(speed_pwm) + ',' + str(direction_pwm) + ')'

def init():
    controller = find_controller()
    if controller:
        print('found motor controller')
        write(speed=0, direction=0)

def shutdown():
    if controller:
        write(speed=0, direction=0)
        controller.close()
        controller = False

def write(speed, direction):
    if controller:
        controller.write(control_str(speed=speed, direction=direction))
