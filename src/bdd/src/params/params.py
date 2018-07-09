# arduino_utils.py
speed_pwm_null = 24
speed_pwm_gain = 2
speed_pwm_min = 0
speed_pwm_max = 4096
direction_pwm_null = 21
direction_pwm_gain = 3
direction_pwm_min = 0
direction_pwm_max = 4096
baudrate = 19200
timeout = 1

# driver.py
test_mode = True

# models
model_info = [
    {
        'name': 'blank',
        'full_path': '/home/mark/bddcar/path/to/model.py',
        'framework': 'pytorch', # or something else like 'tensorflow'
                                # if your framework isn't supported,
                                # then you will have to convert a
                                # numpy matrix to your networks's
                                # desired type
    },
]


