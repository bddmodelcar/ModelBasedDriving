import numpy as np
import cv2
from params import params
from enum import Enum

class MemType (Enum):
    """
    enum for representing what kind
    of memory is or should be used for
    storage
    """
    CPU = 0
    GPU = 1

class Camera():
    def __init__(self, camera_number=0, mem_type=MemType.CPU):
        """Usually, camera_number=0 is correct.
        However, if there are multiple cameras
        connected or if you run into issues, check
        the output of "ls /dev/ | grep video" and
        replace camera_number with the one of the
        numbers in the output from grep

        mem_type = GPU is currently not supported
        """
        self.cam = cv2.VideoCapture(0)
        status = self.cam.isOpened()
        if not status:
            rospy.logfatal("failed to open camera..is the Zed camera connected?")
            exit(1)
        else:
            self.initialized = True
            self.setCameraSettings()

    def __del__(self):
        if self.initialized:
            self.cam.release()
        cv2.destroyAllWindows()

    def setCameraSettings(self):
        """set the camera resolution, frame rate,
        and other parameters based on the values
        read by the params module

        see [here](https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture-get)
        for possible settings
        """
        #s = params.camera_settings
        s = {'fps': 30, 'frame_height': 480, 'frame_width': 640}
        self.fps = s['fps']
        self.height = s['frame_height']
        self.width = s['frame_width']

        self.cam.set(cv2.CAP_PROP_FPS, self.fps)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        # the *2 is because zed frames contain the left and
        # right images side by side
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width*2)

    def capture(self, display=False):
        """captures a frame and if successful,
        returns True, image. Otherwise, returns
        False, None, None

        display is a boolean flag indicating
        whether to display the captured image
        in a window, which is useful for debugging
        """
        success = False
        frame = None
        if self.initialized:
            success, frame = self.cam.read()
        if display and success:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
        return success, frame
