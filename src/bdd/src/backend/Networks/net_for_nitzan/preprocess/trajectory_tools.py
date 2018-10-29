'''
Created on May 8, 2017

@author: Sascha Hornauer
'''

import numpy as np
from operator import mul, div, sub,mod, add
import cv2
import angles
import sys
import math
from collections import OrderedDict
from scipy.spatial import distance as dist
#from sklearn import datasets, linear_model
#from sklearn.metrics import mean_squared_error, r2_score


try:
    from angles import normalize as normalize_angle # Adjust for different angles packages
except ImportError:
    from angles import normalize_angle #  Adjust for different angles packages

def get_center(list_xys):
    # TODO add z axis
    
    xs_pos = []
    ys_pos = []
    
    for xy_pos in list_xys:
        xs_pos.append(xy_pos[0])
        ys_pos.append(xy_pos[1])
    
    center = (sum(xs_pos)/len(xs_pos),sum(ys_pos)/len(ys_pos))
    
    return center

def get_heading_old(seq_xy):
    '''
    Give an average heading over all positions. Two different positions are 
    the minimum, more will return a smoothed angle
    '''

    diffsX = []
    diffsY = []

    # calculate the angle:
    for i in range(0,len(seq_xy)-1):
        diffsX.append(seq_xy[i+1][0]-seq_xy[i][0])
        diffsY.append(seq_xy[i+1][1]-seq_xy[i][1])
    
    my_angle = np.arctan2(diffsY, diffsX)
    
    mean_angle = np.arctan2(np.mean(np.sin(my_angle)),np.mean(np.cos(my_angle)));
    
    return mean_angle

def get_heading(seq_xy):
    '''
    Give an average heading over all positions. Two different positions are 
    the minimum, more will return a smoothed angle
    '''
    diffsX = []
    diffsY = []

    #measure_accuracy = 0.01
    #no_of_measurements = 0
    # calculate the angle:
    
    seq_xy_no_dublicates = list(OrderedDict.fromkeys(seq_xy)) 
    
    for i in range(0,len(seq_xy_no_dublicates)-1):
        #if seq_xy[i+1] == seq_xy[i]:
        #    # Skipping equal values
        #    continue
        #
        #
        #if np.abs(seq_xy[i+1][0] - seq_xy[i][0]) < measure_accuracy:
        #    diffsX.append(0)
        #    diffsY.append(seq_xy[i+1][1]-seq_xy[i][1])
        #elif np.abs(seq_xy[i+1][1] - seq_xy[i][1]) < measure_accuracy:
        #    diffsX.append(seq_xy[i+1][0]-seq_xy[i][0])
        #    diffsY.append(0)
        #else:        
        diffsX.append(seq_xy_no_dublicates[i+1][0]-seq_xy_no_dublicates[i][0])
        diffsY.append(seq_xy_no_dublicates[i+1][1]-seq_xy_no_dublicates[i][1])
        #no_of_measurements += 1
    
    my_angle = np.arctan2(diffsY, diffsX)
    
    mean_angle = np.arctan2(np.mean(np.sin(my_angle)),np.mean(np.cos(my_angle)));
    
    return mean_angle

def get_heading_lin(seq_xy):
    '''
    Give an average heading over all positions. Two different positions are 
    the minimum, more will return a smoothed angle
    '''
    
    x1 = seq_xy[0][0]
    y1 = seq_xy[0][1]
    x2 = seq_xy[-1][0]
    y2 = seq_xy[-1][1]
    
    if (y2-y1) == 0:
        if x2 > x1:
            return 0 # return angle 0
        else:
            return np.pi # return pi
    elif (x2-x1) == 0:
        if y2 > y1:
            return np.pi/2.
        else:
            return -np.pi/2.
        
    m = (y2-y1)/(x2-x1)

    b = y1
    x_values = np.linspace(x1,x2,len(seq_xy))
    y_values = m*x_values+b
    
    my_angles = np.arctan2(y_values,x_values)
    
    mean_angle = np.arctan2(np.mean(np.sin(my_angles)),np.mean(np.cos(my_angles)))
    
    return mean_angle


def project_pos(xy,current_heading, distance=2):
    '''
    Take the position as xy and the current heading and returns future
    xy in a distance of distance
    '''
    
    x,y = cv2.polarToCart(distance,current_heading)
    
    return [x[0][0]+xy[0],y[0][0]+xy[1]]


def get_velocities(xy_positions,framerate):
    '''
    Returns the diffs in between two x,y positions divided by the
    framerate aka velocity in m/s since the distance in time between two
    positions is exactly that framerate
    '''
    
    velocities = []
    # First add 0 as a start
    velocities.append([0.0,0.0])
    
    for i in range(1,len(xy_positions)):
        x = xy_positions[i-1][0]
        y = xy_positions[i-1][1]
        x_t1 = xy_positions[i][0]
        y_t1 = xy_positions[i][1]
    
        sx = x_t1 - x
        sy = y_t1 - y
        
        vx = sx / framerate
        vy = sy / framerate
        velocities.append([vx,vy])
        
    
    return velocities

def distance_2d(point_a,point_b):
    return np.hypot(point_a[0]-point_b[0],point_a[1]-point_b[1])

def get_pos_diff(xy_positions):
    
    
    diffs = []
    # First add 0 as a start
    #diffs.append([0.0,0.0])
    
    for i in range(1,len(xy_positions)):
        x = xy_positions[i-1][0]
        y = xy_positions[i-1][1]
        x_t1 = xy_positions[i][0]
        y_t1 = xy_positions[i][1]
    
        sx = x_t1 - x
        sy = y_t1 - y
        
        vx = sx 
        vy = sy
        diffs.append([vx,vy])
        
    
    return diffs


def convert_delta_to_steer(delta_values):
    
    # delta values are assumed to be in +pi/2,-pi/2
    
    max_left_command = 100
    max_right_command = 0
    
    range = max_left_command-max_right_command
    
    steering_values=[]
    
    for values in delta_values:
        
        norm_values = map(div,values,[np.pi]*np.ones(len(values)))
        norm_values = map(add,norm_values,[0.5]*np.ones(len(values)))
        norm_values = map(mul,norm_values,[range]*np.ones(len(values)))
        norm_values = map(add,norm_values,[max_right_command]*np.ones(len(values)))
        
        
#         for value in values:
#             value = normalize_angle(value)
#             
#             range = max_left_command-max_right_command
#             value = normalize_angle(np.pi - value)/np.pi
#             value = value*range + max_right_command
#             norm_values.append(value)
#         
        steering_values.append(norm_values) 
        
    return steering_values

### Approach by Martin Thoma https://martin-thoma.com/author/martin-thoma/
class Point:
    """Represents a two dimensional point."""

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __get__(self, obj, cls=None):
        return obj

    def __repr__(self):
        return "P(%.2lf|%.2lf)" % (self.x, self.y)

    def __str__(self):
        return repr(self)
    
def to_point(xy):
    # Convenience method to convert an arbitrary tuple/array to a point
    return Point(xy[0],xy[1])

def distance_of_points(point_a, point_b):
    return distance_2d((point_a.x,point_a.y), (point_b.x,point_b.y))

def points_to_list(points):
    return_points = []
    
    if isinstance(points, list):
        for point in points:
            return_points.append((point.x,point.y))
    else:
        return_points = (points.x,points.y)
    return return_points

class Triangle:
    """Represents a triangle in R^2."""

    epsilon = 0.001

    def __init__(self, a, b, c):
        assert isinstance(a, Point)
        assert isinstance(b, Point)
        assert isinstance(c, Point)
        self.a = a
        self.b = b
        self.c = c

    def getArea(self):
        """Get area of this triangle.
           >>> Triangle(Point(0.,0.), Point(10.,0.), Point(10.,10.)).getArea()
           50.0
           >>> Triangle(Point(-10.,0.), Point(10.,0.), Point(10.,10.)).getArea()
           100.0
        """
        a, b, c = self.a, self.b, self.c
        return abs(a.x*(b.y-c.y)+b.x*(c.y-a.y)+c.x*(a.y-b.y))/2

    def isInside(self, p):
        """Check if p is inside this triangle."""
        assert isinstance(p, Point)
        currentArea = self.getArea()
        pab = Triangle(p,self.a, self.b)
        pac = Triangle(p,self.a, self.c)
        pbc = Triangle(p,self.b, self.c)
        newArea = pab.getArea()+pac.getArea()+pbc.getArea()
        return (abs(currentArea - newArea) < Triangle.epsilon)

if __name__ == '__main__':
    print "test"
    
    test_data = [(2.5, 3.3900000000000001, -0.4370375297663911), (2.5, 3.3900000000000001, -0.43703752976639115), (2.5, 3.3900000000000001, -0.43703752976639115), (2.5099999999999998, 3.3100000000000001, 0.0), (2.5099999999999998, 3.3100000000000001, -0.44622533687811844), (2.5099999999999998, 3.3100000000000001, -0.44622533687811844), (2.5099999999999998, 3.3100000000000001, -0.44622533687811844), (2.52, 3.1899999999999999, 0.0), (2.52, 3.1899999999999999, 0.0), (2.52, 3.1899999999999999, -0.46364760900080609), (2.52, 3.1899999999999999, -0.46364760900080609), (2.52, 3.1899999999999999, -0.46364760900080609), (2.52, 3.04, 0.0), (2.52, 3.04, 0.0), (2.52, 3.04, -0.48653267660238503), (2.52, 3.04, -0.48653267660238508), (2.52, 3.04, -0.48653267660238508), (2.5, 2.8799999999999999, 0.0), (2.5, 2.8799999999999999, -0.5043870891924439), (2.5, 2.8799999999999999, -0.5043870891924439), (2.5, 2.8799999999999999, -0.5043870891924439), (2.46, 2.7200000000000002, 0.0), (2.46, 2.7200000000000002, 0.0), (2.46, 2.7200000000000002, -0.51309362353662524), (2.46, 2.7200000000000002, -0.51309362353662524), (2.46, 2.7200000000000002, -0.51309362353662524), (2.4100000000000001, 2.5699999999999998, 0.0), (2.4100000000000001, 2.5699999999999998, 0.0), (2.4100000000000001, 2.5699999999999998, -0.52259622737025502), (2.4100000000000001, 2.5699999999999998, -0.52259622737025502), (2.4100000000000001, 2.5699999999999998, -0.52259622737025502), (2.3300000000000001, 2.4100000000000001, 0.0), (2.3300000000000001, 2.4100000000000001, 0.0), (2.3300000000000001, 2.4100000000000001, -0.52152080785174848), (2.3300000000000001, 2.4100000000000001, -0.52152080785174848), (2.3300000000000001, 2.4100000000000001, -0.52152080785174848), (2.2599999999999998, 2.2599999999999998, 0.0), (2.2599999999999998, 2.2599999999999998, -0.51979577634579066), (2.2599999999999998, 2.2599999999999998, -0.51979577634579066), (2.2599999999999998, 2.2599999999999998, -0.51979577634579066), (2.2000000000000002, 2.1200000000000001, 0.0), (2.2000000000000002, 2.1200000000000001, 0.0), (2.2000000000000002, 2.1200000000000001, -0.51509354448165978), (2.2000000000000002, 2.1200000000000001, -0.51509354448165978), (2.2000000000000002, 2.1200000000000001, -0.51509354448165978), (2.1499999999999999, 1.98, 0.0), (2.1499999999999999, 1.98, 0.0), (2.1499999999999999, 1.98, -0.48941577102625644), (2.1499999999999999, 1.98, -0.48941577102625644), (2.1499999999999999, 1.98, -0.48941577102625644), (2.1299999999999999, 1.8400000000000001, 0.0), (2.1299999999999999, 1.8400000000000001, 0.0), (2.1299999999999999, 1.8400000000000001, -0.47641003085543959), (2.1299999999999999, 1.8400000000000001, -0.47641003085543959), (2.1299999999999999, 1.8400000000000001, -0.47641003085543959), (2.1200000000000001, 1.6899999999999999, 0.0), (2.1200000000000001, 1.6899999999999999, -0.47727496972591188), (2.1200000000000001, 1.6899999999999999, -0.47727496972591188), (2.1200000000000001, 1.6899999999999999, -0.47727496972591188), (2.1099999999999999, 1.55, 0.0), (2.1099999999999999, 1.55, 0.0), (2.1099999999999999, 1.55, -0.44982055420290568), (2.1099999999999999, 1.55, -0.44982055420290579), (2.1099999999999999, 1.55, -0.44982055420290579), (2.1200000000000001, 1.3999999999999999, 0.0), (2.1200000000000001, 1.3999999999999999, 0.0), (2.1200000000000001, 1.3999999999999999, -0.44982055420290634), (2.1200000000000001, 1.3999999999999999, -0.44982055420290634), (2.1200000000000001, 1.3999999999999999, -0.44982055420290634), (2.1299999999999999, 1.25, 0.0), (2.1299999999999999, 1.25, -0.43703752976639049), (2.1299999999999999, 1.25, -0.43703752976639049), (2.1299999999999999, 1.25, -0.43703752976639049), (2.1499999999999999, 1.0900000000000001, 0.0), (2.1499999999999999, 1.0900000000000001, 0.0), (2.1499999999999999, 1.0900000000000001, -0.42214576129839426), (2.1499999999999999, 1.0900000000000001, -0.42214576129839426), (2.1499999999999999, 1.0900000000000001, -0.42214576129839426), (2.1899999999999999, 0.88, 0.0), (2.1899999999999999, 0.88, 0.0), (2.1899999999999999, 0.88, 0.0), (2.1899999999999999, 0.88, 0.0), (2.1899999999999999, 0.88, 0.0), (2.1899999999999999, 0.88, -0.4127933544084299), (2.1899999999999999, 0.88, -0.4127933544084299), (2.1899999999999999, 0.88, -0.4127933544084299), (2.2200000000000002, 0.75, 0.0), (2.2200000000000002, 0.75, 0.0), (2.2200000000000002, 0.75, -0.41279335440843073), (2.2200000000000002, 0.75, -0.41279335440843073), (2.2200000000000002, 0.75, -0.41279335440843073), (2.25, 0.62, 0.0), (2.25, 0.62, 0.0), (2.25, 0.62, -0.4241399447705243), (2.25, 0.62, -0.4241399447705243), (2.25, 0.62, -0.4241399447705243), (2.27, 0.51000000000000001, 0.0), (2.27, 0.51000000000000001, -0.40829837367507821), (2.27, 0.51000000000000001, -0.40829837367507821), (2.27, 0.51000000000000001, -0.40829837367507821), (2.2799999999999998, 0.46999999999999997, 0.0), (2.2799999999999998, 0.46999999999999997, 0.0), (2.2799999999999998, 0.46999999999999997, 0.0), (2.2799999999999998, 0.46999999999999997, 0.50047403677538804), (2.2799999999999998, 0.46999999999999997, 0.50047403677538804), (2.2799999999999998, 0.46999999999999997, 0.50047403677538804), (2.27, 0.47999999999999998, 0.0), (2.27, 0.47999999999999998, -0.46364760900080609), (2.27, 0.47999999999999998, -0.78539816339744828), (2.27, 0.47999999999999998, -1.5707963267948966)]
    
    test_xy = [(x[0],x[1]) for x in test_data]
    
    for i in range(len(test_xy)):
        local_seq = test_xy[i:i+4]
        #print local_seq
        print "New {0} and old {1}".format(get_heading(local_seq) , get_heading_old(local_seq))

    
    