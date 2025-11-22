#!/usr/bin/env python3
import ast
import math
import time


class LaserModel(object):

    def __init__(self, angle_min, angle_max, range_min, range_max):

        # here the basic settings of your laser are defined
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.range_min = range_min
        self.range_max = range_max
        self.angle_inc = 0

        # some more member variables that we use during execution
        self.scan_data = []
        self.laser_read_cycle = -1

    '''
    Method to simulate a laserdata stream, you don't have to change anything here
    '''

    def update_laserdata(self, laserdata_file):
        file = open(laserdata_file, "r")
        laserdata_raw = file.read()
        scan_data_raw = ast.literal_eval(laserdata_raw)

        self.scan_data = scan_data_raw[self.laser_read_cycle % len(scan_data_raw)]
        self.laser_read_cycle += 1

    '''
    Setter and getter methods for some member variables
    '''

    def set_angle_inc(self, angle_inc):
        self.angle_inc = angle_inc

    def get_angle_inc(self):
        return self.angle_inc

    def get_scan_length(self):
        return len(self.scan_data)

    '''
    TODO: calculate the angle increment
    '''

    def calc_angle_inc(self):
        length = self.get_scan_length()
        if length <= 1:
            return 0
        return (self.angle_max - self.angle_min) / (length - 1)

    '''
    TODO: port your code from previous exercise
    '''

    def calc_index_of_closest_point(self):
        min_distance = None
        min_index = None

        for i, d in enumerate(self.scan_data):
            if d == 0.0:
                continue
            if d < self.range_min or d > self.range_max:
                continue
            if min_distance is None or d < min_distance:
                min_distance = d
                min_index = i

        return min_index

    '''
    TODO: port your code from previous exercise
    '''

    def calc_angle_of_closest_point(self):
        index = self.calc_index_of_closest_point()
        if index is None:
            return None
        return self.angle_min + index * self.angle_inc

    '''
    core method for your function calls etc
    '''

    def run(self):
        self.update_laserdata("laser-testdata_2")
        # print current results
        print("-"*20 +
              str("\nCurrent cycle: {0}" +
                  "\nIndex of closest point: {1}" +
                  "\nAngle of closest point: {2}"
                  ).format(self.laser_read_cycle, self.calc_index_of_closest_point(), self.calc_angle_of_closest_point())
              )


if __name__ == '__main__':

    # instantiate the class and set some parameters
    app = LaserModel(-math.pi/2, math.pi/2, 0.0, 7.0)
    app.update_laserdata("laser-testdata_2")
    app.set_angle_inc(app.calc_angle_inc())

    print("-"*20 +
          str("\nAngle increment: {}" +
              "\nLenght of scan data: {}"
              ).format(app.get_angle_inc(), app.get_scan_length())
          )

    # run the script in an infite loop to continously read and process laserdata
    while(True):
        app.run()
        time.sleep(1)
