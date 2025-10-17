import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from math import pi,radians,degrees,sin
from time import time,sleep
import matplotlib.pyplot as plt
import numpy as np

from scripts.common import *
from scripts.Offboard import Offboard

class NetworkTest(PrintObject):
    def __init__(self):
        self.car = Offboard("192.168.10.13",2390)
        self.car.debug = True
        self.debug = True
        return

    def main(self):
        try:
            while True:
                sleep(10)
        except KeyboardInterrupt:
            mean_dt = np.mean(self.car.dt_vec[1:])
            self.car.quit()
            print(f'mean dt = {mean_dt}')
            self.print_info("waiting to quit")


if __name__ == '__main__':
    main = NetworkTest() 
    main.main()

