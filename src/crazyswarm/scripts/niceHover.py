#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 2.0

if __name__ == "__main__":

    print("niceHover.py launched")

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    print("swarm initialized")

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)

    timeHelper.sleep(1.5+Z)

    for cf in allcfs.crazyflies:

        start_wp = np.array(cf.initialPosition) + np.array([0, 0, Z])

        wp1 = np.array(cf.initialPosition) + np.array([1, 1, Z+1.0])

        cf.goTo(wp1, 0, 3.0)

        timeHelper.sleep(5.0)

        cf.goTo(start_wp, 0, 3.0)

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
