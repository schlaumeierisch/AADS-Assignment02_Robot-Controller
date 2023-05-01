# -*- coding: utf-8 -*-
"""
minimal template for BasicEPuck.ePuckVRep
for usage with ePuckS5V12.ttm

@author: hoch ralph
"""
import time

from BasicEPuck.ePuckVRep import EPuckVRep
from EPuckControllers.behavior import HomingBehavior, FindPuckBehavior, WanderingBehavior


def main():
    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    robot.enableCamera()  # enable camera to detect objective
    robot.enablePose()  # enable pose to determine current position
    robot.enableAllSensors()
    robot.setSensesAllTogether(False)  # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    # behaviors
    find_puck = FindPuckBehavior()
    homing = HomingBehavior()
    wandering = WanderingBehavior()

    behaviors = [find_puck, homing, wandering]

    # main sense-act cycle
    while robot.isConnected():
        robot.fastSensingOverSignal()

        for behavior in behaviors:
            if behavior.is_applicable(robot):  # sense & plan
                behavior.execute(robot)  # act
                break

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()
