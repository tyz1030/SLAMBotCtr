#! /usr/bin/python
import lcm
import time
from time import sleep
import sys
import math
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t



class WaypointFollower():
    def __init__(self):
        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        self.lcm_sub = self.lc.subscribe("ODOMETRY", self.odometry_handler)
        self.waypoints = [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0],[0.0,0.0]]
        self.xyt = ()
        self.wpt_num = 0
        self.wpt_thresh = 0.01
   
    def odometry_handler(self, channel, data):
        msg = odometry_t().decode(data)
        self.xyt = (msg.x, msg.y, msg.theta)
        self.motor_cmd_publish()


    def motor_cmd_publish(self):
        if self.wpt_num < len(self.waypoints):
            w = self.waypoints[self.wpt_num]
            dx = w[0] - self.xyt[0]
            dy = w[1] - self.xyt[1]
            targetTheta = math.atan2(dy, dx)
            dtheta = targetTheta - self.xyt[2]
            dist = math.sqrt(dx*dx + dy*dy)

            # decide if we've reached there
            if dist < self.wpt_thresh:
                self.wpt_num += 1
                return
            else:
                print dist

            msg = mbot_motor_command_t()
            msg.utime = time.time()
            msg.trans_v = dist * 0.1
            # # scale angular velocity with distance so we don't turn as much close to target
            # msg.angular_v = dist * dtheta * 0.5
            self.lc.publish("MBOT_MOTOR_COMMAND",msg.encode())

    def finished(self):
        return self.wpt_num == len(self.waypoints)


wp = WaypointFollower()
while not wp.finished():
    wp.lc.handle()
