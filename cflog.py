#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Hicham Belhseine"
__email__ = "hbelhsei@purdue.edu"


"""
Simple real-time logging framework

Crazyflie data is stored in a multiprocessing shared array so that matplotlib
may run in the background while the main thread controls the crazyflie.
The GUI framework that matplotlib uses is not thread safe meaning it must be
run as another process.
"""

import logging
import time
import multiprocessing

import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class CFLog:
    "Simple logging framework that provides real-time state data"

    def __init__(self, scf, queue_len=300, period_ms=100):
        self.scf = scf
        self.cf = self.scf.cf

        self.queue_len = queue_len  # time history list length
        self.period_ms = period_ms  # time history data recording interval

        # retained time history for plots [seconds]
        self.hist_length = queue_len * period_ms / 1000

        # Initializing time hitory arrays
        self.pos_timestamp = multiprocessing.Array('d', queue_len)
        self.x = multiprocessing.Array('d', queue_len)
        self.y = multiprocessing.Array('d', queue_len)
        self.z = multiprocessing.Array('d', queue_len)

        self.target_timestamp = multiprocessing.Array('d', queue_len)
        self.targetX = multiprocessing.Array('d', queue_len)
        self.targetY = multiprocessing.Array('d', queue_len)
        self.targetZ = multiprocessing.Array('d', queue_len)

        self.motor_timestamp = multiprocessing.Array('d', queue_len)
        self.m1 = multiprocessing.Array('d', queue_len)
        self.m2 = multiprocessing.Array('d', queue_len)
        self.m3 = multiprocessing.Array('d', queue_len)
        self.m4 = multiprocessing.Array('d', queue_len)

        # Log config for state estimate
        self.log_conf_pos = LogConfig(name='Position', period_in_ms=period_ms)
        self.log_conf_pos.add_variable('kalman.stateX', 'float')
        self.log_conf_pos.add_variable('kalman.stateY', 'float')
        self.log_conf_pos.add_variable('kalman.stateZ', 'float')
        self.scf.cf.log.add_config(self.log_conf_pos)
        self.log_conf_pos.data_received_cb.add_callback(self.position_callback)

        # Log config for position controller setpoint
        self.log_conf_target = LogConfig(name='Position Target',
                                         period_in_ms=period_ms)
        self.log_conf_target.add_variable('posCtl.targetX', 'float')
        self.log_conf_target.add_variable('posCtl.targetY', 'float')
        self.log_conf_target.add_variable('posCtl.targetZ', 'float')
        self.scf.cf.log.add_config(self.log_conf_target)
        self.log_conf_target.data_received_cb.add_callback(
            self.target_callback
            )

        # Log config for motor output
        self.log_conf_motor = LogConfig(name="Motor Output",
                                        period_in_ms=period_ms)
        self.log_conf_motor.add_variable('motor.m1', 'int32_t')
        self.log_conf_motor.add_variable('motor.m2', 'int32_t')
        self.log_conf_motor.add_variable('motor.m3', 'int32_t')
        self.log_conf_motor.add_variable('motor.m4', 'int32_t')
        self.scf.cf.log.add_config(self.log_conf_motor)
        self.log_conf_motor.data_received_cb.add_callback(
            self.motor_callback
        )

        # Starting logs
        self.log_conf_pos.start()
        self.log_conf_target.start()
        self.log_conf_motor.start()

        # Wait for logs to fill
        time.sleep(1)

    def position_callback(self, timestamp, data, logconf):
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']

        if self.pos_timestamp[-1] == 0:
            self.pos_timestamp[:] = [timestamp]*self.queue_len

        # Pop oldest item and add latest
        self.pos_timestamp[:-1] = self.pos_timestamp[1:]
        self.pos_timestamp[-1] = timestamp
        self.x[:-1] = self.x[1:]
        self.y[:-1] = self.y[1:]
        self.z[:-1] = self.z[1:]
        self.x[-1] = x
        self.y[-1] = y
        self.z[-1] = z

    def target_callback(self, timestamp, data, logconf):
        targetX = data['posCtl.targetX']
        targetY = data['posCtl.targetY']
        targetZ = data['posCtl.targetZ']

        if self.target_timestamp[-1] == 0:
            self.target_timestamp[:] = [timestamp]*self.queue_len

        # Pop oldest item and add latest
        self.target_timestamp[:-1] = self.target_timestamp[1:]
        self.target_timestamp[-1] = timestamp
        self.targetX[:-1] = self.targetX[1:]
        self.targetY[:-1] = self.targetY[1:]
        self.targetZ[:-1] = self.targetZ[1:]
        self.targetX[-1] = targetX
        self.targetY[-1] = targetY
        self.targetZ[-1] = targetZ

    def motor_callback(self, timestamp, data, logconf):
        m1 = data['motor.m1']
        m2 = data['motor.m2']
        m3 = data['motor.m3']
        m4 = data['motor.m4']

        if self.motor_timestamp[-1] == 0:
            self.motor_timestamp[:] = [timestamp]*self.queue_len

        # Pop oldest item and add latest
        self.motor_timestamp[:-1] = self.motor_timestamp[1:]
        self.motor_timestamp[-1] = timestamp
        self.m1[:-1] = self.m1[1:]
        self.m2[:-1] = self.m2[1:]
        self.m3[:-1] = self.m3[1:]
        self.m4[:-1] = self.m4[1:]
        self.m1[-1] = m1
        self.m2[-1] = m2
        self.m3[-1] = m3
        self.m4[-1] = m4

    def get_position(self):
        return self.x, self.y, self.z, self.pos_timestamp

    def get_target_position(self):
        return self.targetX, self.targetY, self.targetZ, self.target_timestamp

    def get_motor_output(self):
        return self.m1, self.m2, self.m3, self.m4, self.motor_timestamp
