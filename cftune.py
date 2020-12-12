#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Hicham Belhseine"
__email__ = "hbelhsei@purdue.edu"

import logging
import time
import multiprocessing

import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger


class PositionTuner:
    def __init__(self, scf, queue_len=300, period_ms=50):
        # 15 sec time history be defualt
        self.scf = scf
        self.cf = self.scf.cf

        self.queue_len = queue_len  # time history list length
        self.period_ms = period_ms  # time history data recording interval

        # retained time history for plots [seconds]
        self.hist_length = queue_len * period_ms / 1000

        # Initializing time hitory arrays
        self.timestamps = multiprocessing.Array('d', queue_len)
        self.z = multiprocessing.Array('d', queue_len)
        self.targetZ = multiprocessing.Array('d', queue_len)

        # Recording step response indicator
        self.recording = False
        self.record_time_start = None

        # Log config for altitude
        self.log_conf_alt = LogConfig(name='Position', period_in_ms=period_ms)
        self.log_conf_alt.add_variable('kalman.stateZ', 'float')
        self.log_conf_alt.add_variable('posCtl.targetZ', 'float')
        self.scf.cf.log.add_config(self.log_conf_alt)
        self.log_conf_alt.data_received_cb.add_callback(self.altitude_callback)
        self.log_conf_alt.start()
        time.sleep(1)

    def altitude_callback(self, timestamp, data, logconf):
        z = data['kalman.stateZ']
        targetZ = data['posCtl.targetZ']

        if self.recording:
            if self.timestamps[-1] == 0:
                self.timestamps[:] = [timestamp]*self.queue_len

            # Pop oldest item and add latest
            self.timestamps[:-1] = self.timestamps[1:]
            self.targetZ[:-1] = self.targetZ[1:]
            self.z[:-1] = self.z[1:]

            self.timestamps[-1] = timestamp
            self.targetZ[-1] = targetZ
            self.z[-1] = z

            # Stop recording after time hist length has passed
            if time.time() - self.record_time_start >= self.hist_length:
                self.recording = False

    def record_response(self):
        self.recording = True
        self.record_time_start = time.time()

    def get_response(self):
        return self.timestamps, self.z, self.targetZ

    def step_info(self, time_hist, response, start, end):
        # Rough step response analysis
        response = self.smooth(response, int(self.queue_len / 50))

        # Finding rise time (time from 5% to 95%)
        # bad
        idx_low = (np.abs(response[:150] - (end - start)*.05)).argmin()
        idx_high = (np.abs(response[:200] - (end - start)*.95)).argmin()

        rise_time = (time_hist[idx_high] - time_hist[idx_low]) / 1000

        # Finding steady state error
        e_ss = np.abs(end - response[-1])

        # finding percent overshoot
        if end > start:
            overshoot = np.max(response)
        elif end > start:
            overshoot = np.min(response)
        else:
            overshoot = 0

        percent_overshoot = np.abs(overshoot / (end-start)) - 1

        # settling time
        lower = start + end - abs(end - start)*.05
        upper = start + end + abs(end - start)*.05

        # last index of response outside bounds
        unsettled = 1*((response[:280] >= lower) * (response[:280] <= upper))
        idx_unsettled = np.argwhere(unsettled == 0)
        idx_settle = idx_unsettled[-1][0]
        settle_time = (np.array(time_hist[idx_settle]) - time_hist[0]) / 1000

        return rise_time, e_ss, percent_overshoot, settle_time

    def smooth(self, signal, box_pts):
        box = np.ones(box_pts)/box_pts
        sig_smooth = np.convolve(signal, box, mode='same')
        return sig_smooth

    def get_alt_pid(self):
        Kp = float(self.cf.param.values["posCtlPid"]["zKp"])
        Ki = float(self.cf.param.values["posCtlPid"]["zKi"])
        Kd = float(self.cf.param.values["posCtlPid"]["zKd"])

        return Kp, Ki, Kd

    def set_alt_pid(self, Kp, Ki, Kd):
        if Kp < 0 or Ki < 0 or Kd < 0:
            print("PID gains cannot be set lower than zero")
            return -1
        else:
            self.cf.param.set_value("posCtlPid.zKp", Kp)
            self.cf.param.set_value("posCtlPid.zKi", Ki)
            self.cf.param.set_value("posCtlPid.zKd", Kd)
            print("Altitude posctl PID parameters updated.")
