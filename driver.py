#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Hicham Belhseine"
__email__ = "hbelhsei@purdue.edu"

import logging
import time
import multiprocessing

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import style
from matplotlib import use
import numpy as np

import cftune
import cflog

# design requirements
overshoot_tgt = .1   # 10% overshoot
rise_time_tgt = 1    # 1s rise time (5% -> 95%)
settle_time_tgt = 3  # 3s settling time (5%)

style.use('seaborn-whitegrid')

URI = 'radio://0/80/2M/E7E7E7E711'

alt_takeoff = .5  # target takeoff altitude [m]
alt_target = 1    # setpoint altitude [m]

#               x     y  z         YAW [m, m, m, deg]
setpoint_pos = np.array(
               [.5, .5, alt_target, 0]
)


#
# Crazyflie control methods
#
def takeoff():
    for i in range(60):
        cf.commander.send_position_setpoint(.5,
                                            .5,
                                            alt_takeoff,
                                            0)
        time.sleep(.1)


def land():
    for i in range(10):
        cf.commander.send_position_setpoint(.5,
                                            .5,
                                            0.1,
                                            0.0)
        time.sleep(0.1)


def alt_setpoint(cf, t):
    alt_sp = setpoint_pos + np.array([0, 0, alt_takeoff, 0])

    time_start = time.time()
    while time.time() < (time_start + t):
        cf.commander.send_position_setpoint(alt_sp[0],
                                            alt_sp[1],
                                            alt_sp[2],
                                            alt_sp[3])
        time.sleep(0.1)

    time.sleep(0.1)


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break
    print("Estimator reset.")


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


#
# Plotting methods
#
def start_plots(x, y, z, pos_ts, tX, tY, tZ, tar_ts):
    position = [x, y, z, pos_ts]
    setpoint = [tX, tY, tZ, tar_ts]

    fig = plt.figure()
    fig.set_size_inches(15, 8)
    gs = GridSpec(2, 4)
    ax_x = fig.add_subplot(gs[0, 0])
    ax_y = fig.add_subplot(gs[0, 1])
    ax_z = fig.add_subplot(gs[1, :-2])
    ax_3d = fig.add_subplot(gs[0:, 2:], projection='3d')

    ani = FuncAnimation(fig, plot, interval=100,
                        fargs=(ax_x, ax_y, ax_z, ax_3d,
                               position, setpoint))
    plt.show()


def plot(i, ax_x, ax_y, ax_z, ax_3d, position, setpoint):
    x, y, z, pos_ts = position
    tX, tY, tZ, tar_ts = setpoint
    ax_x.clear()
    ax_y.clear()
    ax_z.clear()
    ax_3d.clear()

    x_line, = ax_x.plot(np.subtract(pos_ts, pos_ts[0]) / 1000, x)
    targetX_line, = ax_x.plot(np.subtract(tar_ts, tar_ts[0]) / 1000, tX)
    ax_x.set_title("X Position Time History")
    ax_x.set_xlabel("Time Elapsed (seconds)")
    ax_x.set_ylabel("Local X Position (m)")
    ax_x.legend((x_line, targetX_line),
                ("Local X Position", "Local X Setpoint"))

    y_line, = ax_y.plot(np.subtract(pos_ts, pos_ts[0]) / 1000, y)
    targetY_line, = ax_y.plot(np.subtract(tar_ts, tar_ts[0]) / 1000, tY)
    ax_y.set_title("Y Position Time History")
    ax_y.set_xlabel("Time Elapsed (seconds)")
    ax_y.set_ylabel("Local Y Position (m)")
    ax_y.legend((y_line, targetY_line),
                ("Local Y Position", "Local Y Setpoint"))

    z_line, = ax_z.plot(np.subtract(pos_ts, pos_ts[0]) / 1000, z)
    targetZ_line, = ax_z.plot(np.subtract(tar_ts, tar_ts[0]) / 1000, tZ)
    ax_z.set_title("Z Position Time History")
    ax_z.set_xlabel("Time Elapsed (seconds)")
    ax_z.set_ylabel("Local Z Position (m)")
    ax_z.legend((z_line, targetZ_line),
                ("Local Z Position", "Local Z Setpoint"))

    ax_3d.plot(x[-50:], y[-50:], z[-50:], label="Quadrotor Position")
    ax_3d.set_xlim3d(-3, 3)
    ax_3d.set_ylim3d(-3, 3)
    ax_3d.set_zlim3d(0, 3)
    ax_3d.legend(['x', 'y', 'z'])


def plot_step_response(tuner):
    # Get trial PID results and params
    timestamp, response, setpoint = tuner.get_response()
    Kp, Ki, Kd = tuner.get_alt_pid()

    # Get step response info
    response = np.array(response) - alt_takeoff
    setpoint = np.array(setpoint) - alt_takeoff
    rise_time, e_ss, p_over, settle_time = tuner.step_info(timestamp,
                                                           response,
                                                           0,
                                                           alt_target)  # noqa

    fig = plt.figure()
    fig.set_size_inches(14, 10.5)
    ax = plt.gca()
    ax.plot(np.subtract(timestamp, timestamp[0]) / 1000, response,
            label="Alt Response")
    ax.plot(np.subtract(timestamp, timestamp[0]) / 1000, setpoint,
            label="Alt Setpoint")
    ax.set_title("Altitude Step Response, Kp={:1.2f} Ki={:1.2f} Kd={:1.2f}".format(Kp, Ki, Kd))  # noqa
    ax.set_xlabel("Time Elapsed (seconds)")
    ax.set_ylabel("Altitude (m)")
    plt.suptitle("Rise Time: {:2.2f} s\nError SS: {:2.2f} m\nPercent Overshoot: {:1.2f}\nSettling Time: {:2.2f} s".format(rise_time, e_ss, p_over*100, settle_time))  # noqa
    ax.legend()
    fig.savefig("alt_ctl_step_" + time.strftime("%Y%m%d-%H%M%S"))
    print("Close the plot window to continue")
    plt.show()


def save_motor_data(t_strt, t_range, m1, m2, m3, m4, timestamps):
    # get motor data only during step response recording
    t_end = t_strt + (t_range * 1000)

    idx_strt = (np.abs(np.array(timestamps) - t_strt)).argmin()
    idx_end = (np.abs(np.array(timestamps) - t_end)).argmin()

    timestamps = timestamps[idx_strt:idx_end]
    m1 = m1[idx_strt:idx_end]
    m2 = m2[idx_strt:idx_end]
    m3 = m3[idx_strt:idx_end]
    m4 = m4[idx_strt:idx_end]
    m_all = np.add(m1, np.add(m2, np.add(m3, m4)))

    fig = plt.figure()
    fig.set_size_inches(11, 8)
    ax = plt.gca()
    ax.plot(np.subtract(timestamps, timestamps[0]) / 1000, m1,
            label='Motor 1 Output')
    ax.plot(np.subtract(timestamps, timestamps[0]) / 1000, m2,
            label='Motor 2 Output')
    ax.plot(np.subtract(timestamps, timestamps[0]) / 1000, m3,
            label='Motor 3 Output')
    ax.plot(np.subtract(timestamps, timestamps[0]) / 1000, m4,
            label='Motor 4 Output')
    ax.plot(np.subtract(timestamps, timestamps[0]) / 1000, m_all,
            label='Combined Motor Ouput')

    ax.set_title("Motor Response from Altitude Step Input")
    ax.set_xlabel("Time Elapsed (Seconds)")
    ax.set_ylabel("Motor Output")
    ax.legend()
    fig.savefig("motor_output_" + time.strftime("%Y%m%d-%H%M%S"))


if __name__ == "__main__":
    # Initialize low-level drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # cf position and setpoint logger
        log = cflog.CFLog(scf)

        # PID analyzer and parameter manager
        pidtune = cftune.PositionTuner(scf)

        # Dirty implementation of cf data piping
        x, y, z, pos_ts = log.get_position()
        tX, tY, tZ, tar_ts = log.get_target_position()
        m1, m2, m3, m4, motor_ts = log.get_motor_output()
        time.sleep(1)
        p_plot = multiprocessing.Process(target=start_plots,
                                         args=(x, y, z, pos_ts,
                                               tX, tY, tZ, tar_ts))
        p_plot.daemon = True
        p_plot.start()

        cf = scf.cf
        while True:
            user_input = -1
            print("Select an item:")
            print("01) Takeoff and land while recording data.")
            print("02) Set new PID parameters.")
            print("10) Exit program")
            try:
                user_input = int(input("Item select: "))
            except ValueError:
                print("Error, Unknown Input")
                continue

            if user_input == 1:
                Kp, Ki, Kd = pidtune.get_alt_pid()
                print("Current z-position PID controller gains:")
                print("\tKp: {:2.2f}".format(Kp))
                print("\tKi: {:2.2f}".format(Ki))
                print("\tKd: {:2.2f}".format(Kd))

                reset_estimator(scf)

                print("Taking off.")
                takeoff()
                pidtune.record_response()
                print("Ascending to setpoint altitude.")
                alt_setpoint(cf, 20)  # takeoff for 20 seconds
                print("Landing")
                land()

                # Flight data
                timestamps, z, targetZ = pidtune.get_response()
                rise_time, e_ss, p_over, settle_time = pidtune.step_info(
                                                            timestamps, np.array(z) - alt_takeoff,  # noqa
                                                            0,
                                                            alt_target  # noqa
                                                            )
                print("Flight results:")
                print("\tRise Time: {:2.2f} s, [{}]".format(rise_time,
                      'Success' if rise_time < rise_time_tgt else 'Failed'))
                print("\tError SS: {:2.2f} m".format(e_ss))
                print("\tOvershoot: {:2.2f} %, [{}]".format(p_over * 100,
                      'Success' if p_over < overshoot_tgt else 'Failed'))
                print("\tSettling Time: {:2.2f} s, [{}]".format(settle_time,
                      'Success' if settle_time < settle_time_tgt else 'Failed'))  # noqa
                time.sleep(.5)
                save_motor_data(timestamps[1], 15, m1, m2, m3, m4, motor_ts)
                plot_step_response(pidtune)

            elif user_input == 2:
                # Updating cf posCtlZ PID gains
                print("Enter new PID params")
                Kp_new = float(input("New Kp: "))
                Ki_new = float(input("New Ki: "))
                Kd_new = float(input("New Kd: "))
                pidtune.set_alt_pid(Kp_new, Ki_new, Kd_new)

            elif user_input == 10:
                print("Exiting Program.")
                break
            else:
                print("Error, unknown input.")
