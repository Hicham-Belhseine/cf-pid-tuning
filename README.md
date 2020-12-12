## Setup

A small mass consisting of double sided electrical tape with a micro USB receiver hass been added to the Crazyflie as seen below. 

![cf](https://github.com/purdue-aae490-iar/hw4-pid-tuning-Hicham-Belhseine/blob/master/media/cf_mass.jpg)

There is a deficiency with the test setup as the propellers are scratched and slightly damaged. A motor shaft is also loose causing oscillations during hover.

The software used in the assignment allows a user to provide an altitude step input to the Crazyflie, characterize the response from the unit step input (e.g. find overshoot or rise time), change the Crazyflie's PID gains, then immediately test a new response with the new PID gains. The terminal interface for the assignment is seen below

```
Select an item:
01) Takeoff and land while recording data.
02) Set new PID parameters.
10) Exit program
Item select:
```

Finally, the user can monitor the position and setpoint for the Crazyflie's x, y, and z position realtime with a plotter that runs in parallel with the Crazyflie control thread. An example of this monitor is seen below.

![Monitor](https://github.com/purdue-aae490-iar/hw4-pid-tuning-Hicham-Belhseine/blob/master/media/rt_monitor.png)

## Test Trials
The test trials with an attached mass can be found in the table below.

| Trial # | Kp | Ki | Kd | Overshoot (%) | Rise Time (s) | Settling Time (s) | Pass/Fail |
| ------- | -- | -- | -- | ------------- | ------------- | ----------------- | --------- |
| 1       |2.0 |0.5 | 0.0| 16.95         | .80           | 6.80              | Fail      |
| 2       |1.8 |0.3 | 0.5| 14.32         | 1.25          | 9.75              | Fail      |
| 3       |2.4 |0.2 | 0.3| 8.89          | 1.00          | 7.75              | Fail      |
| 4       |2.6 |0.4 | 0.1| 11.99         | 0.85          | 8.10              | Fail      |
| 5       |2.6 |0.4 | 0.2| 11.96         | 0.95          | 7.45              | Fail      |
| 6       |2.5 |0.5 | 0.2| 12.35         | 0.95          | 8.30              | Fail      |
| 7       |2.5 |0.2 | 0.6| 7.95          | 1.35          | 10.05             | Fail      |
| 8       |2.2 |0.25| 0.4| 10.3          | 1.15          | 8.75              | Fail      |
| 9       |2.9 |0.1 | 0.6| 4.05          | 1.40          | 1.50              | Fail      |
| 10      |3.2 |0.1 | 0.6| 3.60          | 1.20          | 1.35              | Fail      |
| 11      |3.5 |0.1 | 0.6| 3.43          | 1.00          | 1.10              | Pass      |


## Selected Trial
A trial meeting the requirements of a rise time of 1s, a percent overshoot of 10%, and a settling time of 3s was achieved after 10 iterations. The gains from this trial are Kp=3.5, Ki=.1, Kd=.6, achieving a percent overshoot of 3.43%, a rise time of 1.0s, and finally a settling time of 1.10s. The response of the selected trial can be seen below.

![Trial Image](https://github.com/purdue-aae490-iar/hw4-pid-tuning-Hicham-Belhseine/blob/master/trials/11/alt_ctl_step_20200328-001541.png)
