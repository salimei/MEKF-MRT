"Thou shall document." -Marcus Aurelius (probably)

# The plan for Arrays (as of 2022-02-06)
1.  Write Basic EKF in Python. Do testing.
2.  Convert Python code to C. 
3.  Integrate into FC code. 

## Why use the MEKF?  

This particular implementation of the MEKF (Multiplicative Extended Kalman Filter) integrates gyroscope, accelerometer, and GPS data to provide a very precise estimate on position (and a much less precise estimate on orientation).

I won't reexplain MEKF theory, but here's a quick rundown of how it works on a conceptual level:
1. predict(): the MEKF uses a process model to compute the state at the next time step. Think of it like using the forward Euler's method to do numerical integration (oversimplified). To compute the next step, this particular implementation uses the current step and sensor input.
2. correct(): the issue with predict() is that errors (numerical and other) accumulate over time. The sensors used in the predict() step have a local frame of reference. To overcome this, we periodically run the correct() step where GPS data is used to correct the prediction and tighten the uncertainty bounds. The correct() step uses sensors with a global frame of reference (with respect to the rocket).

In this way, we get the precision from the gyroscope and accelerometers, and the accuracy of gps measurements. The MEKF is also very robust in the face of noise.

The advantages of using an MEKF over raw sensor data:
1. More precise position and orientation estimate.
2. The filter gives us uncertainty bounds on the estimates. 
3. The filter is stable and converges quickly. 
4. The filter can deal with the GPS going offline, and it can also overcome factory limitation on speed and/or altitude. 
5. more stuff I forgot to mention

## How to use the MEKF? 

### 1.The methods:
#### constructor()
Instantiate the MEKF, where:

* dt: time step (s) (dt is constant in this implementation of the MEKF but that's not always the case) 
        recommended value: around 0.01 s
* sigma_gyro: process noise on orientation. In other words, the uncertainty or noise on gyroscope measurements. In practice, the square root of covariance on the gyroscope measurements (rad). 
        recommended value: < 0.1 rad
* sigma_acc: same as sigma gyro but for the acclerometer (m/s^2)
        recommended value: < 0.1 m/s^2 
* sigma_gps: same as sigma gyro but for the GPS (m)
        recommended value: < 2.5 m 
* P_init_orien: confidence in orientation at time = 0 (in rad).
        recommended value: 0.1 rad (can be more)
* P_init_pos: confidence in position at time = 0 (m).
        recommended value: 0.5 m (can be more)      

#### predict()
Compute the next state, where:

* gyro_input: gyroscope sensor data (xyz)
* acc_input: accelerometer sensor data (xyz)

#### correct()
Correct the prediction, where:

* gps_input: GPS data **the frame of reference used is the original position of the rocket (see illustration)**

#### update()
To be called after correct() or predict(). It updates the variables (t->t+1).

### 2.The procedure

1.Install the rocket on the launchpad. 
2.Measure its initial angle and assume its position is 0m in the frame of reference you chose. 
3.In a loop:
    1.Call predict() at each time step. 
    2.Call update()
    3.Call correct() whenever GPS sensor data is available. 
    4.Call update()
4.Monitor the uncertainty on the estimates!!! Make sure the MEKF stays stable (ie. the uncertainty bounds should be stable and reasonable).
