# Quadrocopter Controller

## Getting Started

Compiling the project:

```
# install Qt5
$ export Qt5Core_DIR=/usr/local/opt/qt/lib/cmake/Qt5Core
$ export Qt5Network_DIR=/usr/local/opt/qt/lib/cmake/Qt5Network
$ export Qt5Widgets_DIR=/usr/local/opt/qt/lib/cmake/Qt5Widgets

$ mkdir _build && cd _build
$ cmake ..
$ make -j8
$ ./CPPSim
```

## Solution: Scenario 1_Intro

Tuning the parameter ``Mass`` in the ``config/QuadControlParams.txt``:

```
Mass = 0.5
```

Run the result:

```
Simulation #1 (../config/1_Intro.txt)
Simulation #2 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
```

The chart will be shown as below:

![1_Intro](./images/1_Intro.png)


## Solution: Scenario 2_AttitudeControl

For the scenario 2_AttitudeControl, there are three tasks:

- motor control: generating the thrust of each single rotor by the collective thrust
and the moment at the body frame;
- body rate control: calculating the desired moment by the angular velocity (p, q, r); 
- roll pitch control: getting the angular velocity of roll and pitch at body frame.

**Motor Control**

As known that the relation of the torque (x, y, z), the collective thrust and the forces of
each rotor, resolve the desired force for each single rotor by the equations as

![equation](./images/generate_motor_control.gif)

**Body Rate Control**

Solving the ``momentCmd`` by the formulas as:

![equation](./images/body_rate_control.gif)

**Roll Pitch Control**

In order to get the angular velocity of roll and pitch, it needs the rotation matrix from
estimated attitude to tranform the result from world frame to body frame.

![equation](./images/roll_pitch_control.gif)

### Implementation

**GenerateMotorControl**

Codes implemented in ``GenerateMotorCommands()``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

   // cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
   // cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
   // cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
   // cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

   float l = L / sqrt(2.f);

   float t_p = momentCmd.x / l;
   float t_q = momentCmd.y / l;
   float t_r = -momentCmd.z / kappa;

   cmd.desiredThrustsN[0] = (t_p + t_q + t_r + collThrustCmd) / 4.f;
   cmd.desiredThrustsN[1] = (-t_p + t_q - t_r + collThrustCmd) / 4.f;
   cmd.desiredThrustsN[2] = (t_p - t_q - t_r + collThrustCmd) / 4.f;
   cmd.desiredThrustsN[3] = (-t_p - t_q + t_r + collThrustCmd) / 4.f;
   /////////////////////////////// END STUDENT CODE ////////////////////////////
```

**BodyRateControl**

Codes implemented in ``BodyRateControl``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   // moment(x, y, z) = I * omega_dot
   // omega_dot = kpPQR * e(t)
   // e(t) = pqr_des - pqr
   momentCmd = V3F(Ixx, Iyy, Izz) * kpPQR * (pqrCmd - pqr);
   /////////////////////////////// END STUDENT CODE ////////////////////////////
```

Tunning the parameter ``kpPQR`` in ``config/QuadControlParams.txt``:

```
# Angle rate gains
kpPQR = 92, 92, 20
```

At this stage, run the result, tuning the angle rate gains, the chart is shown as
below. We could see the ``omega.x`` is getting colsed to 0, and the vehicle flies
off quickly.

![2_AttitudeControl_step1](./images/2_AttitudeControl_step1.png)

**RollPitchControl**

Codes implemented in ``RollPitchControl()``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   float acc = - collThrustCmd / mass;
   float angleX = CONSTRAIN(accelCmd[0] / acc, -maxTiltAngle, maxTiltAngle);
   float angleY = CONSTRAIN(accelCmd[1] / acc, -maxTiltAngle, maxTiltAngle);
   float errorX = angleX - R(0, 2);
   float errorY = angleY - R(1, 2);
   float omegaX = kpBank * errorX;
   float omegaY = kpBank * errorY;

   if (collThrustCmd > 0)
   {
     pqrCmd.x = 1/R(2, 2) * (R(1, 0) * omegaX - R(0, 0) * omegaY);
     pqrCmd.y = 1/R(2, 2) * (R(1, 1) * omegaX - R(0, 1) * omegaY);
   }
   else
   {
     pqrCmd.x = 0;
     pqrCmd.y = 0;
   }
   /////////////////////////////// END STUDENT CODE ////////////////////////////
```

Tunning the parameter ``kpBank`` in ``config/QuadControlParams.txt``:

```
# Angle control gains
kpBank = 10
```

Run the result:

```
Simulation #175 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```

Completed the roll and pitch control function, and fine tuned the angle control gain
``kpBank``, the roll value is getting closed to 0 at the first graph as shown, at the
same time, the vehicle is flying away slowly. 

![2_AttitudeControl](./images/2_AttitudeControl.png)

## Solution: Scenario 3_PositionControl

### Implementation

