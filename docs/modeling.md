# Quadrocopter Controller

## Modeling of Quadrocopter

### Termilogies

Physical properties

- mass
- L: the arm length
- Ixx/Iyy/Izz: the moment of inertia along x/y/z-axis
- kappa: the drag/thrust ratio
- minMotorThrust/maxMotorThrust

Gains:
- kpPosXY, kpPosZ: the gain of position
- KiPosZ:
- kpVelXY, kpVelZ: the gain of velocity
- kpBank, kpYaw: angle control gains
- kpPQR: angle rate gains

Limits:
- maxAscentRate / maxDescentRate:
- maxSpeedXY/maxHorizAccel:
- maxTiltAngle:

Variables:
- posCmd, pos: desired position, current/estimated position
- velCmd, vel: desired velocity, current/estimated velocity 


### Modeling

```
RunControl:

--> TrajectoryPoint()           --> TrajPoint(time, position, velocity, omega, accel, attitude)

--> AltitudeControl()           --> collective thrust
--> LateralPositionControl()    --> acceleration 

--> RollPitchControl()          --> pqr
--> YawControl()                --> yaw_rate
--> BodyRateControl()           --> moment
--> GenerateMotorCommands()     --> thrust on motor
```

#### Altitude Control

```
velZCmd = velocity_z + kpPosZ * (posZCmd - posZ)
integratedAltitudeError += (posZCmd - posZ) * dt
-maxAscentRate < velocity_z < maxDescentRate

desAccel = kpVelZ * (velZCmd - velZ) + KiPosZ * integratedAltitudeError + accelZCmd - 9.81f
R = attitude.RotationMatrix_IwrtB()
thrust = -(desAccel / R(2, 2) * mass

thrust_margin = 0.1 * (maxMotorThrust - minMotorThurst)
(minMotorThrust + thrustMargin) * 4 < thrust < (maxMotorThrust - thrustMargin) * 4
```

- velZCmd: the desired velocity in NED
- velZ: the current velocity in NED
- posZCmd: the desired vertical position in NED
- posZ: the current vertical position in NED
- accelZCmd: feed-forward vertical acceleration in NED
- attitude: Quaternion<float>, 
- dt: seconds, the time step of the measurements
- R: the rotation matrix
- kpPosZ: the gain parameter
- kpVelZ: the gain parameter
- maxAscentRate: the maximum vertical speeds
- maxDescentRate: the maximum vertical speeds



#### Lateral Position Control --> acceleration

```
# don't have any incoming z-component
accelCmdFF.z = 0
velCmd.z = 0
posCmd.z = pos.z

velCmd = velCmd + kpPosXY * (posCmd - pos)
- velCmd.mag() > maxSpeedXY: velCmd = velCmd * maxSpeedXY / velCmd.mag()

accelCmd = accelCmd + kpVelXY * (velCmd - vel)
- accelCmd.mag() > maxAccelXY: accelCmd = accelCmd * maxAccelXY / accelCmd.mag()
```

- posCmd: position
- velCmd: velocity
- accelCmdFF: feed-forward acceleration
- kpPosXY:
- kpVelXY
- maxSpeedXY:
- maxAccelXY:


#### Roll Pitch Control --> pqr

```
R = attitude.RotationMatrix_IwrtB()

target_R13 = -maxTiltAngle < accelCmd[0] / (collThrustCmd / mass) < maxTiltAngle
target_R23 = -maxTiltAngle < accelCmd[1] / (collThrustCmd / mass) < maxTiltAngle
- if collThrustCmd < 0: target_R13 = target_R23 = 0

pqrCmd.x = (1 / R(2, 2)) * (-R(1, 0) * kpBank * R(0, 2) - target_R13) + R(0, 0) * kpBank*(R(1, 2) - target_R23))
pqrCmd.y = (1 / R(2, 2))*(-R(1, 1) * kpBank*(R(0, 2) - target_R13) + R(0, 1) * kpBank*(R(1, 2) - target_R23))
```

- kpBank: roll/pitch gain 


#### Yaw Control - yaw_rate

```
yaw_error = yawCmd - yaw
yaw_error = yaw_error % (F_PI * 2)
- if yaw_error > F_PI: yaw_error = yaw_error - 2 * F_PI
- if yaw_error < -F_PI: yaw_error = yaw_error + 2 * F_PI

yaw_rate = yaw_error * kpYaw
```

- kpYaw: the yaw control gain parameter


#### Body Rate Control --> moment

```
I = (Ixx, Iyy, Izz)
rate_error (rad/s) = pqrCmd - pqr
moment = rate_error * kpPQR * I
```

- I (Ixx, Iyy, Izz):
- pqrCmd: desired body rates [rad/s]
- pqr: current/estimated body rates
- kpPQR: the gain parameter


#### Generate Motor Commands --> thrust

```
l = L / 2 / sqrt(2)
diffThrust.x = moment.x / l
diffThrust.y = moment.y / l
diffThrust.z = moment.z / 4 / kappa 

average_thrust = collective_thrust / 4
desired_thrust_front_left = average_thrust - diffThrust.z + diffThrust.y + diffThrust.x
desired_thrust_front_right = average_thrust + diffThrust.z + diffThrust.y - diffThrust.x
desired_thrust_rear_left = average_thrust + diffThrust.z - diffThrust.y + diffThrust.x
desired_thrust_rear_right = average_thrust - diffThrust.z - diffThrust.y - diffThrust.x
```

- thrust: the desired collective thrust
- moment: the desired rotation moment of each axis
- L: the arm length parameter
- kappa: the drag/thrust ratio

## Solutions

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
