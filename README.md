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

Tuning the param: mass:

```
# start the simulator
$ ./CPPSim

# tuning the param on QuadControlParams.txt
$ vim config/QuadControlParams.txt
```

``config/QuadControlParams.txt``: final result

```
Mass = 0.5
```

## Solution: Scenario 2_AttitudeControl
