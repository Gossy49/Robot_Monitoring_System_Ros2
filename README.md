# ROBOT MONITORING SYSTEM IN ROS 2

This project implements a ROS 2–based robot monitoring system that detects sensor failures and automatically recovers from them.
The system monitors the health of multiple sensors, reports errors when failures occur, and restarts faulty sensors using custom ROS 2 services.

---

## SYSTEM OVERVIEW

The system consists of the following components:

- Sensor nodes that publish their health state
- State monitor node that evaluates the overall robot health
- Error handler node that performs recovery actions
- Custom service interface for reporting sensor failures

This project demonstrates fault detection, recovery, and inter-node communication using ROS 2 topics, services, parameters, and launch files.
---

## HOW THE SYSTEM WORKS

SENSOR NODES
- Sensor 1 (sensor1)
  - Continuously publishes a Bool message with value TRUE
  - Indicates the sensor is healthy
  - Provides a restart service (/sensor1/restart)

- Sensor 2 (sensor2)
  - Publishes TRUE initially
  - After approximately 5 seconds, simulates a failure by publishing FALSE
  - Provides a restart service (/sensor2/restart) that resets its state

STATE MONITOR
- Subscribes to:
  - /sensor1_state
  - /sensor2_state
- Publishes the overall robot health on /robot_state
- Detects failures when any sensor publishes FALSE
- Reports sensor failures to the error handler using a custom service

ERROR HANDLER
- Receives error reports through the /report_error service
- Calls the restart service of the failed sensor
- Uses parameters defined in YAML configuration files

RECOVERY
- Once a failed sensor is restarted, it resumes publishing TRUE
- The robot returns to a healthy state automatically

---

## CUSTOM INTERFACES


This project uses a custom ROS 2 service for error reporting.

Service: ReportError.srv

Fields:
- string sensor_name
- bool accepted
- string info

This service allows the state monitor to notify the error handler about which sensor failed and whether the recovery action was accepted.

---

## PACKAGE STRUCTURE


This repository contains two ROS 2 packages:

1) lab2_py_pkg
   Contains all Python nodes:
   - sensor1.py
   - sensor2.py
   - state_monitor.py
   - error_handler.py

   Additional folders:
   - launch/   (launch files)
   - config/   (YAML parameter files)
   - resource/ (ROS 2 package resources)

2) custom_interfaces
   Contains custom interface definitions but below is the custom interface used:
   - srv/ReportError.srv

---

## WORKSPACE LAYOUT (IMPORTANT)


For ROS 2 best practices, custom interfaces should NOT be nested inside node packages. The packages should be placed side-by-side in the ROS 2 workspace src/ directory.

Recommended layout:

ros2_ws/src/
  lab2_py_pkg/
  custom_interfaces/

This layout avoids build and dependency issues and follows standard ROS 2 conventions.

---

## HOW TO RUN


STEP 1: Create a ROS 2 workspace

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

STEP 2: Clone the repository

git clone git@github.com:Gossy49/Robot_Monitoring_System_Ros2.git

STEP 3: Arrange packages in the workspace

Ensure your workspace looks like this:

ros2_ws/src/
  lab2_py_pkg/
  custom_interfaces/

If the folders are nested, move them so they sit directly under src/.

STEP 4: Install dependencies

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

STEP 5: Build the workspace

colcon build

STEP 6: Source the workspace
(This step is required in EVERY terminal)

source install/setup.bash

STEP 7: Run the system (3 terminals)

Terminal 1 – Launch state monitor and error handler
ros2 launch lab2_py_pkg system_launch.py

Terminal 2 – Run sensor 1
ros2 run lab2_py_pkg sensor1

Terminal 3 – Run sensor 2
ros2 run lab2_py_pkg sensor2

---

## EXPECTED BEHAVIOR


- Both sensors start in a healthy state
- After about 5 seconds, sensor2 simulates a failure
- The state monitor detects the failure
- The error handler restarts the failed sensor
- The robot returns to a healthy state automatically

---

