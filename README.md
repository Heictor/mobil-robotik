# mobil-robotik

:us: <br>
This repository is focused on various mobile robotics simulation applications, using the CoppeliaSim software.

:sweden: <br>
Detta förråd är fokuserat på olika mobila robotsimulering applikationer, med hjälp av CoppeliaSim-mjukvaran.

```markdown
# Overview of Developed Codes in "1_Robotben_Kinematik"
The following codes collectively simulate and analyze the kinematics of a single robot leg with precision and control.
---

## **1. `ex01.m`**
- **Functionality:** Computes the relative rotation matrices for the robot leg.
- **Details:**
  - Uses the generalized coordinates `[α, β, γ]` to calculate the rotation matrices.
  - Transforms vectors between coordinate frames using the computed matrices.

---

## **2. `ex02.m`**
- **Functionality:** Calculates the homogeneous transformation matrices and relative position vectors.
- **Details:**
  - Utilizes the rotation matrices derived in `ex01.m`.
  - Computes the transformation matrices to map the foot point position from one coordinate frame to another.
  - Assumes unit link lengths for simplicity.

---

## **3. `ex03.m`**
- **Functionality:** Determines the Jacobian matrix and generalized velocities.
- **Details:**
  - Computes the Jacobian matrix for the end-effector position based on the generalized coordinates.
  - Calculates the foot point velocity for a desired Cartesian motion using the Jacobian.

---

## **4. `ex04.m`**
- **Functionality:** Implements numeric inverse kinematics to achieve a target configuration.
- **Details:**
  - Uses Newton's method to iteratively solve the inverse kinematics problem.
  - Adjusts joint angles `[α, β, γ]` to position the foot at a specified goal point.
  - Synchronizes with V-REP (CoppeliaSim) to apply the calculated angles and validate the solution.

---

## **5. `ex05.m`**
- **Functionality:** Executes trajectory tracking using inverse differential kinematics.
- **Details:**
  - Implements a proportional controller to compute the desired Cartesian velocity of the foot point.
  - Uses inverse differential kinematics to calculate joint velocities `[dα, dβ, dγ]` for following a circular trajectory.
  - Exports and plots the desired and actual trajectories, enabling a comparison of the tracking accuracy.

---
```
# Overview of Developed Codes in "2_Differential_Drivna_Fordon"

This document outlines the purpose and functionality of each task implemented for the kinematics and control of a differential-drive robot in "Exercise 2."

---

## **1. `calculateWheelSpeeds.m`**
- **Functionality:** Implements the feed-forward control for wheel speed calculation.
- **Details:**
  - Computes the spinning speeds (`φr` and `φl`) for the right and left wheels based on the desired forward (`v`) and angular (`ω`) velocities.
  - Validates the implementation by simulating a 0.5-meter radius circular trajectory using the `testCircleDrive.m` script.

---

## **2. `teleoperation.m`**
- **Functionality:** Enables teleoperation of the robot via keyboard commands.
- **Details:**
  - Implements five basic commands for robot control:
    1. Move forward.
    2. Move backward.
    3. Rotate counterclockwise.
    4. Rotate clockwise.
    5. Stop immediately.
  - Smoothly varies speeds for precise control.
  - Supports testing in simulation (`realRobot = 0`) or with a real robot (`realRobot = 1`).

---

## **3. `calculateControlOutput.m` (Closed-Loop Control)**
- **Functionality:** Implements a closed-loop position controller for the robot to reach a specified target position.
- **Details:**
  - Uses a linear state feedback control law to compute velocity commands (`v` and `ω`).
  - Ensures precise and smooth control for reaching the target pose, adjusting both position and orientation.
  - Validates the controller by driving the robot toward a ghost target (`GhostPioneer_p3dxTarget`) in the V-REP simulation.

---

## **4. `calculateControlOutput.m` (Enhanced Closed-Loop Control)**
- **Functionality:** Enhances the closed-loop controller for smoother and more efficient motion.
- **Details:**
  - Modifies the controller to:
    - Allow forward and backward motions based on the target's relative position.
    - Maintain a constant speed while approaching the target, avoiding exponential deceleration.
  - Validates the enhancements by ensuring the robot reaches the target with improved trajectory and speed consistency.

---

### **Summary**

The tasks developed for "Exercise 2" implement a robust control system for a differential-drive robot:
1. **Feed-Forward Control:** Calculates wheel speeds for specific trajectories.
2. **Teleoperation:** Provides manual control through keyboard inputs.
3. **Closed-Loop Control:** Drives the robot to a target position with smooth and precise motion.
4. **Enhanced Closed-Loop Control:** Optimizes motion with bidirectional movement and constant speed.

These implementations enable effective simulation and testing of robot kinematics and control within MATLAB and V-REP.


# Overview of Developed Codes in "3_Sensor_Karakterisering"

This document provides an overview of the tasks implemented for sensor characterization and line fitting in "Exercise 3."

---

## **1. `GetRealLaserDistances.m`**
- **Functionality:** Characterizes the laser sensor and evaluates its accuracy.
- **Details:**
  - Connects to the REALabs platform to retrieve laser distance measurements from the Pioneer P3-DX robot.
  - Collects measurements from angles ranging from `-100°` to `100°` with steps of `20°`.
  - Computes the average and standard deviation for the measurements at each angle.
  - Validates data distribution using the Shapiro-Wilk test for different significance levels.
  - Plots the measurements to visually inspect their adherence to a normal distribution.

---

## **2. `fitLine.m`**
- **Functionality:** Implements line fitting for range scan data in Cartesian coordinates.
- **Details:**
  - Computes the line parameters (`r`, `α`) that best fit a set of points, minimizing the mean squared error.
  - Uses the centroid of the input points to determine the polar representation of the line.
  - Outputs the fitted line parameters for further processing.

---

## **3. `extractLines.m`**
- **Functionality:** Performs line extraction from range scan data using the Split-and-Merge algorithm.
- **Details:**
  - Divides the scan points into segments that approximately lie on a common line.
  - Employs the `fitLine` function to fit a line to each segment.
  - Outputs the line segments' endpoints and their corresponding parameters.

---

## **Validation Scripts**
- **`testLineFitting.m`:** Tests the `fitLine` function using artificial data. Validates the correctness of the line fitting implementation.
- **`testLineExtraction.m`:** Tests the entire line extraction pipeline on real scan data. Displays:
  - Measured points.
  - Expected lines (yellow).
  - Extracted lines and segments (red, green, blue).

---

### **Summary**

The tasks developed in "Exercise 3" provide a comprehensive framework for sensor characterization and line extraction:
1. **Sensor Characterization (`GetRealLaserDistances.m`):** Evaluates the accuracy and distribution of laser measurements.
2. **Line Fitting (`fitLine.m`):** Implements regression-based line fitting for range scan data.
3. **Line Extraction (`extractLines.m`):** Applies the Split-and-Merge algorithm to extract meaningful line features from the environment.

These implementations enable accurate localization and environment mapping for mobile robots using laser sensors.


# Overview of Developed Codes in "4_Kalman_Filter"

This document outlines the purpose and functionality of each task implemented for line-based Extended Kalman Filter (EKF) robot localization in "Exercise 4."

---

## **1. `transitionFunction.m`**
- **Functionality:** Implements the state prediction step of the EKF.
- **Details:**
  - Computes the robot's predicted state (`xt`) and covariance (`Pt`) using the motion model for a differential-drive robot.
  - Models motion noise as Gaussian and calculates Jacobians of the motion model with respect to state and control inputs.
  - Validates the implementation using `validateTransitionFunction()`, which compares the predicted trajectory with ground truth.

---

## **2. `measurementFunction.m`**
- **Functionality:** Implements the measurement model for line features in the EKF.
- **Details:**
  - Transforms line features from the map (world frame) into the robot’s body frame using the estimated state.
  - Computes the expected measurements (`zt`) and their Jacobian with respect to the state.
  - Validates correctness with `validateMeasurementFunction()`.

---

## **3. `associateMeasurements.m`**
- **Functionality:** Associates observed measurements with map entries.
- **Details:**
  - Uses Mahalanobis distance to match perceived lines with known map features.
  - Implements a validation gate to filter out incorrect associations.
  - No modifications required; validated using `validateAssociations()`.

---

## **4. `filterStep.m`**
- **Functionality:** Performs a complete EKF update step.
- **Details:**
  - Combines state prediction and update steps to refine the robot’s state estimate and covariance.
  - Uses innovations from `associateMeasurements.m` to adjust the prediction with sensory data.
  - Validated using `validateFilter()`, which compares the EKF implementation against a baseline.

---

## **5. `incrementalLocalization.m`**
- **Functionality:** Integrates EKF localization with laser scan data for real-time operation.
- **Details:**
  - Processes laser scan data to refine the robot’s pose incrementally.
  - Combines all previous components to achieve real-time localization in a simulated environment.
  - Validated using the V-REP simulation scene `Exercise4.ttt`. The estimated robot pose is visualized as a yellow ghost near the actual robot.

---

### **Validation Environment**
- **Simulation:** V-REP (CoppeliaSim) is used to simulate the localization in a laboratory environment with visible laser measurements.
- **Testing:** 
  - The robot follows a circular trajectory while maintaining accurate localization.
  - Performance is affected by environmental factors like map accuracy and feature associations.

---

### **Summary**

The tasks implemented in "Exercise 4" build a complete framework for robot localization using an Extended Kalman Filter:
1. **State Prediction (`transitionFunction.m`):** Predicts the robot's next state using motion models.
2. **Measurement Model (`measurementFunction.m`):** Transforms map features to match observations.
3. **Feature Association (`associateMeasurements.m`):** Matches perceived features to the map.
4. **EKF Update (`filterStep.m`):** Refines state estimates by combining predictions with observations.
5. **Incremental Localization (`incrementalLocalization.m`):** Implements real-time EKF localization.

These codes enable accurate robot pose estimation for navigation in structured environments.

