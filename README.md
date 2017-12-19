# ECE 383 Final Notes

## Lab 5
### Overview
Estimate position and velocities of objects thrown towards a goal using camera sensor. (No air resistance)
Camera:
- Located at fixed location near goal.
- `Update()` function in code chooses between different state estimators for different sensors.
- Coordinates of the camera frame w.r.t. the world frame is know.
	- Origin is at the camera focal point
	- x-direction points to "right"
	- y-direction points to "down" in camera imatge
Omniscient Mode
- Omniscient Sensor: provides precise readings of object positions and velocities
	- `OmniscientStateEstimator` - processes the raw measurements into a `MultiObjectStateEstimate` object.
- Indexed by name: (r,g,b,a) tuple giving color.
Position Mode
- Position sensor: Noisy position estimates for each ball, within +- 0.1 unit of the true position. 
	- `PositionStateEstimator` class processes a stream of `ObjectPositionOutput` objects to produce updated `MultiObjectStateEstimate` object.
Blob Detector Mode
- Blob Detector: rectangular windows of detected "blobs" that indicate various objects of interest.
- Blobs used to estimate object positions and velocities in 3D space.
	- Stored in `CameraColorDetectorOutput` class.
- `BlobStateEstimator` class: process a stream of `CameraColorDetectorOutput` objects to produce an updated `MultiObjectStateEstimate` object.

### Part A - Kalman Filter with Position Sensor
Implement a Kalman Filter for the Position Sensing Mode
- Part 1
	- Linear Gaussian forward dynamics model and observation model.
	- Implement Kalman filter to properly maintain a state estimate so that the estimated position and velocity estimate the correct ballistic trajectory quickly after the object is launched.
	- Store time-sensitive data in `self`. 

### Part B
Implement an Extended Kalman Filter with the Blob Detector Sensing Mode.
- Build a forward dynamics model and observation model.
- Implement an EKF by linearizing each update about the current state estimate. You will need to perform initialization of each tracked object when a new blob appears.


## Final Project
Event A: In your event, balls will be hurled toward a goal, and your robot should block as many of them as possible from passing through the goal.
- -10 points for every ball scored. 
- -Additional Points for penalties, collisions, joint limit violations, taking too long...

### Robot and Sensors
6DOF Fixed Base Industrial Robot
- Robot's joint encoders are available
Camera Sensor
- origin: camera focal point
- x-direction to "right" of camera image
- y-direction to "down" of camera image
Omniscient vs. Blob Detector
- Omniscient is for debugging
- Blog Detector will be used in final implementation

### High-Level Controller
Implementing...
- Control Loop (read from sensors, process information, send command to the robot's low level controller)
	- Processing: perception (state estimation of moving objects)
	- Control (internal state of different control components)
	- Planning (deciding where and how to move in response to detected objects + robot's current state)

### State Estimation
`MultiObjectStateEstimate` is available to us with Omniscient mode. We will have to process a stream of CameraColorDetectorOutput objects into a stream of `MultiObjectStateEstimate` objects.


### System Implementation
Part A: High level controller subsystem
	- [20 points]
Part B: Perception (state estimation) subsystem.
	- [20 points] Tsensor is the sensor transform. 

#### Construction of Controller


