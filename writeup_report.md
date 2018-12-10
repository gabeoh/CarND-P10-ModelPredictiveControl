# Model Predictive Control

## Objective

The goal of this project is to implement Model Predictive Control (MPC).
The MPC predicts the vehicle trajectory based on the vehicle kinetic model.
For a given desired waypoint coordinates, MPC computes series of future
actuator values (steering wheel turn value and accelerations) while
optimizing the model cost.

---

## MPC Implementation

### Vehicle Model
The simulator provides vehicle position (x, y), heading direction (psi),
and speed.  In addition, it also provides x and y coordinates of series of
vehicle waypoints.

The state of vehicle is composed of four provided values (x, y, psi, v),
errors, and actuators.  Two types of errors are used in this vehicle 
kinetic model; the Cross Tracking Error (CTE) and the Orientation Error
(ePsi).  The steering angle and the acceleration are used as vehicle
actuator models.

MPC computes the series of vehicle states that minimize the model cost 
over the specified time steps (25 steps of 50ms) .

### Cost
The following factors contribute the model error:
- Cross Tracking Error (CTE)
- Orientation Error (ePsi)
- Difference between projected and reference speed
- Magnitude of steering angle
- Magnitude of acceleration value
- Changes in steering angle
- Changes in acceleration value

Each of these factors over each time step is squared and added to the
overall cost.  The cost contribution of each of these factors is normalized
to make meaningful contributions.

First, the magnitude of the speed value is much bigger than other factors
such as CTE and orientation error (ePsi).  Thus, the cost contributions
from the speed discrepancy is heavily discounted.

In addition, the magnitude of angular components such as the orientation
error and the steering angle is smaller than that of the CTE and the
acceleration values.  Thus, the larger multiplication factors are applied
to the angular components.

Lastly, through the experiments, it was discovered that the model has a
strong tendency to optimize the cost by maneuvering the steering wheel
rather than the acceleration.  This caused the vehicle to move in a
wobbly manner.  In real life, it is more common (and safef) to reduce the
speed than to perform crazy steering maneuver in order to track sharp turns.
In order to achieve this, heavy penalties on the steering angle magnitude
and its differentials, large reductions on the acceleration contributions,
and further reductions on the speed discrepancies are applied.
 
### Preprocessing Vehicle States and Waypoints
Both vehicle states and waypoints are transformed from the global map
perspective to the vehicle perspective.  This makes the position of vehicle
to be the origin (0, 0) with 0 angle.  The speed of the vehicle remains
identical through the transform.

The waypoint transformation was a bit more complicated.  It was achieved
by computing the distance between the points and vehicle, and the relative
angle from the vehicle heading direction.

### Parameter Settings
For the vehicle trajectory predictions, 25 time steps of 50ms duration are
used as the final values.  These values, along with the cost multiplication
factors described above and the referential speed of 50 mph, yielded
a smooth driving throughout the lap with the maximum speed over 45 mph.

This predicts the vehicle trajectory for about 1.25s (= 25 * 0.05s).
When a shorter time step of 10 is used, the prediction was short-sighted.
The vehicle achieved slower speed and it was troubled to clear some sharp
turns.
When a large time step of 50 is used, the prediction was extended much
further.  However, the extended predictions did not necessarily improve
the drive.  The vehicle achieved a slightly faster maximum speed, but
it traveled unsteadily at times.
When a larger time step of 100 is used, the model fails to compute
the solution in the allocated time slot to run the live simulation.

The solution of 25 time steps of 50ms is one of many possible solutions.
Other solutions with potentially better results would be possible by
carefully tuning other parameters such as `v_ref` and cost multiplication
factors along with time steps and duration.

| Parameter      | Value    | Description   |
| -------------- |---------:| ------------- |
| N              |       25 | Time steps    |
| dt             |    50 ms | Step duration |
| N_latency      |        2 | Number of steps to cover the latency (100ms) |
| v_ref          |   50 mph | Referential speed value |

 
### Model Predictive Control with Latency
The MPC achieves a smooth vehicle control by taking future vehicle
trajectories into consideration.
Due to its predictive nature, the MPC also benefits by accommodating
the latency between the control command and its actuation.

In this project, the latency of 100ms is simulated.  The MPC model stores
the pending actuator control values.  When computing the vehicle
trajectories, these pending values are used as fixed constraints for
the actuations of the initial time steps (2 steps in this case; 2 * 50ms
= 100ms).  Then, the first computed actuator controls (steering angle and
acceleration) after the latency time steps are returned as the control
values.
