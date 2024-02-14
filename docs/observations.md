# Observations {#observations}

Here is an index of observation dictionaries. Keys are a shorthand for nested dictionaries where ``a.b.c`` corresponds to ``observation["a"]["b"]["c"]``.

| Observation key | Description |
|-----------------|-------------|
| `imu` | Inertial measurement unit on the pi3hat. See also [`ImuData`](https://upkie.github.io/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `imu.angular_velocity` | Body angular velocity of the IMU frame in [rad] / [s] |
| `imu.linear_acceleration` | Body linear acceleration of the IMU in [m] / [s]² |
| `imu.orientation` | Unit quaternion of the orientation from the IMU frame to the attitude reference system (ARS) frame |
| `servos` | Servo motor measurements |
| `servos.<X>` | Observations for servo ``X`` in the servo layout @ref upkie::config::servo_layout |
| `servos.<X>.position` | Angle between the stator and the rotor in [rad] |
| `servos.<X>.torque` | Joint torque in [N] * [m] |
| `servos.<X>.velocity` | Angular velocity of the rotor w.r.t. stator in rotor, in [rad] / [s] |
| `wheel_odometry.position` | Ground position in [m], see @ref upkie::observers::WheelOdometry |
| `wheel_odometry.velocity` | Ground velocity in [m] / [s], see @ref upkie::observers::WheelOdometry |

See also [Sensors](@ref sensors).

## Observers {#observers}

<img src="https://upkie.github.io/upkie/observers.png" align="right">

<dl>
  <dt><a href="https://upkie.github.io/upkie/classupkie_1_1observers_1_1FloorContact.html#details">Floor contact</a></dt>
  <dd>Detect contact between the wheels and the floor. The pink and wheel balancers use contact as a reset flag for their integrators, to avoid over-spinning the wheels while the robot is in the air.</dd>

  <dt><a href="https://upkie.github.io/upkie/classupkie_1_1observers_1_1WheelContact.html#details">Wheel contact</a></dt>
  <dd>Detect contact between a given wheel and the floor.</dd>

  <dt><a href="https://upkie.github.io/upkie/classupkie_1_1observers_1_1WheelOdometry.html#details">Wheel odometry</a></dt>
  <dd>Measure the relative motion of the floating base with respect to the floor. Wheel odometry is part of their secondary task (after keeping the head straight), which is to stay around the same spot on the floor.</dd>
</dl>

