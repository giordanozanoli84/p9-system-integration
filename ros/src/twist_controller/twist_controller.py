GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


class Controller(object):
    def __init__(self, vehicle_mass=None, fuel_capacity=None,
                 brake_deadband=None, decel_limit=None, accel_limit=None,
                 wheel_radius=None, wheel_base=None, steer_ratio=None,
                 max_lat_accel=None, max_steer_angle=None):

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.controller_steering = YawController(
            wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        self.controller_throttle = PID(0.3, 0.01, 0.1, 0.0, 0.2)

        self.lowpass = LowPassFilter(0.5, 1. / 50.)

        self.last_time = rospy.get_time()
        self.last_vel = 0.0

    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.controller_throttle.reset()
            return 0.0, 0.0, 0.0

        # low pass the current velocity
        current_vel = self.lowpass.filt(current_vel)

        # update time
        current_time = rospy.get_time()
        step_time = current_time - self.last_time
        self.last_time = current_time

        # steering
        steering = self.controller_steering.get_steering(
            linear_vel, angular_vel, current_vel)

        # velocity
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        # logic on throttle and brake
        throttle = self.controller_throttle.step(vel_error, step_time)
        brake = 0.0

        if linear_vel == 0 and current_vel < 0.1:
            # we want to stop
            throttle = 0.0
            brake = 700  # N*m

        elif throttle < 0.1 and vel_error < 0.0:
            # we want to decelerate
            throttle = 0.0
            decelerate = max(vel_error, self.decel_limit)
            brake = abs(decelerate) * self.vehicle_mass * self.wheel_radius

        rospy.logwarn('Throttle {} - Brake {} - Steer {}'.format(throttle, brake, steering))

        return throttle, brake, steering
