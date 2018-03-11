import time
import rospy
import math

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):

    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_base,
                 wheel_radius,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle,
                 min_speed=0.1):
        # PID controller for throttle and braking
        #self.tb_pid = PID(kp=0.4, ki=0.01, kd=0.01, mn=-1.0, mx=accel_limit)
        self.tb_pid = PID(kp=0.7, ki=0.01, kd=0.01, mn=decel_limit, mx=accel_limit)
        # Low pass filter to smooth out throttle/braking actuations.
        #self.tb_lpf = LowPassFilter(0.2,1.0)
        self.tb_lpf = LowPassFilter(0.8)
        # Low pass filter to smooth out steering commands.
        #self.steer_lpf = LowPassFilter(0.2,1.0)
        self.steer_lpf = LowPassFilter(0.5)
        # Constants that are relevant to computing the final throttle/brake value
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        # Compute the braking conversion constant according to the formula `braking_torque = total_mass * max_deceleration * wheel_radius`.
        # `self.tb_pid` outputs barking values in `[-1,0]`, which will then be scaled by the constant below.
        self.braking_constant = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius
        # Record the time of the last control computation.
        self.last_time =  None
        # Controller for steering angle
        self.yaw_controller = YawController(wheel_base=wheel_base,
                                            steer_ratio=steer_ratio,
                                            min_speed=min_speed,
                                            max_lat_accel=max_lat_accel,
                                            max_steer_angle=max_steer_angle)

    def control(self,
                target_linear_velocity,
                target_angular_velocity,
                current_linear_velocity,
                dbw_enabled):

        if dbw_enabled:
            if self.last_time is None:
                sample_time = 0.02
                self.last_time = time.time()
            else:
                this_time = time.time()
                sample_time = this_time - self.last_time
                self.last_time = this_time


            # Compute the raw throttle/braking value.
            tb = self.tb_pid.step(error=(current_linear_velocity - target_linear_velocity), sample_time=sample_time)
            # Smoothen it.
            tb = self.tb_lpf.filt(tb)
            #rospy.loginfo("throttle/braking PID value: %s, target_linear_velocity: %s, current_linear_velocity: %s", tb, target_linear_velocity, current_linear_velocity)
            # Scale it.
            if target_linear_velocity== 0.0 and current_linear_velocity < 2.0:
                brake = abs(self.braking_constant * 5.0)
                throttle = 0
            elif tb < 0: # We're braking.
                # Convert the raw braking value to torque in Nm (Newton-meters).
                brake = abs(self.braking_constant * tb)
                throttle = 0
            elif target_linear_velocity > 10.0 and current_linear_velocity < 1.5: # We want to accelerate as much as we can
                brake = 0
                throttle = 1.0
            else: # We're accelerating.
                throttle = tb
                brake = 0
            # Compute the steering value.
            steer = self.yaw_controller.get_steering(linear_velocity=target_linear_velocity,
                                                     angular_velocity=target_angular_velocity,
                                                     current_velocity=current_linear_velocity)
            if (brake > 0) and (current_linear_velocity < 3.0): # Smooth out steering values at low velocities.
                steer = self.steer_lpf.filt(steer)
            else:
                self.steer_lpf.ready = False

            return throttle, brake, steer

        else: # If `dbw_enabled` is False.
            self.tb_pid.reset()
            return 0, 0, 0
