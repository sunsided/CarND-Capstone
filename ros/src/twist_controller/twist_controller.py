import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                       decel_limit, accel_limit, wheel_radius,
                       wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                       stop_torque):
        
        # TODO: Fix magic numbers
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, 
                                            max_lat_accel, max_steer_angle)

        # TODO: Fix magic numbers
        kp = 0.3   # Taken from walkthrough lesson
        ki = 0.1   # Taken from walkthrough lesson
        kd = 0.    # Taken from walkthrough lesson
        mn = 0.    # Minimum throttle value, taken from walkthrough lesson
        mx = 0.2   # Maximum throttle value, taken from walkthrough lesson
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # TODO: Fix magic numbers
        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = .02   # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.stop_torque = stop_torque

        self.last_time = rospy.get_time()

    def get_sample_time(self):
        """
        Gets the sample time, i.e. the actual time between two calls
        of the controller.
        """
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        return sample_time

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Get the sample time. We're doing this even if DBW is disengaged
        # to make sure that the last_time value is always correct.
        sample_time = self.get_sample_time()

        # If DBW is disabled, we're returning default values by convention
        # and reset the controller so as to not build up errors in the 
        # PID's integrator; this is important, as otherwise the car might
        # otherwise behave erratically as soon as DBW is re-enabled.
        if not dbw_enabled:
            rospy.logdebug("Drive by wire disengaged; not controlling.")
            self.throttle_controller.reset()
            return 0., 0., 0.

        rospy.logdebug("Target velocity: {0}".format(linear_vel))
        rospy.logdebug("Target angular velocity: {0}".format(angular_vel))

        # We're applying a low-pass filter to the observed velocity
        # to suppress high-frequency noise.
        current_vel = self.vel_lpf.filt(current_vel)
        rospy.logdebug("Current velocity: {0}".format(current_vel))
        rospy.logdebug("Filtered velocity: {0}".format(self.vel_lpf.get()))

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # If the target velocity is zero and the car is driving very slow already,
        # we want to stop the car entirely.
        if linear_vel == 0. and abs(current_vel) <= self.brake_deadband:
            throttle = 0.
            brake = self.stop_torque  # Nm - to hold the car in place if stopped at a light.

        # If the velocity error is negative, the car is going faster than the controller
        # dictates it to be. Since we can't apply a negative throttle, we're need to brake instead.
        # For this, we're determining the torque required to decelerate the car to the target velocity.
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = abs(max(vel_error, self.decel_limit))
            brake = decel * self.vehicle_mass * self.wheel_radius # Nm (torque)

        return throttle, brake, steering
