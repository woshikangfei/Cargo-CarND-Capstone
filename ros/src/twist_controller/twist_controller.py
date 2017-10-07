# import pid, lowpass and yaw_controller
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
	self.last_time = None
	self.wheel_base = kwargs['wheel_base']
	self.vehicle_mass = kwargs['vehicle_mass']
	self.fuel_capacity = kwargs['fuel_capacity']
	steer_ratio = kwargs['steer_ratio']
 	min_speed = 0.
	max_lat_accel = kwargs['max_lat_accel']
        accel_limit = kwargs['accel_limit']
	decel_limit = kwargs['decel_limit']
	self.max_steer_angle = kwargs['max_steer_angle']
	self.brake_deadband = kwargs['brake_deadband']
        
	self.control_pid = PID(-100, -0.1, -10, decel_limit, accel_limit)  
	#self.control_pid = PID(-5, -0.05, 0, decel_limit, accel_limit)    
	self.yaw_controller = YawController(self.wheel_base, steer_ratio, min_speed, max_lat_accel, self.max_steer_angle)
	self.lowpassfilter  = LowPassFilter(0.7, 0.1)


    def control(self, twist_cmd, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
	proposed_linear_velocity = twist_cmd.twist.linear.x
	proposed_angular_velocity = twist_cmd.twist.angular.z
 	current_linear_velocity = current_velocity.twist.linear.x
        current_angular_velocity = current_velocity.twist.angular.z
	
	# CTE: error_linear_velocity
	error_linear_velocity = current_linear_velocity - proposed_linear_velocity 
  
        if(dbw_enabled == False):
            self.control_pid.reset()
        
	if(dbw_enabled and self.last_time):
            # get time interval
            time = rospy.get_time()
            delta_time = time - self.last_time
            self.last_time = time
	    # get pid velocity
	    pid_control = self.control_pid.step(error_linear_velocity, delta_time)	 
            if pid_control > 0:
	        throttle = min(1.0, pid_control)  
                brake = 0.
            else:
                throttle = 0.
	        brake = (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY )*abs(pid_control)*self.wheel_base if abs(pid_control) > self.brake_deadband else 0

	    steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
	    steer = max(-self.max_steer_angle, min(steer, self.max_steer_angle))
	    
	else:
	    # reset self.last_time
	    self.last_time = rospy.get_time()
	    throttle = 1.
            brake = 0.
            steer = 0.
	
	steer = self.lowpassfilter.filt(steer)
        # Return throttle, brake, steer
        return throttle, brake, steer
