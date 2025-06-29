
import math
import csv
from ekf import EKF
import numpy as np

#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - Constants for grid setup
ROWS, COLS = 80, 80
EMPTY_SPACE = '.'
OCCUPIED_SPACE = '#'
occupancygrid = [[EMPTY_SPACE for _ in range(COLS)] for _ in range(ROWS)]

#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - Initialize robot and keyboard
TIME_STEP = 64



                        # # # # # # # # # # #
                         # Manual drive code
                        # # # # # # # # # # #
                        
#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - Initialize wheels
# In GPS_and_drive.py or wherever your controllers are defined

class PController:
    def __init__(self, kp):
        self.kp = kp
        self.previous_error = 0
    def update(self, error, dt):
        output = self.kp * error 
        self.previous_error = error
        return output

class RobotController:
    def __init__(self, robot):
        self.robot = robot
        self.TIME_STEP = int(robot.getBasicTimeStep())
        self.wheels = self.initialize_wheels()
        self.gps_sensors, self.gps_coordinates = self.initialize_gps()
        self.lidar = self.initialize_lidar()
        

        self.ekf = EKF(dt=self.TIME_STEP/1000.0)


        self.max_speed = 3.0
        
        self.steady_speed = self.max_speed / 2
        self.steady = 1

        self.pid_controller = PController(kp=0.1)
        self.dt = 0.1 
    def initialize_wheels(self):
        wheels = []
        wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
        for wheel_name in wheel_names:
            wheel = self.robot.getDevice(wheel_name)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0)
            wheels.append(wheel)
        return wheels
    
    def initialize_gps(self):
        gps_sensors = []
        gps_names = ['gpsFront', 'gpsCenter']
        gps_coordinates = [[0, 0, 0], [0, 0, 0]]
        for name in gps_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.TIME_STEP)
            gps_sensors.append(sensor)
        return gps_sensors, gps_coordinates
    
    def initialize_lidar(self):
        lidar = self.robot.getDevice('lidar')
        lidar.enable(self.TIME_STEP)
        lidar.enablePointCloud()
        return lidar
    
    def acquire_gps_coordinates(self):
        for i in range(2):
            self.gps_coordinates[i] = self.gps_sensors[i].getValues()
        gps_xy = np.array([self.gps_coordinates[1][0],
                           self.gps_coordinates[1][1]])
        self.ekf.update_gps(gps_xy)
        return self.gps_coordinates
    

    def odom_predict(self, omega):
        self.ekf.predict(omega)

    def calculate_robot_pose(self):
        # Use EKF state instead of raw GPS
        self.robot_pose = [self.ekf.x[0], self.ekf.x[1],
                           np.degrees(self.ekf.x[2]) % 360]
        return self.robot_pose



    def calculate_robot_pose(self):
        xCoordinateFront, yCoordinateFront = self.gps_coordinates[0][:2]
        xCoordinateMid, yCoordinateMid = self.gps_coordinates[1][:2]
        
        xdifference = xCoordinateMid - xCoordinateFront
        ydifference = yCoordinateMid - yCoordinateFront
        xCoordinateRobot = xCoordinateMid
        yCoordinateRobot = yCoordinateMid
        robotFacingAngleRad = math.atan2(ydifference, xdifference)
        robotFacingAngleDegrees = math.degrees(robotFacingAngleRad) % 360
        
        return [xCoordinateRobot, yCoordinateRobot, robotFacingAngleDegrees]
    
    def calculate_heading(self, robot_pose, next_step):
        current_x, current_y, current_orientation_deg = robot_pose
        target_x, target_y = next_step
        current_orientation_deg += 180
        if current_orientation_deg > 360:
            current_orientation_deg -= 360
         
        # Calculate the angle to the target
        delta_x = target_x - current_x
        delta_y = target_y - current_y
        target_heading_rad = math.atan2(delta_y, delta_x)
        target_heading_deg = math.degrees(target_heading_rad)
        
        # Adjusting for the custom orientation system:
        # In your system, North (0,0 to 0,1) should be 270 degrees
        # atan2(1,0) gives 90 degrees, so we need to shift this by -180 degrees to align north to 270 degrees
        heading_difference_deg = (target_heading_deg - current_orientation_deg) % 360

        if heading_difference_deg > 180:
            heading_difference_deg -= 360  # Normalize to range -180 to 180

        return heading_difference_deg
    

        
    def navigate_to_position(self, heading_difference, next_step):

        
        pid_output = self.pid_controller.update(heading_difference, 1)  # assuming dt=1 for simplicity
        turn_speed_adjustment = pid_output 
        # Directly adjust motor speeds based on PID output, clamping them within the max_speed limits
        print("turn sppeed", turn_speed_adjustment)

        # Calculate the motor speeds using the PID output
        left_speed = self.max_speed + turn_speed_adjustment
        right_speed = self.max_speed - turn_speed_adjustment

        # Clamp the motor speeds to ensure they don't exceed the robot's capability
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)

        # Set the velocities for each wheel
        for wheel in self.wheels:
            if wheel.getName() in ['wheel1', 'wheel2']:
                wheel.setVelocity(left_speed)
            else:
                wheel.setVelocity(right_speed)
    
    def check_proximity(self, robot_pose, target_position, tolerance=0.5):
        x, y = robot_pose[0], robot_pose[1]
        target_x, target_y = target_position
        distance = math.sqrt((x - target_x) ** 2 + (y - target_y) ** 2)
        return distance <= tolerance
        
    def align(self, robotpose):
        # Implement this method to get the robot's current heading
        heading_error = -robotpose[2]  # Negative error if we need to turn the robot to face 0 degrees

        # Allows a small error threshold
        pid_output = self.pid_controller.update(heading_error, 1)  # assuming dt=1 for simplicity
        turn_speed_adjustment = max(min(pid_output, 1), -1)

        # Adjusting wheel speeds to turn the robot towards 0 degrees
        left_speed = -turn_speed_adjustment
        right_speed = turn_speed_adjustment

        # Update the wheel velocities to turn the robot
        for wheel in self.wheels:
            if wheel.getName() in ['wheel1', 'wheel2']:
                wheel.setVelocity(left_speed)
            else:
                wheel.setVelocity(right_speed)

        # Assume some time delay or another method call to update the current heading
        
        heading_error = -robotpose[2]
    
    def stop_all_motors(self):
        for wheel in self.wheels:
            wheel.setVelocity(0)  
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - update the occupancy grid
    
