"""Nav_control controller."""

import numpy as np
import WebotsNavAlgorithms as nav
import AStar as asr
from controller import Robot
from GPS_and_drive import RobotController  # Assuming this class is in the GPS_and_drive module
import csv
# Main function to control the robot



            
def main():
    # create the Robot instance.
    robot = Robot()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # Initialize the robot controller with the robot instance
    controller = RobotController(robot)

    # Define target position (update these values accordingly)
    target_position = (1.8, 7.4) # Example coordinates

    # Read occupancy grid from a CSV file and add buffer for path planning
    occupancy_grid = nav.readOccupancyGrid("SampleWorld.csv")
   
    buffer_width = 3 # Define buffer width as needed
    occupancy_grid = nav.addBufferToOccupancyGrid(occupancy_grid, buffer_width)

    # Scale factor to match path coordinates to grid indices

    
        # Mark the path points on the grid
        
    # Main simulation loop
    while robot.step(timestep) != -1:
        # Update GPS coordinates and calculate the robot's current pose
        current_position = controller.acquire_gps_coordinates()
        robot_pose = controller.calculate_robot_pose()
        print("Robot position", robot_pose)

        # Plan path to target using A*
        path, success = asr.aStar(robot_pose[0], robot_pose[1], target_position[0], target_position[1], occupancy_grid)
        path = asr.compute_bezier_points(path)

        if path:
            path.pop(0)
            next_step = path.pop(2)
            #write an exception to handle when pop(1) isnt available
             #reconstruct A star to find path

            
            heading_difference = controller.calculate_heading(robot_pose, next_step)

            
            controller.navigate_to_position(heading_difference, next_step)


        # Check if the robot has reached the target
        if controller.check_proximity(robot_pose, target_position, tolerance=0.1):
            print("Target reached. Please Wait for alignment")  
            break
            
            
                
    while robot.step(timestep) != -1:
    # Update GPS coordinates and calculate the robot's current pose
        current_position = controller.acquire_gps_coordinates()
        robot_pose = controller.calculate_robot_pose()
        
        if not 89 < abs(robot_pose[2]) < 91:
                controller.align(robot_pose)
        else:
            controller.stop_all_motors()
            print("Target reached. Current Robot Pose is: ", robot_pose)  
            break
        
        
        
if __name__ == "__main__":
    main()