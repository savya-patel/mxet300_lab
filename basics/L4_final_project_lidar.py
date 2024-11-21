#get an array of distances 
#go to the closest obstance and maintain a distance of 0.4m 
#logic to find straight lines 
#untill 0.4 to oj both side sensors HC-SR04
#third sensor in front to go in line

import numpy as np
import time
import L1_lidar as lidar  # Level 1 LIDAR functions for polar scans
import L2_vector as vec  # Level 2 functions for vector manipulation and conversion
import L2_speed_control as sc  # Level 2 speed control for motor commands
import L1_motor as motor  # Low-level motor control
from threading import Thread  # For running concurrent threads
import lidar_driving
import socket
import json

class WallNavigation:
    """
    This class handles wall-following navigation for SCUTTLE using LIDAR and NodeRED integration.
    It calculates the robot's global position, aligns it with walls, scans room walls, and outputs
    real-time data and room area.
    """

    def __init__(self):
        """
        Initialization of the WallNavigation system.
        Sets up the robot's parameters, communication with NodeRED, and starts necessary threads.
        """
        # Robot kinematics and geometry
        self.global_position = np.array([0, 0])  # Robot's global position [x, y]
        self.previous_position = np.array([0, 0])  # Used for path tracking
        self.global_angle = 0  # Robot's current orientation in radians
        self.path = []  # Stores all visited positions for calculating the room area
        self.start_position = None  # Saves the robot's starting position to detect loop completion

        # Wall-following parameters
        self.wall_distance = 0.3  # Desired distance from the wall (in meters)
        self.turn_threshold = 0.5  # Distance threshold to detect corners (in meters)

        # Status flag to control the threads
        self.is_running = True

        # NodeRED UDP communication setup, like Lidar_driving
        self.IP = "127.0.0.1"  # Localhost for UDP communication with NodeRED
        self.port = 3553
        self.dashBoardDatasock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dashBoardDatasock.bind((self.IP, self.port))
        self.dashBoardDatasock.settimeout(0.25)  # Timeout for receiving data

        # Initialize the SCUTTLE object
        self.scuttle = lidar_driving.SCUTTLE()

        # Start threads for LIDAR scanning
        self.lidar_thread = Thread(target=self.scan_and_update, daemon=True)
        self.lidar_thread.start()

        #Thread for driving logic
        self.drive_thread = Thread(target=self.drive_loop, daemon=True)
        self.drive_thread.start()

        #NodeRED Data Thread (from lidar_driving)
        self.dashBoardDataThread = Thread(target=self._dashBoardDataLoop, daemon=True)
        self.dashBoardDataThread.start()
        

    def _dashBoardDataLoop(self):
        while True:
            try:
                dashBoardData,recvAddr = self.dashBoardDatasock.recvfrom(1024)
                self.dashBoardData = json.loads(dashBoardData)

            except socket.timeout:
                self.dashBoardData = None

    def scan_and_update(self):
        """
        LIDAR thread function: continuously scans the environment and sends the data to NodeRED.
        Uses SCUTTLE's cartesian_scan method.
        """
        while self.is_running:
            # Use SCUTTLE's cartesian_scan method to get LIDAR data
            cartesian_data = self.scuttle.cartesian_scan()
            print("LIDAR Cartesian Data:", cartesian_data)

            # Optionally send to NodeRED
            message = json.dumps({"lidar": cartesian_data})
            self.dashBoardDatasock.sendto(message.encode(), ("127.0.0.1", 3555))
            time.sleep(0.05)  # Avoid flooding

    def drive_loop(self):
        """
        Driving logic thread: aligns the robot with walls, detects corners, and moves forward.
        """
        while self.is_running:
            # Find the nearest object using LIDAR
            nearest_object = vec.getNearest()

            if nearest_object[0] < self.turn_threshold:
                # If a corner is detected, perform a 90-degree turn
                self.turn_corner()
            else:
                # Otherwise, align with the wall and move forward
                self.align_with_wall(nearest_object)
                self.update_global_position()

            # Check if back at the starting position
            if self.start_position is None:
                # Save the starting position
                self.start_position = np.copy(self.global_position)
            elif np.linalg.norm(self.global_position - self.start_position) < 0.1:
                # If close enough to the start position, stop
                print("Completed room scan!")
                self.calculate_area()
                self.is_running = False

    def align_with_wall(self, nearest_object):
        """
        Aligns the robot with the wall by maintaining a constant distance.
        Uses proportional control to adjust the angle.
        """
        angle = nearest_object[1]  # Angle of the nearest object
        distance_error = self.wall_distance - nearest_object[0]  # Difference from the desired distance

        # Simple proportional control for alignment
        if abs(distance_error) > 0.05:
            theta_dot = -0.2 * distance_error  # Proportional adjustment
        else:
            theta_dot = 0

        # Drive forward with the adjusted angle
        sc.driveOpenLoop([0.2, theta_dot])

    def turn_corner(self):
        """
        Executes a 90-degree turn when a corner is detected.
        """
        target_angle = self.global_angle + np.pi / 2  # Turn 90 degrees
        while abs(self.global_angle - target_angle) > 0.05:
            sc.driveOpenLoop([0, 0.5])  # Turn in place
            self.global_angle += 0.05  # Simulated update of orientation
            time.sleep(0.1)

    def update_global_position(self):
        """
        Updates the robot's global position based on LIDAR readings.
        """
        nearest_object = vec.getNearest()
        cartesian = vec.polar2cart(nearest_object[0], nearest_object[1])
        self.global_position += cartesian  # Update position
        self.path.append(np.copy(self.global_position))  # Save for area calculation

    def calculate_area(self):
        """
        Calculates the area of the room using the Shoelace formula.
        Outputs the result to NodeRED.
        """
        if len(self.path) < 3:
            print("Not enough points to calculate area.")
            return

        # Extract x and y coordinates from the path
        x_coords = [p[0] for p in self.path]
        y_coords = [p[1] for p in self.path]

        # Shoelace formula for polygon area
        area = 0.5 * abs(
            sum(x_coords[i] * y_coords[i + 1] - y_coords[i] * x_coords[i + 1]
                for i in range(len(self.path) - 1))
        )
        print(f"Calculated room area: {area:.2f} square meters")

        # Send area result to NodeRED
        message = json.dumps({"area": area})
        self.dashBoardDatasock.sendto(message.encode(), ("127.0.0.1", 3555))

    def stop(self):
        """
        Stops all operations and shuts down threads safely.
        """
        self.is_running = False
        sc.driveOpenLoop([0, 0])  # Stop motors

if __name__ == "__main__":
    try:
        # Initialize and start the navigation system
        robot = WallNavigation()
        while robot.is_running:
            time.sleep(1)
    except KeyboardInterrupt:
        # Stop the robot on interrupt
        robot.stop()
        print("Stopped robot.")
