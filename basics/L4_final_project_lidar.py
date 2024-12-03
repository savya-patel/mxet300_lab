import sys
import numpy as np
import json
from time import sleep
import socket
from threading import Thread
import L1_lidar as lidar
import L1_ina as ina
import L2_vector as vec
import L2_kinematics as kin
import L2_compass_heading as compass
import L2_inverse_kinematics as ik
import L2_speed_control as sc
import L1_log as log
from time import sleep


class WallNavigation:
    """
    Wall-following navigation for SCUTTLE using LIDAR.
    Detects corners using LIDAR and turns 90 degrees when a perpendicular wall is detected.
    """

    def __init__(self):
        # Robot parameters
        self.global_position = np.array([0.0, 0.0], dtype=np.float64)  # Global position
        self.heading = 0.0  # For the IMU
        self.path = []  # Stores global positions for area calculation
        self.is_running = True  # Status flag for threads
        self.turns = 0  # To count the number of 90-degree turns

        # Wall-following parameters
        self.wall_distance = 0.4  # Desired distance from the wall (meters)

        # Threads
        self.lidar_thread = Thread(target=self.lidar_scan, daemon=True)
        self.drive_thread = Thread(target=self.drive_loop, daemon=True)
        self.position_thread = Thread(target=self.update_global_position_thread, daemon=True)

        self.lidar_thread.start()
        self.drive_thread.start()
        self.position_thread.start()

    def lidar_scan(self):
        """Continuously scans with LIDAR."""
        while self.is_running:
            data = lidar.polarScan(num_points=100)
            for d, t in data:
                cartesian_point = vec.polar2cart(d, t)
                # Logging for debugging
                log.tmpFile(cartesian_point[0], "lidar_x.txt")
                log.tmpFile(cartesian_point[1], "lidar_y.txt")
            sleep(0.1)

    def detect_corner(self,lidar_data):
        """
        Uses LIDAR to detect a straight perpendicular wall upfront, indicating a corner.
        Prints the 5 points directly in front as distances and checks for a wall.
        """
        points = [(d, t) for d, t in lidar_data if -5 <= t <= 5]  # Points directly in front

        # Extract only the distances of the first 5 points
        distances = [d for d, t in points[:10]]
        print("5 Points in Front (Distances):", distances)

        # If points form a straight line and distances are below a threshold
        if len(points) >= 5 and all(d < 0.3 for d in distances):  # Check if all distances are below 0.3m
            if max(distances) - min(distances) < 0.1:  # Check if points are aligned
                print("Wall found!")
                return True
        return False

    def drive_loop(self):
        """Controls the robot to move along walls and handle corners."""
        while self.is_running:
            lidar_data = lidar.polarScan(num_points=100)
            if self.detect_corner():  # Check for a corner using LIDAR
                self.turn_corner()
            else:
                self.follow_wall()

            if self.turns >= 4:  # Completed a full loop
                self.stop()
                self.calculate_area()

    def follow_wall(self, lidar_data):
        """
        Maintains the robot's alignment with the wall.
        Scans in the range of 120 to 0 degrees, finds the closest wall, and aligns the heading angle parallel to the wall.
        Keeps the robot at a minimum distance of 0.3m from the wall.
        """
        # Extract points in the range of 120 to 0 degrees
        points = [(d, t) for d, t in lidar_data if 0 <= t <= 120]

        # Extract distances
        distances = [d for d, t in points]
        print("Distances in 120 to 0 Degrees:", distances)

        # Check if points form a line (indicating a wall)
        if len(points) >= 5:  # Ensure sufficient points
            # Check if there's a stable wall
            if max(distances) - min(distances) < 0.1:  # Points are aligned
                wall_distance = sum(distances) / len(distances)  # Average distance to the wall
                wall_angle = sum(t for d, t in points if d < 0.3) / len(points)  # Average angle for points within 0.3m

                print(f"Wall detected at {wall_distance:.2f}m and angle {wall_angle:.2f} degrees")

                # Calculate heading alignment and distance correction
                angle_error = self.heading - wall_angle  # Difference between heading and wall angle
                distance_error = self.wall_distance - wall_distance  # Maintain 0.3m distance

                # Control speeds
                forward_speed = 0.2  # Base forward speed
                angular_speed = -0.1 * angle_error + -0.2 * distance_error  # Adjust angular speed for alignment and distance

                # Drive closed-loop to maintain alignment
                wheel_speeds = ik.getPdTargets(np.array([forward_speed, angular_speed]))
                sc.driveClosedLoop(wheel_speeds, [0.0, 0.0], [0.0, 0.0])
            else:
                print("Realigning to wall...")
                self.realign_to_wall(lidar_data)
        else:
            print("No wall detected, move forward")
            sc.driveClosedLoop([0.2, 0.2], [0.0, 0.0], [0.0, 0.0])  # Keep moving forward

            # angle_error = right_front - right_back
            # distance_error = self.wall_distance - distance
            # angular_speed = -0.2 * angle_error + -0.2 * distance_error
            # wheel_speeds = ik.getPdTargets(np.array([forward_speed, angular_speed]))
            # sc.driveClosedLoop(wheel_speeds, [0.0, 0.0], [0.0, 0.0])

    def turn_corner(self):
        """Executes a 90-degree turn."""
        initial_heading = self.heading
        target_heading = (initial_heading + 90) % 360

        while abs(self.heading - target_heading) > 5:  # Allowable error
            sc.driveClosedLoop([0.0, 0.5], [0.0, 0.0], [0.0, 0.0])  # Turn in place
            self.heading = compass.get_heading()

        self.turns += 1

    def update_global_position_thread(self):
        """Updates the robot's global position every 1 second."""
        while self.is_running:
            xdot, thetadot = kin.getMotion()[:2]  # Extract motion values
            self.heading = compass.get_heading()
            dy = xdot * np.cos(np.radians(self.heading))
            dx = xdot * np.sin(np.radians(self.heading))
            self.global_position += np.array([dx, dy])
            self.path.append(np.copy(self.global_position))  # Log position
            sleep(1)

    def calculate_area(self):
        """Calculates the area of the room using the Shoelace formula."""
        if len(self.path) < 3:
            print("Not enough points to calculate area.")
            return

        x_coords = [p[0] for p in self.path]
        y_coords = [p[1] for p in self.path]
        area = 0.5 * abs(sum(x_coords[i] * y_coords[i + 1] - y_coords[i] * x_coords[i + 1]
                             for i in range(len(self.path) - 1)))
        print("Calculated room area: {:.2f} square meters.".format(area))
        log.tmpFile(area, "room_area.txt")

    def stop(self):
        """Stops the robot."""
        self.is_running = False
        sc.driveClosedLoop([0.0, 0.0], [0.0, 0.0], [0.0, 0.0])


if __name__ == "__main__":
    robot = WallNavigation()
    try:
        while robot.is_running:
            sleep(1)
    except KeyboardInterrupt:
        robot.stop()
        print("Stopped robot.")
