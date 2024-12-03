import numpy as np
import json
from time import sleep
import socket
from threading import Thread
import L1_lidar as lidar
import L1_ina as ina
import L2_vector as vec
import L2_kinematics as kin
import L2_inverse_kinematics as ik
import L2_speed_control as sc
import L1_log as log

class WallNavigation:
    """
    Wall-following navigation for SCUTTLE using LIDAR and ultrasonic sensors.
    Calculates the robot's global position using L2_kinematics and controls wheels with L2_inverse_kinematics.
    """

    def __init__(self):
        # Robot parameters
        #self.global_position = np.array([0.0, 0.0], dtype=np.float64)  # Explicitly float64
        #self.global_angle = 0  # Orientation (theta in radians)
        #self.path = []  # Stores global positions for area calculation
        self.start_position = None  # Starting point for loop detection
        self.is_running = True  # Status flag for threads

        # Wall-following parameters
        self.wall_distance = 0.4  # Desired distance from the wall (meters)
        self.turn_threshold = 0.3  # Threshold for corner detection (meters)

        #UPD communication#
        self.IP = "127.0.0.1"
        self.port = 3553
        self.dashBoardDatasock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dashBoardDatasock.bind((self.IP, self.port))
        self.dashBoardDatasock.settimeout(.25)
        
        #NodeRED data in#
        self.dashBoardData = None

        # Start threads
        self.log_thread = Thread(target=self.lidar_log, daemon=True)
        self.drive_thread = Thread(target=self.drive_loop, daemon=True)
        self.scan_thread = Thread(target=self.lidar_scan, daemon=True)
        
        self.log_thread.start()
        self.drive_thread.start()
        self.scan_thread.start()


    def lidar_scan(self):
        while True:
            data = self.cartesian_scan()
            data_msg = data.encode('utf-8')
            self.dashBoardDatasock.sendto(data_msg, ("127.0.0.1", 3555))
            sleep(.025)

    def cartesian_scan(self):
        rows = ''
        polar_data = lidar.polarScan(num_points=100)

        for d,t in polar_data:
            cartesian_point = vec.polar2cart(d,t)
            rows += self.format_row(cartesian_point)

        return rows[:-1]

    # Format the x,y lidar coordinates so that the bubble-chart can display them
    def format_row(self, point, r=3):
        x, y = point
        return '{x: ' + str(x) + ', y: ' + str(y) + ', r:' + str(r) + '},'

    def _dashBoardDataLoop(self):
        while True:
            try:
                dashBoardData,recvAddr = self.dashBoardDatasock.recvfrom(1024)
                self.dashBoardData = json.loads(dashBoardData)

            except socket.timeout:
                self.dashBoardData = None

    def lidar_log(self):
        """Continuously scans with LIDAR, logs data, and sends to NodeRED."""
        while self.is_running:
            # Get LIDAR data
            closest_obstacle = vec.getNearest()
            cartesian_coords = vec.polar2cart(closest_obstacle[0], closest_obstacle[1])

            # Log LIDAR and cartesian data
            voltage = ina.readVolts()
            log.tmpFile(voltage, "tmp.txt")
            log.tmpFile(closest_obstacle[0], "distance.txt")
            log.tmpFile(closest_obstacle[1], "angle.txt")
            log.tmpFile(cartesian_coords[0], "x_value.txt")
            log.tmpFile(cartesian_coords[1], "y_value.txt")

            #log.tmpFile(self.global_position[0], "global_x.txt")  # Global x-coordinate
            #log.tmpFile(self.global_position[1], "global_y.txt")  # Global y-coordinate

            sleep(0.1)

    def drive_loop(self):
        """Controls the robot to follow walls and calculate the area."""
        while self.is_running:
            # Get chassis motion from kinematics
            xdot, thetadot = kin.getMotion()[:2]  # Extract xdot (linear) and thetadot (angular)

            # Update global position
            #self.update_global_position(xdot, thetadot)

            # Get closest obstacle using LIDAR
            closest_obstacle = vec.getNearest()
            closest_obstacle = closest_obstacle.tolist()  # Convert to Python list
            distance=closest_obstacle[0]

            # Simulated ultrasonic sensor readings
            front_distance = 0.35
            right_front_distance = 0.25
            right_back_distance = 0.4

            log.tmpFile(front_distance, "front.txt")
            log.tmpFile(right_front_distance, "right_f.txt")
            log.tmpFile(right_back_distance, "right_b.txt")

            if front_distance < self.turn_threshold:
                self.turn_corner()
            else:
                self.align_with_wall(closest_obstacle[0], right_front_distance, right_back_distance)

            # Check if the robot has returned to the starting position commented out to let code continously for now
            '''
            if self.start_position is None:
                self.start_position = np.copy(self.global_position)
            elif np.linalg.norm(self.global_position - self.start_position) < 0.1:
                print("Completed loop. Stopping.")
                self.is_running = False
                self.calculate_area()
            '''

    def align_with_wall(self, distance, right_front, right_back):
        """Aligns the robot with the wall using ultrasonic sensors."""
        angle_error = right_front - right_back
        distance_error = self.wall_distance - distance

        # Proportional control
        forward_speed = 0.2
        angular_speed = -0.2 * angle_error + -0.2 * distance_error

        # Calculate wheel speeds using inverse kinematics
        wheel_speeds = ik.getPdTargets(np.array([forward_speed, angular_speed]))
        sc.driveOpenLoop(wheel_speeds)

    def turn_corner(self):
        """Executes a 90-degree turn."""
        wheel_speeds = ik.getPdTargets(np.array([0, 0.5]))  # Turn in place
        sc.driveOpenLoop(wheel_speeds)
        sleep(1.5)  # Adjust timing for a 90-degree turn
        sc.driveOpenLoop([0, 0])  # Stop motors

    def update_global_position(self, xdot, thetadot):
        """Updates the robot's global position using kinematics."""
        dx = xdot * np.cos(self.global_angle)
        dy = xdot * np.sin(self.global_angle)
        self.global_position += np.array([dx, dy])
        self.global_angle += thetadot
        self.path.append(np.copy(self.global_position))  # Log the new position

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
        sc.driveOpenLoop([0, 0])


if __name__ == "__main__":
    robot = WallNavigation()
    try:
        while robot.is_running:
            sleep(1)
    except KeyboardInterrupt:
        robot.stop()
        print("Stopped robot.")
