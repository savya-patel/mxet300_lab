import L2_inverse_kinematics as ik
import L2_compass_heading as compass
import L2_speed_control as sc
import L1_ina as ina
import numpy as np
import L1_log as log
from time import sleep
import L2_vector as vec
import L2_kinematics as kin
from threading import Thread

# Define some variables that can be used to create the path
pi = np.pi                  # Utilize the numpy library to define pi
d1 = .4                      # Distance in meters of segment 1 in the path
d2 = .4                      # Distance in meters of segment 2 in the path
forward_velocity = 0.4      # Forward velocity (x dot) in m/s of SCUTTLE. NOTE that the max forward velocity of SCUTTLE is 0.4 m/s

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
        self.is_running = True  # Status flag
        self.turns = 0  # To count the number of 90-degree turns

    def log(self):
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

            log.tmpFile(self.global_position[0], "global_x.txt")  # Global x-coordinate
            log.tmpFile(self.global_position[1], "global_y.txt")  # Global y-coordinate

            sleep(0.1)

    def update_global_position(self):
        """Updates the robot's global position every second."""
        while self.is_running:
            xdot, thetadot = kin.getMotion()[:2]
            self.heading = compass.get_heading()
            dy = xdot * np.sin(np.radians(self.heading))
            dx = xdot * np.cos(np.radians(self.heading))
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
        print(f"Calculated room area: {area:.2f} square meters.")
        log.tmpFile(area, "room_area.txt")

    def stop(self):
        """Stops the robot."""
        self.is_running = False
        sc.driveClosedLoop([0.0, 0.0], [0.0, 0.0], [0.0, 0.0])

# Thread function to run the path creation
def execute_motion(path_segments, robot):
    for count, motion in enumerate(path_segments):
        print("Motion: ", count + 1, "\t Chassis Forward Velocity (m/s): {:.2f} \t Chassis Angular Velocity (rad/s): {:.2f} \t Duration (sec): {:.2f}".format(motion[0], motion[1], motion[2]))
        wheel_speeds = ik.getPdTargets(motion[:2])  # Get wheel speeds using inverse kinematics
        sc.driveOpenLoop(wheel_speeds)  # Run motors with the calculated wheel speeds

        # Log the values
        x = motion[0]
        theta = motion[1]
        log.tmpFile(x, "x_dot.txt")
        log.tmpFile(theta, "theta_dot.txt")

        voltage = ina.readVolts()
        log.tmpFile(voltage, "tmp.txt")
        sleep(motion[2])  # Wait the motion duration

    # Stop robot after all motions are completed
    robot.stop()

# Main execution
if __name__ == "__main__":
    robot = WallNavigation()

    # Create threads for concurrent tasks
    log_thread = Thread(target=robot.log, daemon=True)
    position_thread = Thread(target=robot.update_global_position, daemon=True)

    # Define path to follow
    motions = [
        [0.4, 0.0, 2],            # Motion 1
        [0.0, -1.57, 1.35],        # Turn 1
        [0.4, 0.0, 2],            # Motion 2
        [0.0, -1.57, 1.35],        # Turn 2
        [0.4, 0.0, 2],            # Motion 3
        [0.0, -1.57, 1.35],       # Turn 3
        [0.4, 0.0, 2],            # Motion 4
        [0.0, -1.57, 1.35],       # Turn 4
        [0.4, 0.0, 2],            # Motion 5
    ]

    # Start threads for logging and position updating
    log_thread.start()
    position_thread.start()

    # Execute motions in a separate thread
    motion_thread = Thread(target=execute_motion, args=(motions, robot), daemon=True)
    motion_thread.start()

    # Wait for the motion thread to finish before stopping the program
    motion_thread.join()

    # After all motions are completed, robot will stop and main execution will end
    print("All motions completed. Robot stopped.")

