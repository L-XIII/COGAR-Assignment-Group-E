#!/usr/bin/env python

import rospy
import random
import math
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class OccupancyGridMap:
    """
    OccupancyGridMap acts as the interface with the SLAM component.
    It represents the environment using a grid and stores the occupancy status (free, occupied, unknown) for each cell.
    """

    def __init__(self, width=100, height=100, resolution=0.1):
        self.width = width  # Width of the grid (number of cells)
        self.height = height  # Height of the grid (number of cells)
        self.resolution = resolution  # Meters per cell
        self.origin = [0, 0]  # Origin position (meters) [x, y]
        # 0 = free, 100 = occupied, -1 = unknown
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        self.updated = False

    def update_cell(self, x, y, value):
        """
        Update the cell in the occupancy grid.
        x, y: cell coordinates
        value: occupancy value (0-100, or -1 for unknown)
        """
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = value
            self.updated = True

    def get_cell(self, x, y):
        """
        Get occupancy value of a cell.
        """
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x]
        return -1  # Return unknown if outside grid

    def world_to_map(self, wx, wy):
        """
        Convert world coordinates to map coordinates.
        """
        mx = int((wx - self.origin[0]) / self.resolution)
        my = int((wy - self.origin[1]) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        """
        Convert map coordinates to world coordinates.
        """
        wx = mx * self.resolution + self.origin[0]
        wy = my * self.resolution + self.origin[1]
        return wx, wy

    def is_occupied(self, x, y, world_coords=True):
        """
        Check if the specified location is occupied.
        If world_coords is True, x and y are in world coordinates; otherwise they are in map coordinates.
        """
        if world_coords:
            mx, my = self.world_to_map(x, y)
        else:
            mx, my = x, y

        cell_value = self.get_cell(mx, my)
        return cell_value > 50  # Consider occupied if above threshold

    def to_message(self):
        """
        Convert the occupancy grid to a ROS OccupancyGrid message.
        """
        msg = OccupancyGrid()
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.resolution = self.resolution
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]

        # Flatten the grid for the ROS message
        msg.data = [
            self.grid[j][i] for j in range(self.height) for i in range(self.width)
        ]

        return msg


class SLAM:
    """
    SLAM component for Simultaneous Localization And Mapping.
    Processes sensor data to update the map and estimate the robot's position.
    """

    def __init__(self, occupancy_grid=None):
        self.map = occupancy_grid if occupancy_grid else OccupancyGridMap()
        self.robot_pose = [0, 0, 0]  # [x, y, theta]
        self.path = []  # History of robot poses
        self.position_uncertainty = 0.0  # Uncertainty in the robot's position

        # Publishers
        self.map_publisher = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        self.pose_publisher = rospy.Publisher("robot_pose", PoseStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber("imu_data", String, self.process_imu_data)
        rospy.Subscriber("left_encoder", Float32, self.process_left_encoder)
        rospy.Subscriber("right_encoder", Float32, self.process_right_encoder)
        rospy.Subscriber("object_detection", String, self.process_object_detection)
        rospy.Subscriber(
            "manipulation_status", String, self.process_manipulation_status
        )

    def process_imu_data(self, imu_msg):
        """
        Process IMU data to estimate orientation.
        """
        imu_data = imu_msg.data
        try:
            # Parse the IMU data
            orientation = float(imu_data.split(":")[1])

            # Add noise and update the robot's orientation
            noise = random.uniform(-0.01, 0.01)
            self.robot_pose[2] = orientation + noise

        except (ValueError, IndexError):
            rospy.logwarn("Invalid IMU data format")

    def process_left_encoder(self, encoder_msg):
        """
        Process left wheel encoder data.
        """
        encoder_ticks = encoder_msg.data
        rospy.loginfo(f"Received left encoder data: {encoder_ticks} ticks")

    def process_right_encoder(self, encoder_msg):
        """
        Process right wheel encoder data.
        """
        encoder_ticks = encoder_msg.data
        rospy.loginfo(f"Received right encoder data: {encoder_ticks} ticks")

    def process_object_detection(self, detection_msg):
        """
        Process object detection data from the perception system.
        """
        detection_data = detection_msg.data
        rospy.loginfo(f"Received object detection data: {detection_data}")

        # Simplified mapping update using object detection data
        try:
            parts = detection_data.split(",")
            if len(parts) >= 3:
                obj_type = parts[0]
                obj_x = float(parts[1])
                obj_y = float(parts[2])

                # Update map: mark cell as occupied
                mx, my = self.map.world_to_map(obj_x, obj_y)
                self.map.update_cell(mx, my, 100)  # 100 = occupied

        except (ValueError, IndexError):
            rospy.logwarn("Invalid object detection data format")

    def process_manipulation_status(self, status_msg):
        """
        Process status information from the manipulation subsystem.
        """
        manipulation_status = status_msg.data
        rospy.loginfo(f"Received manipulation status: {manipulation_status}")
        # Adjust navigation actions based on manipulation status (e.g., slow down if manipulation is active)

    def update_position(self, delta_x, delta_y, delta_theta):
        """
        Update the robot's position based on odometry.
        """
        self.robot_pose[0] += delta_x
        self.robot_pose[1] += delta_y
        self.robot_pose[2] += delta_theta

        # Append the updated pose to the path
        self.path.append(self.robot_pose.copy())

        # Publish the updated pose
        self.publish_pose()

    def update_map(self):
        """
        Update the occupancy grid map based on sensor data.
        """
        # In a real SLAM implementation, sensor data would update the map
        # For simulation, add a random obstacle occasionally
        if random.random() < 0.05:  # 5% chance to add an obstacle
            x = random.randint(0, self.map.width - 1)
            y = random.randint(0, self.map.height - 1)
            self.map.update_cell(x, y, 100)  # 100 = occupied

        # Publish the updated map if modified
        if self.map.updated:
            self.publish_map()
            self.map.updated = False

    def publish_map(self):
        """
        Publish the occupancy grid map.
        """
        map_msg = self.map.to_message()
        self.map_publisher.publish(map_msg)

    def publish_pose(self):
        """
        Publish the current pose of the robot.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.robot_pose[0]
        pose_msg.pose.position.y = self.robot_pose[1]
        pose_msg.pose.position.z = 0.0

        # Convert theta to a quaternion (simplified)
        pose_msg.pose.orientation.z = math.sin(self.robot_pose[2] / 2)
        pose_msg.pose.orientation.w = math.cos(self.robot_pose[2] / 2)

        self.pose_publisher.publish(pose_msg)

    def get_current_pose(self):
        """
        Return the current pose of the robot.
        """
        return self.robot_pose

    def get_uncertainty(self):
        """
        Return the current position uncertainty.
        """
        return self.position_uncertainty

    def is_path_clear(self, start, end):
        """
        Check if there is an obstacle between the start and end points using a simple ray-casting algorithm.
        """
        start_x, start_y = self.map.world_to_map(start[0], start[1])
        end_x, end_y = self.map.world_to_map(end[0], end[1])

        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        sx = 1 if start_x < end_x else -1
        sy = 1 if start_y < end_y else -1
        err = dx - dy

        x, y = start_x, start_y
        while x != end_x or y != end_y:
            if self.map.is_occupied(x, y, False):
                return False  # Path is blocked

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True  # Path is clear


class TrajectoryController:
    """
    The TrajectoryController generates and controls the robot's trajectory.
    It receives goals from SLAM and sends appropriate motor commands to the left and right motor controllers.
    """

    def __init__(self, slam=None):
        self.slam = slam
        self.current_trajectory = []
        self.goal = None
        self.reached_goal = True
        self.pid_params = {"Kp": 0.5, "Ki": 0.1, "Kd": 0.2}  # PID control parameters

        # Publishers
        self.left_motor_publisher = rospy.Publisher(
            "left_motor_cmd", Float32, queue_size=10
        )
        self.right_motor_publisher = rospy.Publisher(
            "right_motor_cmd", Float32, queue_size=10
        )
        self.trajectory_publisher = rospy.Publisher("trajectory", Path, queue_size=10)

        # Subscriber
        rospy.Subscriber("goal", PoseStamped, self.set_goal)

    def set_goal(self, goal_msg):
        """
        Set a new navigation goal.
        """
        self.goal = [goal_msg.pose.position.x, goal_msg.pose.position.y]
        self.reached_goal = False
        rospy.loginfo(f"New goal set: {self.goal}")

        # Plan trajectory toward the goal
        self.plan_trajectory()

    def plan_trajectory(self):
        """
        Plan a path from the current position to the goal.
        """
        if not self.slam or not self.goal:
            return

        current_pose = self.slam.get_current_pose()
        start = [current_pose[0], current_pose[1]]

        # Check if a direct path is available
        if self.slam.is_path_clear(start, self.goal):
            # Use a simple straight-line trajectory
            self.current_trajectory = [start, self.goal]
        else:
            # In a real system, use A* or similar algorithm. For simulation, create a simple midpoint.
            midpoint = [(start[0] + self.goal[0]) / 2, (start[1] + self.goal[1]) / 2]
            self.current_trajectory = [start, midpoint, self.goal]

        # Publish the planned trajectory
        self.publish_trajectory()

    def publish_trajectory(self):
        """
        Publish the planned trajectory.
        """
        if not self.current_trajectory:
            return

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for point in self.current_trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_msg.poses.append(pose)

        self.trajectory_publisher.publish(path_msg)

    def update_control(self):
        """
        Update control signals based on current position and the planned trajectory using a simple PID controller.
        """
        if not self.slam or not self.goal or self.reached_goal:
            return

        current_pose = self.slam.get_current_pose()
        current_pos = [current_pose[0], current_pose[1]]
        current_theta = current_pose[2]

        dx = self.goal[0] - current_pos[0]
        dy = self.goal[1] - current_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)

        # Check if goal is reached
        if distance < 0.1:  # Threshold in meters
            self.reached_goal = True
            self.stop_motors()
            rospy.loginfo("Goal reached")
            return

        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - current_theta)
        Kp = self.pid_params["Kp"]
        base_speed = 0.5  # m/s

        left_speed = base_speed - Kp * theta_error
        right_speed = base_speed + Kp * theta_error

        self.left_motor_publisher.publish(left_speed)
        self.right_motor_publisher.publish(right_speed)

    def normalize_angle(self, angle):
        """
        Normalize angle to be within -pi to pi.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_motors(self):
        """
        Stop both motors.
        """
        self.left_motor_publisher.publish(0.0)
        self.right_motor_publisher.publish(0.0)


class NavigationSystem:
    """
    NavigationSystem is the main interface for the navigation subsystem.
    It integrates the SLAM and TrajectoryController components.
    """

    _instance = None

    def __new__(cls, tiago_platform=None):
        """
        Create a singleton instance of NavigationSystem.
        """
        if cls._instance is None:
            cls._instance = super(NavigationSystem, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, tiago_platform=None):
        """
        Initialize the navigation system with its components.
        """
        if not hasattr(self, "_initialized") or not self._initialized:
            self.tiago = tiago_platform

            # Initialize components
            self.occupancy_grid = OccupancyGridMap()
            self.slam = SLAM(self.occupancy_grid)
            self.trajectory_controller = TrajectoryController(self.slam)

            self._initialized = True

    def navigate_to(self, goal_position):
        """
        Navigate to the specified goal position.
        """
        if not isinstance(goal_position, list) or len(goal_position) < 2:
            rospy.logwarn("Invalid goal position format")
            return False

        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_position[0]
        goal_msg.pose.position.y = goal_position[1]

        self.trajectory_controller.set_goal(goal_msg)
        return True

    def update(self):
        """
        Update the navigation system.
        """
        if self.slam:
            self.slam.update_map()
        if self.trajectory_controller:
            self.trajectory_controller.update_control()

    def stop(self):
        """
        Stop the robot's movement.
        """
        if self.trajectory_controller:
            self.trajectory_controller.stop_motors()
