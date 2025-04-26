#!/usr/bin/env python3

import rospy
import random
import math
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class OccupancyGridMap:
    """
    OccupancyGridMap represents the environment as a 2D grid for navigation planning.
    
    This class handles:
    - Storing and managing the environment map as a grid of cells
    - Converting between world coordinates and map coordinates
    - Detecting obstacles and free spaces
    - Converting the map to ROS message format
    
    Each cell in the grid contains a value:
    - 0: Free space (no obstacle)
    - 100: Occupied (obstacle present)
    - -1: Unknown (not yet mapped)
    """

    def __init__(self, width=100, height=100, resolution=0.1):
        """
        Initialize the occupancy grid map.
        
        Args:
            width (int): Width of the grid in cells
            height (int): Height of the grid in cells
            resolution (float): Size of each cell in meters (e.g., 0.1m = 10cm per cell)
        """
        self.width = width  # Width of the grid (number of cells)
        self.height = height  # Height of the grid (number of cells)
        self.resolution = resolution  # Meters per cell
        self.origin = [0, 0]  # Origin position (meters) [x, y]
        # Initialize grid with all cells as free (0)
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        self.updated = False  # Flag to track if the map has been modified

    def update_cell(self, x, y, value):
        """
        Update the occupancy value of a specific cell in the grid.
        
        Args:
            x (int): X-coordinate in the grid
            y (int): Y-coordinate in the grid
            value (int): Occupancy value:
                         0 = free space
                         100 = occupied
                         -1 = unknown
                         
        Returns:
            bool: True if the cell was updated, False if coordinates are out of bounds
        """
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = value
            self.updated = True
            return True
        return False

    def get_cell(self, x, y):
        """
        Get the occupancy value of a specific cell in the grid.
        
        Args:
            x (int): X-coordinate in the grid
            y (int): Y-coordinate in the grid
            
        Returns:
            int: The occupancy value of the cell (0, 100, or -1)
                 Returns -1 (unknown) if coordinates are out of bounds
        """
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x]
        return -1  # Return unknown if outside grid

    def world_to_map(self, wx, wy):
        """
        Convert world coordinates (meters) to map coordinates (grid cells).
        
        Args:
            wx (float): X-coordinate in world frame (meters)
            wy (float): Y-coordinate in world frame (meters)
            
        Returns:
            tuple: (x, y) coordinates in the grid (integers)
        """
        mx = int((wx - self.origin[0]) / self.resolution)
        my = int((wy - self.origin[1]) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        """
        Convert map coordinates (grid cells) to world coordinates (meters).
        
        Args:
            mx (int): X-coordinate in the grid
            my (int): Y-coordinate in the grid
            
        Returns:
            tuple: (x, y) coordinates in the world frame (meters)
        """
        wx = mx * self.resolution + self.origin[0]
        wy = my * self.resolution + self.origin[1]
        return wx, wy

    def is_occupied(self, x, y, world_coords=True):
        """
        Check if the specified location contains an obstacle.
        
        Args:
            x: X-coordinate
            y: Y-coordinate
            world_coords (bool): If True, x and y are in world coordinates (meters)
                                If False, x and y are in map coordinates (grid cells)
                                
        Returns:
            bool: True if the cell is occupied, False otherwise
        """
        if world_coords:
            mx, my = self.world_to_map(x, y)
        else:
            mx, my = x, y

        cell_value = self.get_cell(mx, my)
        return cell_value > 50  # Consider occupied if above threshold

    def to_message(self):
        """
        Convert the occupancy grid to a ROS OccupancyGrid message for visualization and sharing.
        
        Returns:
            OccupancyGrid: ROS message containing the grid data
        """
        msg = OccupancyGrid()
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.resolution = self.resolution
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]

        # Flatten the grid for the ROS message (row-major order)
        msg.data = [
            self.grid[j][i] for j in range(self.height) for i in range(self.width)
        ]

        return msg


class SLAM:
    """
    SLAM (Simultaneous Localization And Mapping) component for navigation.
    
    This class handles:
    - Processing sensor data to build and update the map of the environment
    - Tracking the robot's position and orientation
    - Maintaining the history of the robot's path
    - Publishing map and pose data to ROS topics
    - Providing mapping and localization services to the navigation system
    
    In a real system, this would implement full SLAM algorithms.
    For simulation, we use a simplified approach.
    """

    def __init__(self, occupancy_grid=None):
        """
        Initialize the SLAM system.
        
        Args:
            occupancy_grid (OccupancyGridMap): Pre-existing map, or creates a new one if None
        """
        # Initialize map and localization state
        self.map = occupancy_grid if occupancy_grid else OccupancyGridMap()
        self.robot_pose = [0, 0, 0]  # [x, y, theta] in meters and radians
        self.path = []  # History of robot poses for path tracking
        self.position_uncertainty = 0.0  # Uncertainty in the robot's position estimate

        # Publishers for map and pose data
        self.map_publisher = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        self.pose_publisher = rospy.Publisher("robot_pose", PoseStamped, queue_size=10)

        # Subscribers for sensor data
        rospy.Subscriber("imu_data", String, self.process_imu_data)
        rospy.Subscriber("left_encoder", Float32, self.process_left_encoder)
        rospy.Subscriber("right_encoder", Float32, self.process_right_encoder)
        rospy.Subscriber("object_detection", String, self.process_object_detection)
        rospy.Subscriber(
            "manipulation_status", String, self.process_manipulation_status
        )

    def process_imu_data(self, imu_msg):
        """
        Process inertial measurement unit data to update orientation estimates.
        
        In a real system, this would process actual IMU measurements.
        For simulation, we parse a simplified message format.
        
        Args:
            imu_msg (String): IMU data message with format "orientation:value"
        """
        imu_data = imu_msg.data
        try:
            # Parse the IMU data
            orientation = float(imu_data.split(":")[1])

            # Add noise to simulate sensor inaccuracy
            noise = random.uniform(-0.01, 0.01)
            self.robot_pose[2] = orientation + noise

        except (ValueError, IndexError):
            rospy.logwarn("Invalid IMU data format")

    def process_left_encoder(self, encoder_msg):
        """
        Process left wheel encoder data for odometry calculations.
        
        In a real system, this would use wheel encoder ticks to calculate displacement.
        For simulation, we just log the data.
        
        Args:
            encoder_msg (Float32): Left wheel encoder ticks
        """
        encoder_ticks = encoder_msg.data

    def process_right_encoder(self, encoder_msg):
        """
        Process right wheel encoder data for odometry calculations.
        
        In a real system, this would use wheel encoder ticks to calculate displacement.
        For simulation, we just log the data.
        
        Args:
            encoder_msg (Float32): Right wheel encoder ticks
        """
        encoder_ticks = encoder_msg.data

    def process_object_detection(self, detection_msg):
        """
        Process object detection data from the perception system to update the map.
        
        Args:
            detection_msg (String): Object detection data with format "type,x,y,..."
        """
        detection_data = detection_msg.data

        # Parse object detection data and update map
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
        
        This allows navigation to be aware of and respond to manipulation activities.
        
        Args:
            status_msg (String): Current manipulation status
        """
        manipulation_status = status_msg.data
        # Adjust navigation behavior based on manipulation status
        # For example, decrease speed during active manipulation

    def update_position(self, delta_x, delta_y, delta_theta):
        """
        Update the robot's position based on odometry or motion commands.
        
        Args:
            delta_x (float): Change in x position (meters)
            delta_y (float): Change in y position (meters)
            delta_theta (float): Change in orientation (radians)
        """
        # Update the robot's pose
        self.robot_pose[0] += delta_x
        self.robot_pose[1] += delta_y
        self.robot_pose[2] += delta_theta

        # Append the updated pose to the path history
        self.path.append(self.robot_pose.copy())

        # Publish the updated pose to ROS
        self.publish_pose()

    def update_map(self):
        """
        Update the occupancy grid map based on sensor data.
        
        In a real system, this would use sensor observations to update the map.
        For simulation, we occasionally add random obstacles.
        """
        # Simulate map updates by occasionally adding obstacles
        if random.random() < 0.05:  # 5% chance to add an obstacle
            x = random.randint(0, self.map.width - 1)
            y = random.randint(0, self.map.height - 1)
            self.map.update_cell(x, y, 100)  # 100 = occupied

        # Publish the updated map if changes were made
        if self.map.updated:
            self.publish_map()
            self.map.updated = False

    def publish_map(self):
        """
        Publish the current occupancy grid map to the 'map' ROS topic.
        """
        map_msg = self.map.to_message()
        self.map_publisher.publish(map_msg)

    def publish_pose(self):
        """
        Publish the current pose of the robot to the 'robot_pose' ROS topic.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        
        # Set position data
        pose_msg.pose.position.x = self.robot_pose[0]
        pose_msg.pose.position.y = self.robot_pose[1]
        pose_msg.pose.position.z = 0.0

        # Convert theta to quaternion representation (simplified)
        # In a real system, would use proper quaternion conversion
        pose_msg.pose.orientation.z = math.sin(self.robot_pose[2] / 2)
        pose_msg.pose.orientation.w = math.cos(self.robot_pose[2] / 2)

        self.pose_publisher.publish(pose_msg)

    def get_current_pose(self):
        """
        Return the current pose (position and orientation) of the robot.
        
        Returns:
            list: [x, y, theta] - position (meters) and orientation (radians)
        """
        return self.robot_pose

    def get_uncertainty(self):
        """
        Return the current position uncertainty estimate.
        
        In a real system, this would be derived from sensor fusion algorithms.
        
        Returns:
            float: Position uncertainty metric
        """
        return self.position_uncertainty

    def is_path_clear(self, start, end):
        """
        Check if there is an obstacle between the start and end points.
        
        Uses a simplified ray-casting algorithm to check for obstacles along a straight line.
        
        Args:
            start (list): [x, y] start position in world coordinates
            end (list): [x, y] end position in world coordinates
            
        Returns:
            bool: True if path is clear, False if an obstacle is detected
        """
        # Convert world coordinates to map coordinates
        start_x, start_y = self.map.world_to_map(start[0], start[1])
        end_x, end_y = self.map.world_to_map(end[0], end[1])

        # Bresenham's line algorithm for ray casting
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        sx = 1 if start_x < end_x else -1
        sy = 1 if start_y < end_y else -1
        err = dx - dy

        x, y = start_x, start_y
        while x != end_x or y != end_y:
            if self.map.is_occupied(x, y, False):
                return False  # Path is blocked by obstacle

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True  # Path is clear of obstacles


class TrajectoryController:
    """
    The TrajectoryController plans and executes robot trajectories for navigation.
    
    This class handles:
    - Receiving navigation goals
    - Planning paths avoiding obstacles
    - Sending control commands to the motors
    - Monitoring progress toward the goal
    - Publishing trajectory information for visualization
    
    Uses a simplified path planning approach with direct paths or via points.
    In a real system, this would implement more sophisticated path planning algorithms.
    """

    def __init__(self, slam=None):
        """
        Initialize the trajectory controller.
        
        Args:
            slam (SLAM): Reference to the SLAM system for position and map data
        """
        self.slam = slam  # Reference to SLAM system for position and map data
        self.current_trajectory = []  # List of waypoints in the current path
        self.goal = None  # Current navigation goal
        self.reached_goal = True  # Flag indicating if goal has been reached
        
        # Control parameters
        self.pid_params = {"Kp": 0.5, "Ki": 0.1, "Kd": 0.2}  # PID control parameters

        # Publishers for motor commands and trajectory visualization
        self.left_motor_publisher = rospy.Publisher(
            "left_motor_cmd", Float32, queue_size=10
        )
        self.right_motor_publisher = rospy.Publisher(
            "right_motor_cmd", Float32, queue_size=10
        )
        self.trajectory_publisher = rospy.Publisher("trajectory", Path, queue_size=10)

        # Subscriber for navigation goals
        rospy.Subscriber("goal", PoseStamped, self.set_goal)

    def set_goal(self, goal_msg):
        """
        Set a new navigation goal from a ROS message.
        
        Begins the planning and execution process for reaching the goal.
        
        Args:
            goal_msg (PoseStamped): Goal position message
        """
        # Extract goal position from message
        self.goal = [goal_msg.pose.position.x, goal_msg.pose.position.y]
        self.reached_goal = False

        # Plan a trajectory to the goal
        self.plan_trajectory()

    def plan_trajectory(self):
        """
        Plan a path from the current position to the goal.
        
        In a real system, this would use A* or RRT algorithms.
        For simulation, we use a simple direct path or via point approach.
        """
        if not self.slam or not self.goal:
            rospy.logwarn("Cannot plan trajectory: SLAM or goal not defined")
            return

        current_pose = self.slam.get_current_pose()
        start = [current_pose[0], current_pose[1]]

        # Check if a direct path is available without obstacles
        if self.slam.is_path_clear(start, self.goal):
            # Use a simple straight-line trajectory
            self.current_trajectory = [start, self.goal]
        else:
            # Simple obstacle avoidance using a midpoint
            midpoint = [(start[0] + self.goal[0]) / 2, (start[1] + self.goal[1]) / 2]
            
            # Offset the midpoint perpendicular to the direct path
            dx = self.goal[0] - start[0]
            dy = self.goal[1] - start[1]
            length = math.sqrt(dx*dx + dy*dy)
            
            if length > 0:
                # Create perpendicular offset
                offset = 0.5  # meters
                midpoint[0] += (-dy/length) * offset
                midpoint[1] += (dx/length) * offset
            
            self.current_trajectory = [start, midpoint, self.goal]

        # Publish the planned trajectory for visualization
        self.publish_trajectory()

    def publish_trajectory(self):
        """
        Publish the planned trajectory as a ROS Path message for visualization.
        """
        if not self.current_trajectory:
            return

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        # Add each waypoint to the path message
        for point in self.current_trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_msg.poses.append(pose)

        self.trajectory_publisher.publish(path_msg)

    def update_control(self):
        """
        Update motor control signals based on current position and planned trajectory.
        
        Uses a simple proportional controller to adjust heading toward the goal.
        """
        if not self.slam or not self.goal or self.reached_goal:
            return

        # Get current position from SLAM
        current_pose = self.slam.get_current_pose()
        current_pos = [current_pose[0], current_pose[1]]
        current_theta = current_pose[2]

        # Calculate vector to goal
        dx = self.goal[0] - current_pos[0]
        dy = self.goal[1] - current_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)

        # Check if goal is reached
        if distance < 0.1:  # Threshold in meters
            self.reached_goal = True
            self.stop_motors()
            return

        # Calculate desired heading to goal
        desired_theta = math.atan2(dy, dx)
        
        # Calculate heading error (difference between current and desired)
        theta_error = self.normalize_angle(desired_theta - current_theta)
        
        # Apply proportional control
        Kp = self.pid_params["Kp"]
        base_speed = 0.5  # m/s

        # Calculate differential drive motor speeds
        left_speed = base_speed - Kp * theta_error
        right_speed = base_speed + Kp * theta_error

        # Send commands to motors
        self.left_motor_publisher.publish(left_speed)
        self.right_motor_publisher.publish(right_speed)

    def normalize_angle(self, angle):
        """
        Normalize angle to be within -pi to pi.
        
        Args:
            angle (float): Angle in radians
            
        Returns:
            float: Normalized angle in radians (-π to π)
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_motors(self):
        """
        Stop both motors by sending zero velocity commands.
        """
        self.left_motor_publisher.publish(0.0)
        self.right_motor_publisher.publish(0.0)


class NavigationSystem:
    """
    NavigationSystem is the main interface for TIAGo's navigation subsystem.
    
    This class integrates:
    - SLAM for mapping and localization
    - TrajectoryController for path planning and execution
    
    It provides a simplified interface for the rest of the robot system to request
    navigation tasks without dealing with the underlying complexity.
    
    Uses the singleton pattern to ensure only one instance exists.
    """

    _instance = None

    def __new__(cls, tiago_platform=None):
        """
        Create a singleton instance of NavigationSystem.
        
        Args:
            tiago_platform: Reference to the TIAGo robot instance
            
        Returns:
            NavigationSystem: The singleton instance
        """
        if cls._instance is None:
            cls._instance = super(NavigationSystem, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, tiago_platform=None):
        """
        Initialize the navigation system with its components.
        
        Only initializes the first time due to singleton pattern.
        
        Args:
            tiago_platform: Reference to the TIAGo robot instance
        """
        if not hasattr(self, "_initialized") or not self._initialized:
            self.tiago = tiago_platform

            # Initialize components in dependency order
            self.occupancy_grid = OccupancyGridMap()
            self.slam = SLAM(self.occupancy_grid)
            self.trajectory_controller = TrajectoryController(self.slam)

            self._initialized = True

    def navigate_to(self, goal_position):
        """
        Navigate to the specified goal position.
        
        This is the main entry point for other systems to request navigation.
        
        Args:
            goal_position (list): [x, y] coordinates in world frame
            
        Returns:
            bool: True if goal was set successfully, False otherwise
        """
        if not isinstance(goal_position, list) or len(goal_position) < 2:
            rospy.logwarn("Invalid goal position format")
            return False

        # Create a goal message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_position[0]
        goal_msg.pose.position.y = goal_position[1]

        # Set goal through the trajectory controller
        self.trajectory_controller.set_goal(goal_msg)
        return True

    def update(self):
        """
        Update the navigation system.
        
        Should be called periodically (e.g., in the main loop) to:
        - Update the map
        - Update control signals
        - Process sensor data
        """
        if self.slam:
            self.slam.update_map()
        if self.trajectory_controller:
            self.trajectory_controller.update_control()

    def stop(self):
        """
        Stop the robot's movement immediately.
        
        Used for emergency stops or when navigation should be interrupted.
        """
        if self.trajectory_controller:
            self.trajectory_controller.stop_motors()

    def is_goal_reached(self):
        """
        Check if the current navigation goal has been reached.
        
        Returns:
            bool: True if goal reached or no active goal, False otherwise
        """
        if not self.trajectory_controller:
            return True
        return self.trajectory_controller.reached_goal
