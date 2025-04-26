#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Point


class TableAnalyzer:
    """
    TableAnalyzer analyzes the table layout and identifies objects on the table.
    
    This class is responsible for:
    - Processing perception data to extract object information
    - Calculating occupied areas based on detected objects
    - Identifying free areas where new objects can be placed
    - Providing comprehensive table layout information for placement planning
    
    It receives information from the PerceptionSystem's ObjectRecognition component
    and sends table layout information to the PlacementPlanner.
    
    Attributes:
        table_objects (list): List of objects detected on the table
        table_dimensions (dict): Dimensions of the table (width, length in meters)
        occupied_areas (list): Areas on the table occupied by objects
    """

    def __init__(self):
        """
        Initialize the TableAnalyzer with empty object list and default table dimensions.
        """
        self.table_objects = []
        self.table_dimensions = {"width": 0.8, "length": 1.2}
        self.occupied_areas = []

    def analyze_table(self, perception_data):
        """
        Analyze perception data to determine the current table layout.
        
        This method processes raw perception data to build a structured 
        representation of the table layout including objects, occupied areas,
        and available free spaces for potential placement.
        
        Args:
            perception_data (list): List of detected objects from the perception system
            
        Returns:
            dict: Table layout information containing:
                - dimensions: Table size (width, length)
                - objects: List of detected objects
                - occupied_areas: Areas occupied by objects
                - free_areas: Available areas for placing new objects
        """
        # Process perception data to identify objects and their positions
        self.table_objects = self._extract_objects(perception_data)
        self.occupied_areas = self._calculate_occupied_areas()

        table_layout = {
            "dimensions": self.table_dimensions,
            "objects": self.table_objects,
            "occupied_areas": self.occupied_areas,
            "free_areas": self._identify_free_areas(),
        }

        return table_layout

    def _extract_objects(self, perception_data):
        """
        Extract object information from perception data.
        
        Converts raw sensor data into structured object representations
        with position and dimension information.
        
        Args:
            perception_data (list): Raw perception data from sensors
            
        Returns:
            list: Structured list of objects with their properties
        """
        # In a real system, this would process actual perception data
        # For simulation, we generate objects based on provided perception data
        objects = []
        if perception_data:
            for obj in perception_data:
                objects.append(
                    {
                        "type": obj.get("type", "unknown"),
                        "position": obj.get("position", [0, 0]),
                        "dimensions": obj.get("dimensions", [0.1, 0.1]),
                    }
                )
        return objects

    def _calculate_occupied_areas(self):
        """
        Calculate areas occupied by objects on the table.
        
        Converts object positions and dimensions into rectangular occupied areas
        defined by their minimum and maximum x,y coordinates.
        
        Returns:
            list: List of dictionaries representing occupied rectangular areas
        """
        occupied = []
        for obj in self.table_objects:
            x, y = obj["position"]
            width, length = obj["dimensions"]
            occupied.append(
                {
                    "x_min": x - width / 2,
                    "x_max": x + width / 2,
                    "y_min": y - length / 2,
                    "y_max": y + length / 2,
                }
            )
        return occupied

    def _identify_free_areas(self):
        """
        Identify free areas on the table where objects can be placed.
        
        Uses a grid-based approach to find positions that do not overlap
        with any occupied areas.
        
        Returns:
            list: List of [x, y] coordinates representing free spots
        """
        # Grid-based approach to identify free areas
        grid_size = 0.1  # 10cm grid resolution
        table_width = self.table_dimensions["width"]
        table_length = self.table_dimensions["length"]

        free_spots = []
        for x in range(int(table_width / grid_size)):
            for y in range(int(table_length / grid_size)):
                x_pos = x * grid_size
                y_pos = y * grid_size

                is_free = True
                for area in self.occupied_areas:
                    if (
                        area["x_min"] <= x_pos <= area["x_max"]
                        and area["y_min"] <= y_pos <= area["y_max"]
                    ):
                        is_free = False
                        break

                if is_free:
                    free_spots.append([x_pos, y_pos])

        return free_spots


class PlacementPlanner:
    """
    PlacementPlanner determines optimal placement spots for dishes.
    
    This class is responsible for:
    - Finding optimal placement locations on the table
    - Considering table layout and dish dimensions
    - Implementing placement strategies (e.g., closest to center)
    - Handling recalculation requests when optimal spots are unavailable
    
    It receives table layout from TableAnalyzer, placement confirmation from OrderVerifier,
    and recalculation requests from ReasoningController.
    It sends optimal spot information to CollisionChecker.
    
    Attributes:
        current_layout (dict): Current table layout information
        dish_dimensions (dict): Dimensions of the dish to be placed
        optimal_spot (list): Calculated optimal placement coordinates
        placement_confirmed (bool): Flag indicating successful placement
    """

    def __init__(self):
        """
        Initialize the PlacementPlanner with default values.
        """
        self.current_layout = None
        self.dish_dimensions = {"width": 0.2, "length": 0.2}
        self.optimal_spot = None
        self.placement_confirmed = False

    def plan_placement(self, table_layout, recalculate=False):
        """
        Plan optimal placement for a dish based on table layout.
        
        Identifies the best location to place a dish considering the current
        table layout and available free spaces. Can recalculate if previous
        placement was unsuccessful.
        
        Args:
            table_layout (dict): Current table layout information
            recalculate (bool): Flag to force recalculation of optimal spot
            
        Returns:
            list: [x, y] coordinates of the optimal placement spot,
                 or None if no suitable spot found
        """
        self.current_layout = table_layout

        if recalculate or self.optimal_spot is None:
            # Find the best spot using placement strategy
            free_areas = table_layout.get("free_areas", [])
            if free_areas:
                # Use placement strategy to find optimal spot
                self.optimal_spot = self._find_optimal_spot(free_areas)
            else:
                # No free areas available
                self.optimal_spot = None

        return self.optimal_spot

    def _find_optimal_spot(self, free_areas):
        """
        Find the optimal spot among free areas.
        
        Uses a strategy to find the spot closest to the table center
        with enough space for the dish, avoiding collisions.
        
        Args:
            free_areas (list): List of available placement spots
            
        Returns:
            list: [x, y] coordinates of optimal placement spot,
                 or None if no suitable spot found
        """
        table_width = self.current_layout["dimensions"]["width"]
        table_length = self.current_layout["dimensions"]["length"]
        table_center = [table_width / 2, table_length / 2]

        best_spot = None
        min_distance = float("inf")

        # Filter spots with enough space for the dish
        dish_width = self.dish_dimensions["width"]
        dish_length = self.dish_dimensions["length"]

        for spot in free_areas:
            # Check if there's enough space for the dish
            has_space = True
            for area in self.current_layout["occupied_areas"]:
                if (
                    abs(spot[0] - area["x_min"]) < dish_width / 2
                    or abs(spot[0] - area["x_max"]) < dish_width / 2
                    or abs(spot[1] - area["y_min"]) < dish_length / 2
                    or abs(spot[1] - area["y_max"]) < dish_length / 2
                ):
                    has_space = False
                    break

            if not has_space:
                continue

            # Calculate distance to center
            distance = math.sqrt(
                (spot[0] - table_center[0]) ** 2 + (spot[1] - table_center[1]) ** 2
            )

            if distance < min_distance:
                min_distance = distance
                best_spot = spot

        return best_spot

    def receive_placement_confirmation(self, confirmed):
        """
        Receive confirmation of successful placement.
        
        Updates the placement status based on feedback from manipulation system.
        
        Args:
            confirmed (bool): Whether placement was successful
        """
        self.placement_confirmed = confirmed


class CollisionChecker:
    """
    CollisionChecker assesses collision risk for planned placement.
    
    This class is responsible for:
    - Evaluating safety of planned placement locations
    - Calculating collision risk based on proximity to other objects
    - Determining if a placement is feasible based on risk assessment
    - Providing detailed risk information for decision making
    
    It receives optimal spot information from PlacementPlanner
    and sends collision risk assessment to ReasoningController.
    
    Attributes:
        risk_threshold (float): Maximum acceptable risk level (0-1)
    """

    def __init__(self):
        """
        Initialize the CollisionChecker with default risk threshold.
        """
        self.risk_threshold = 0.3  # 30% is the maximum acceptable risk

    def check_collision(self, optimal_spot, table_layout, dish_dimensions):
        """
        Check for potential collisions at the optimal spot.
        
        Evaluates collision risk by calculating proximity to other objects
        and determining if the placement is feasible based on risk threshold.
        
        Args:
            optimal_spot (list): [x, y] coordinates of proposed placement
            table_layout (dict): Current table layout information
            dish_dimensions (dict): Size of dish to be placed
            
        Returns:
            dict: Risk assessment containing:
                - risk: Numerical risk value (0-1)
                - feasible: Whether placement is considered safe
                - message: Human-readable risk description
                - min_distance: Minimum distance to nearest object
        """
        if not optimal_spot:
            return {"risk": 1.0, "feasible": False, "message": "No optimal spot found"}

        # Calculate minimum distances to other objects
        min_distance = float("inf")
        for obj in table_layout.get("objects", []):
            obj_x, obj_y = obj["position"]
            distance = math.sqrt(
                (optimal_spot[0] - obj_x) ** 2 + (optimal_spot[1] - obj_y) ** 2
            )
            min_distance = min(min_distance, distance)

        # Calculate risk based on minimum distance
        if min_distance == float("inf"):
            risk = 0.0
        else:
            # Risk increases as distance decreases
            dish_size = max(dish_dimensions["width"], dish_dimensions["length"])
            risk = max(0, 1.0 - (min_distance / (dish_size * 2)))

        return {
            "risk": risk,
            "feasible": risk < self.risk_threshold,
            "message": self._generate_risk_message(risk),
            "min_distance": min_distance,
        }

    def _generate_risk_message(self, risk):
        """
        Generate a descriptive message about the collision risk.
        
        Converts numerical risk value to human-readable description.
        
        Args:
            risk (float): Numerical risk value (0-1)
            
        Returns:
            str: Human-readable risk description
        """
        if risk < 0.1:
            return "Very low risk of collision"
        elif risk < 0.3:
            return "Low risk of collision"
        elif risk < 0.6:
            return "Moderate risk of collision"
        elif risk < 0.9:
            return "High risk of collision"
        else:
            return "Extremely high risk of collision"


class ReasoningController:
    """
    ReasoningController coordinates the entire reasoning process.
    
    This class is responsible for:
    - Orchestrating the reasoning pipeline for dish placement
    - Managing the interaction between component systems
    - Implementing retry logic for failed placement attempts
    - Publishing final placement decisions to the manipulation system
    
    It receives collision risk information from CollisionChecker
    and sends recalculation requests to PlacementPlanner and
    target dish position to ManipulationSupervisor.
    
    Attributes:
        tiago (TIAGo): Reference to the TIAGo robot platform
        table_analyzer (TableAnalyzer): Component for analyzing table layout
        placement_planner (PlacementPlanner): Component for planning placement
        collision_checker (CollisionChecker): Component for checking collision risks
        max_attempts (int): Maximum number of placement attempts before giving up
    """

    def __init__(self, tiago_platform=None):
        """
        Initialize the ReasoningController with its component systems.
        
        Args:
            tiago_platform (TIAGo): Reference to the TIAGo robot platform
        """
        self.tiago = tiago_platform
        self.table_analyzer = TableAnalyzer()
        self.placement_planner = PlacementPlanner()
        self.collision_checker = CollisionChecker()
        self.max_attempts = 3

        # For communication with ManipulationSystem
        if self.tiago:
            self.target_position_publisher = rospy.Publisher(
                "target_dish_position", Point, queue_size=10
            )

    def process_placement_request(self, perception_data, dish_dimensions=None):
        """
        Process a placement request using the reasoning components.
        
        Orchestrates the full reasoning pipeline:
        1. Analyze table layout
        2. Plan optimal placement
        3. Check collision risks
        4. Make final placement decision
        5. Publish target position
        
        Implements retry logic for up to max_attempts tries.
        
        Args:
            perception_data (list): Perception data from sensors
            dish_dimensions (dict, optional): Custom dish dimensions
            
        Returns:
            list: [x, y] coordinates of target position, or None if placement failed
        """
        if dish_dimensions:
            self.placement_planner.dish_dimensions = dish_dimensions

        attempts = 0
        target_position = None

        while attempts < self.max_attempts:
            # Step 1: Analyze table layout
            table_layout = self.table_analyzer.analyze_table(perception_data)

            # Step 2: Plan placement
            recalculate = attempts > 0
            optimal_spot = self.placement_planner.plan_placement(
                table_layout, recalculate
            )

            # Step 3: Check for collisions
            collision_risk = self.collision_checker.check_collision(
                optimal_spot, table_layout, self.placement_planner.dish_dimensions
            )

            # Step 4: Decide whether to proceed or recalculate
            if collision_risk["feasible"]:
                target_position = optimal_spot
                break

            # Increase attempts and try again with different parameters
            attempts += 1

        # Publish target position
        if self.tiago and target_position:
            self._publish_target_position(target_position)

        return target_position

    def _publish_target_position(self, position):
        """
        Publish target dish position for Manipulation System.
        
        Creates and publishes a Point message with the target position
        coordinates for the manipulation system to use.
        
        Args:
            position (list): [x, y] coordinates for dish placement
            
        Returns:
            bool: True if message was published successfully
        """
        point_msg = Point()
        point_msg.x = position[0]
        point_msg.y = position[1]
        point_msg.z = 0.7  # Assuming table height is 70 cm.

        self.target_position_publisher.publish(point_msg)

        return True


class ReasoningSystem:
    """
    ReasoningSystem is the main interface for the reasoning subsystem.
    
    This class serves as the primary entry point for other systems to interact
    with the reasoning components. It implements the singleton pattern to ensure
    only one reasoning system exists per robot.
    
    This class is responsible for:
    - Providing a unified interface to the reasoning capabilities
    - Ensuring single instance through the singleton pattern
    - Delegating reasoning requests to appropriate components
    
    Attributes:
        tiago (TIAGo): Reference to the TIAGo robot platform
        controller (ReasoningController): The core reasoning controller
    """

    _instance = None

    def __new__(cls, tiago_platform=None):
        """
        Create a singleton instance of ReasoningSystem.
        
        Ensures only one instance of ReasoningSystem exists per application.
        
        Args:
            tiago_platform (TIAGo): Reference to the TIAGo robot platform
            
        Returns:
            ReasoningSystem: The singleton instance
        """
        if cls._instance is None:
            cls._instance = super(ReasoningSystem, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, tiago_platform=None):
        """
        Initialize the reasoning system with its components.
        
        Only initializes on first instantiation due to singleton pattern.
        
        Args:
            tiago_platform (TIAGo): Reference to the TIAGo robot platform
        """
        if not hasattr(self, "_initialized") or not self._initialized:
            self.tiago = tiago_platform
            self.controller = ReasoningController(tiago_platform)
            self._initialized = True

    def reason_about_placement(self, perception_data, dish_dimensions=None):
        """
        Main method to reason about optimal placement of a dish.
        
        This is the primary entry point for other systems to request
        reasoning about dish placement.
        
        Args:
            perception_data (list): Perception data from sensors
            dish_dimensions (dict, optional): Custom dish dimensions
            
        Returns:
            list: [x, y] coordinates of target position, or None if placement failed
        """
        return self.controller.process_placement_request(
            perception_data, dish_dimensions
        )
