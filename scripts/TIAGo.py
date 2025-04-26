#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

try:
    from scripts.PerceptionSystem import PerceptionSystem
    from scripts.OrderVerificationSystem import OrderVerificationSystem
    from scripts.NavigationSystem import NavigationSystem
except ImportError:
    from PerceptionSystem import PerceptionSystem
    from OrderVerificationSystem import OrderVerificationSystem
    from NavigationSystem import NavigationSystem


class TIAGo():
    """
    TIAGo Robot class for restaurant service operations.
    
    This class represents a TIAGo robot operating in a restaurant environment.
    It integrates multiple systems (perception, navigation, order verification)
    to coordinate service tasks such as food delivery and table clearing.
    
    The robot follows a state machine approach with different order phases:
    - Phase 0: Idle (no active task)
    - Phase 1: Moving to service/table area
    - Phase 2: Grasping dish/empty plates
    - Phase 3: Moving to table/washing area
    - Phase 4: Placing dish/empty plates and verifying customer satisfaction
    - Phase 5: Returning to service area
    
    The robot communicates with the OrchestrationManager to receive orders
    and report status updates through ROS topics.
    
    Attributes:
        id (int): Robot identifier number
        x (float): Current x position coordinate
        y (float): Current y position coordinate
        status (str): Robot availability status ("available" or "occupied")
        order_phase (int): Current phase of order execution (0-5)
        target_table (int): Table number for current service task
        dish (str): Current dish being handled or "clearing" for cleanup tasks
        perception_system (PerceptionSystem): Handles sensing and object recognition
        order_verification_system (OrderVerificationSystem): Verifies order delivery
        navigation_system (NavigationSystem): Handles path planning and movement
    """

    def __init__(self):
        """
        Initialize the TIAGo robot with subsystems and communication channels.
        
        Sets up:
        - ROS node and parameters
        - Internal state variables
        - Restaurant map representation
        - Subsystems (perception, navigation, order verification)
        - ROS publishers and subscribers
        """
        # Initialize ROS node with error handling for multiple instances
        try:
            rospy.init_node('TIAGo', anonymous=True)
        except rospy.exceptions.ROSInitException:
            rospy.logwarn("ROS node already initialized. Using existing node.")
            
        # Get parameters from ROS parameter server
        frequency = rospy.get_param('~frequency', 0)  # Default to 0 if not specified
        self.id = rospy.get_param('~tiago_id', 1)  # Robot ID from launch file
        
        # Initialize robot state
        self.x = 0  # Initial x position
        self.y = 0  # Initial y position
        self.status = "available"  # Initial availability status
        self.order_phase = 0  # No active order initially
        self.target_table = None  # No target table initially
        self.dish = None  # No dish assigned initially

        # Generate restaurant map layout
        self.generation_map()
                
        # Initialize subsystems with reference to this robot instance
        self.perception_system = PerceptionSystem(self)
        self.order_verificatiion_system = OrderVerificationSystem(self)
        self.navigation_system = NavigationSystem(self)
        
        # Set up ROS publishers
        # Publisher for robot availability status updates
        self.publisher_availability = rospy.Publisher('availability', String, queue_size=10)
        
        # Publisher for robot position updates
        self.publisher_position = rospy.Publisher('position', Point, queue_size=10)
        
        # Publisher for sending clearing order requests
        self.publisher_clearing_order = rospy.Publisher('orders', String, queue_size=10)
        
        # Subscribe to incoming order requests from OrchestrationManager
        rospy.Subscriber("order_TIAGo", String, self.manage_order_request)

        # Set update rate for the robot's control loop
        self.rate = rospy.Rate(10)  # 10Hz - update 10 times per second

    def generation_map(self):
        """
        Generate the restaurant map with coordinates for tables and service areas.
        
        Creates a coordinate system representation of the restaurant layout:
        - Service area: Where dishes are picked up
        - Washing area: Where empty plates are returned
        - Tables: Where customers are seated and served
        
        Each coordinate is stored as [x, y, width, height] in meters
        with origin at the top-left corner of the restaurant.
        """
        self.nb_tables = 60  # Number of tables in the restaurant
        self.service_area_coords = [0, 0, 3, 3]  # [x, y, width, height] in meters
        self.washing_area_coords = [38, 0, 3, 3]  # Cleaning area coordinates
        
        # Generate table coordinates based on a grid layout
        self.table_coords = {}
        for i in range(0, self.nb_tables):
            decade_digit = i % 10
            unit_digit = i - decade_digit
            # Each table is 2m × 1m arranged in a grid pattern
            self.table_coords[i+1] = [unit_digit * 3, decade_digit * 4, 2, 1]
            
        return None

    def send_availability(self):
        """
        Publish the current availability status of the robot.
        
        Formats and publishes a status message to the 'availability' topic
        that can be read by the OrchestrationManager for task assignment.
        
        Message format: "TIAGo X : status" 
        Examples: "TIAGo 1 : occupied", "TIAGo 4 : available"
        """
        availability_message = f"TIAGo {self.id} : {self.status}"
        self.publisher_availability.publish(availability_message)

    def send_position(self):
        """
        Publish the current position of the robot in the restaurant.
        
        Creates and publishes a Point message to the 'position' topic
        that can be used by the OrchestrationManager for coordination.
        
        The message contains:
        - x, y: Position coordinates in meters
        - z: Used to transmit the robot ID (not actual z-position)
        """
        position_msg = Point()
        position_msg.x = self.x
        position_msg.y = self.y
        position_msg.z = self.id  # Using z to transmit robot ID

        self.publisher_position.publish(position_msg)
    
    def manage_order_request(self, msg):
        """
        Process and handle incoming order requests from OrchestrationManager.
        
        Parses the order message to extract:
        - Target TIAGo robot ID
        - Target table number
        - Dish to be served
        
        If the order is for this robot, updates internal state and begins execution.
        
        Args:
            msg (String): ROS message containing the order details
                Format: "TIAGo n°X, table : Y, dish : Z"
                
        Returns:
            None
        """
        try:
            # Extract TIAGo ID from the message format "TIAGo n°X, ..."
            id_start = msg.data.find("n°") + 2
            id_end = msg.data.find(",", id_start)
            tiago_id_concerned = int(msg.data[id_start:id_end])

            # Check if this order is for this specific TIAGo robot
            if tiago_id_concerned == self.id:                
                # Parse table number
                table_start = msg.data.find("table : ") + 8
                table_end = msg.data.find(",", table_start)
                self.target_table = int(msg.data[table_start:table_end])
                
                # Extract dish name
                dish_start = msg.data.find("dish : ") + 7
                self.dish = msg.data[dish_start:].strip()
                
                # Update robot status and phase
                self.status = "occupied"
                self.order_phase = 1  # Start with phase 1 (moving to service area)
                
                # Immediately send updated availability
                self.send_availability()
                
                rospy.loginfo(f"TIAGo {self.id} status changed to: {self.status}, serving table {self.target_table} with dish {self.dish}")
        except Exception as e:
            rospy.logerr(f"Error processing order request: {e}")
            
        return None
    
    def go_to(self, location):
        """
        Navigate the robot to a specified location in the restaurant.
        
        Uses the navigation system to plan and execute movement to target location,
        while simulating incremental position updates for visualization.
        
        Args:
            location (list): [x, y, width, height] coordinates of target area
            
        Returns:
            bool: True if navigation was successful, False otherwise
        """
        # Validate location data
        if not location:
            rospy.logwarn(f"TIAGo {self.id}: Invalid location for navigation")
            return False
        
        # Calculate target position (center of the specified area)
        target_x = location[0] + location[2] / 2
        target_y = location[1] + location[3] / 2
        
        # Request navigation to target position
        self.navigation_system.navigate_to([target_x, target_y])
        
        # Simulate movement by updating position incrementally
        steps = 5  # Number of steps to simulate movement
        start_x, start_y = self.x, self.y
        dx = (target_x - start_x) / steps
        dy = (target_y - start_y) / steps
        
        # Incrementally update position for smoother visualization
        for i in range(steps):
            # Update position step by step
            self.x += dx
            self.y += dy
            
            # Send position updates after each step
            self.send_position()
            rospy.sleep(0.2)  # Short delay to simulate movement time
            
            # Update the navigation system's internal position data
            if hasattr(self.navigation_system, 'slam') and self.navigation_system.slam:
                self.navigation_system.slam.robot_pose = [self.x, self.y, 0]
        
        # Final position update to ensure accuracy
        self.x = target_x
        self.y = target_y
        self.send_position()
        
        return True

    def operation(self):
        """
        Execute the robot's operation cycle based on current state.
        
        This is the main task execution method that implements the state machine logic.
        Different actions are taken based on the robot's status and order phase:
        
        For serving operations (dish != "clearing"):
        - Phase 1: Go to service area
        - Phase 2: Grasp the dish
        - Phase 3: Go to customer table
        - Phase 4: Place dish and verify customer satisfaction
        - Phase 5: Return to service area
        
        For clearing operations (dish == "clearing"):
        - Phase 1: Go to customer table
        - Phase 2: Grasp the empty plates
        - Phase 3: Go to washing area
        - Phase 4: Place empty plates
        - Phase 5: Return to service area
        
        When available, the robot remains at the service area.
        """
        # Case 1: Robot is occupied with a serving task
        if self.status == "occupied" and self.dish != "clearing":
            # The TIAGo robot is performing a serving operation
            if self.order_phase == 1:
                # Phase 1: Navigate to the service area
                if self.go_to(self.service_area_coords):
                    # Check for obstacles during navigation
                    perception_result = self.perception_system.perception()
                    if not perception_result:
                        rospy.logwarn(f"TIAGo {self.id} detected obstacle during navigation to service area and avoided it")
                    # Progress to next phase
                    self.order_phase = 2

            elif self.order_phase == 2:
                # Phase 2: Grasp the correct dish
                perception_result = self.perception_system.perception()
                if not perception_result:
                    rospy.logwarn(f"TIAGo {self.id} failed to grasp dish {self.dish} and tried again")
                # Progress to next phase
                self.order_phase = 3

            elif self.order_phase == 3:
                # Phase 3: Navigate to the customer's table
                if self.go_to(self.table_coords[self.target_table]):
                    # Check for obstacles during navigation
                    perception_result = self.perception_system.perception()
                    if not perception_result:
                        rospy.logwarn(f"TIAGo {self.id} detected obstacle during navigation to table {self.target_table} and avoided it")
                    # Progress to next phase
                    self.order_phase = 4

            elif self.order_phase == 4:
                # Phase 4: Place dish and check for customer feedback
                perception_result = self.perception_system.perception()
                if not perception_result:
                    rospy.logwarn(f"TIAGo {self.id} had difficulty finding suitable placement spot")
                
                # Interact with the customer
                rospy.loginfo(f"TIAGo {self.id} is interacting with customer at table {self.target_table}")
                potential_problems = self.order_verificatiion_system.verify_delivery_client()
                
                placement_problem, client_problem = potential_problems
                
                # Handle any problems that occurred during delivery
                if placement_problem or client_problem:
                    if placement_problem:
                        rospy.logwarn(f"TIAGo {self.id} had trouble placing dish {self.dish} at table {self.target_table}")
                    if client_problem:
                        rospy.logwarn(f"TIAGo {self.id} received complaint from customer: {client_problem}")
                        # Special case: handle request to clear empty plates
                        if client_problem == "empty_plates":                            
                            self.dish = "clearing"  # Change to clearing operation
                            self.order_phase = 1   # Reset to phase 1 for new task
                            rospy.loginfo(f"TIAGo {self.id} will clean away the empty plates for table {self.target_table}")
                            return
                else:
                    rospy.loginfo(f"TIAGo {self.id} successfully delivered {self.dish} to table {self.target_table}")
                
                # Progress to next phase
                self.order_phase = 5

            elif self.order_phase == 5:
                # Phase 5: Return to service area
                if self.go_to(self.service_area_coords):
                    # Check for obstacles during return journey
                    perception_result = self.perception_system.perception()
                    if not perception_result:
                        rospy.logwarn(f"TIAGo {self.id} detected obstacle during return to service area and avoided it")

                    # Task completed, reset robot state
                    rospy.loginfo(f"TIAGo {self.id} returned to service area")
                    self.status = "available"
                    self.order_phase = 0
                    self.target_table = None
                    self.dish = None
                    self.send_availability()

        # Case 2: Robot is occupied with a clearing task
        elif self.status == "occupied" and self.dish == "clearing":
            # The TIAGo robot is performing a clearing operation
            if self.order_phase == 1:
                # Phase 1: Navigate to the table with empty plates
                if self.go_to(self.table_coords[self.target_table]):
                    # Progress to next phase
                    self.order_phase = 2

            elif self.order_phase == 2:
                # Phase 2: Grasp the empty plates
                perception_result = self.perception_system.perception()
                if not perception_result:
                    rospy.logwarn(f"TIAGo {self.id} failed to grasp empty plates and tried again")
                # Progress to next phase
                self.order_phase = 3

            elif self.order_phase == 3:
                # Phase 3: Navigate to the washing/cleaning area
                if self.go_to(self.washing_area_coords):
                    # Check for obstacles during navigation
                    perception_result = self.perception_system.perception()
                    if not perception_result:
                        rospy.logwarn(f"TIAGo {self.id} detected obstacle during navigation to cleaning area and avoided it")
                    # Progress to next phase
                    self.order_phase = 4

            elif self.order_phase == 4:
                # Phase 4: Place empty plates in washing area
                perception_result = self.perception_system.perception()
                if not perception_result:
                    rospy.logwarn(f"TIAGo {self.id} had difficulty finding suitable placement spot")
                else:
                    rospy.loginfo(f"TIAGo {self.id}: Successfully placed empty plates in cleaning area")
                # Progress to next phase
                self.order_phase = 5

            elif self.order_phase == 5:
                # Phase 5: Return to service area
                if self.go_to(self.service_area_coords):
                    # Check for obstacles during return journey
                    perception_result = self.perception_system.perception()
                    if not perception_result:
                        rospy.logwarn(f"TIAGo {self.id} detected obstacle during return to service area and avoided it")

                    # Task completed, reset robot state
                    rospy.loginfo(f"TIAGo {self.id} returned to service area")
                    self.status = "available"
                    self.order_phase = 0
                    self.target_table = None
                    self.dish = None
                    self.send_availability()

        # Case 3: Robot is available (no active task)
        elif self.status == "available":
            # When not occupied, ensure robot stays at service area
            service_x = self.service_area_coords[0] + self.service_area_coords[2]/2
            service_y = self.service_area_coords[1] + self.service_area_coords[3]/2
            
            # If robot is not at service area, navigate there
            if (abs(self.x - service_x) > 0.1 or abs(self.y - service_y) > 0.1):
                self.go_to(self.service_area_coords)

        # Update navigation system state
        if hasattr(self.navigation_system, 'update'):
            self.navigation_system.update()

        return None
    

if __name__ == '__main__':
    try:
        # Initialize and start TIAGo robot
        tiago = TIAGo()
        rospy.loginfo(f"TIAGo robot {tiago.id} initialized and running")
        
        # Create a rate object to control update frequency
        rate = rospy.Rate(1)  # 1 Hz - execute once per second
        
        # Main control loop - runs until ROS is shutdown
        while not rospy.is_shutdown():
            # Execute robot's current operation cycle
            tiago.operation()
            
            # Send status updates
            tiago.send_availability()
            tiago.send_position()
            
            # Sleep to maintain the desired update frequency
            rate.sleep()
            
    except rospy.ROSInterruptException:
        # Handle graceful shutdown on Ctrl+C
        rospy.loginfo("TIAGo node terminated by user")
    except Exception as e:
        # Log any other unexpected errors
        rospy.logerr(f"TIAGo node error: {e}")
