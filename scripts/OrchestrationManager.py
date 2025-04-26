#!/usr/bin/env python3

# Standard libraries
import math
import time 
import rospy

# ROS message types
from std_msgs.msg import String
from geometry_msgs.msg import Point



# Global assumptions
# 1: The orchestration system already has the map of the restaurant 
# 2: The way the tables are positioned in the restaurant is fixed, only their numbers can change

class OrchestrationManager():
    """
    The Orchestration Manager coordinates order processing and robot task assignment.
    
    This class manages:
    - Order queue management (storing, prioritizing, and assigning)
    - Robot status tracking and coordination
    - Task distribution based on robot availability and proximity
    - Error handling and reporting
    - Restaurant map representation
    
    It implements the singleton pattern to ensure only one orchestration manager exists.
    
    Class Attributes:
        list_dishes (list): Available menu items the restaurant serves
    
    Instance Attributes:
        orderQueue (list): Orders waiting to be assigned, stored based on priority
        ongoingTasks (list): Tasks currently being performed by robots
        dictTIAGoAvailable (dict): Maps robot IDs to availability status
        dictTIAGoPosition (dict): Maps robot IDs to position coordinates
        error_occured (int): Count of errors not yet transmitted to staff
        error_messages (list): Error messages waiting to be sent to staff
        last_order_message (str): Most recent order message to avoid duplicates
    """

    list_dishes = ["Nare", "Oshi", "Nigri", "Gunkan", "Maki", "Futo", "Temaki", "Inari", "Temari", "Sashimi", "Uramaki"]
    
    def __new__(cls): 
        """
        Implements singleton pattern for the OrchestrationManager.
        
        Returns:
            OrchestrationManager: The singleton instance
        """
        if not hasattr(cls, 'instance'):
            cls._instance = super(OrchestrationManager, cls).__new__(cls)
            cls._instance.__init__()
        return cls._instance
    
    def __init__(self):
        """
        Initializes the OrchestrationManager with data structures and ROS communication.
        
        - Sets up data structures for orders and robot status
        - Creates the restaurant map
        - Establishes ROS publishers and subscribers
        """
        # Initialize data structures
        self.orderQueue = []
        self.ongoingTasks = []
        self.dictTIAGoAvailable = {}
        self.dictTIAGoPosition = {}
        self.error_occured = 0
        self.error_messages = []
        self.last_order_message = None
        
        # Generate the map of the restaurant
        self.generation_map()
        
        # Set up ROS communication
        # Publisher for sending orders to TIAGo robots
        self.publisher_order = rospy.Publisher("order_TIAGo", String, queue_size=10)
        
        # Publisher for sending error messages to staff interfaces
        self.publisher_error = rospy.Publisher("error_messages", String, queue_size=10)
        
        # Subscribers for receiving information from POS and robots
        rospy.Subscriber("orders", String, self.order_storing)
        rospy.Subscriber("availability", String, self.manage_availability)
        rospy.Subscriber("position", Point, self.manage_position)

    def generation_map(self):
        """
        Generates the restaurant map with coordinates for tables and service areas.
        
        The map uses a coordinate system where:
        - Each coordinate is [x, y, length, width] in meters
        - Origin (0,0) is at the top-left corner of the restaurant
        """
        self.nb_tables = 60  # Number of tables 
        self.service_area_coords = [0, 0, 3, 3]  # [x_coord, y_coord, length, width] in meters
        self.washing_area_coords = [38, 0, 3, 3]  # Cleaning area coordinates
        
        # Generate table coordinates based on a grid layout
        self.table_coords = {}
        for i in range(0, self.nb_tables):
            decade_digit = i % 10
            unit_digit = i - decade_digit
            self.table_coords[i+1] = [unit_digit*3, decade_digit*4, 2, 1]  # [x, y, length, width]
        
        return None
    
    def extraction_data(self, msg):
        """
        Extracts structured data from order message strings.
        
        Parses the message format: "Table : X, dish : Y, loop n°Z, priority : P"
        
        Args:
            msg (String): ROS message containing order information
            
        Returns:
            tuple: Contains (table_number, dish_name, priority, original_message)
        """
        # Extract table number
        index_order = 8  # Skip "Table : " part
        table_number = msg.data[8]
        if msg.data[9] != " " and msg.data[9] != ",":  
            table_number = msg.data[8:10]
            index_order = 9 
        table_number = ''.join(c for c in table_number if c.isdigit())
        table_number = int(table_number)
        index_order += 9  # Skip ", dish : " part
        
        # Extract dish name
        index_order += 1 
        index_order_beginning_dish = index_order
        while msg.data[index_order] not in (" ", ","):
            index_order += 1
        dish = msg.data[index_order_beginning_dish:index_order]
        
        # Extract priority level
        priority = int(msg.data[-1])
        
        return table_number, dish, priority, msg.data
    
    def order_storing(self, msg):
        """
        Processes and stores incoming orders from the POS system.
        
        Validates order data and adds it to the appropriate position in the queue
        based on priority level.
        
        Priority levels:
        - 0: Normal order (added to end of queue)
        - 1: Priority order (added to middle of queue)
        - 2: Cleaning order (added to front of queue)
        
        Args:
            msg (String): ROS message containing order information
        
        Returns:
            None
        """
        # Avoid duplicate processing if message was just received
        if self.last_order_message == msg.data:
            return None
        self.last_order_message = msg.data
        rospy.loginfo("Message received by the orchestration manager: %s", msg.data)
        
        # Extract order data
        table_number, dish, priority, message = self.extraction_data(msg)
        order_data = [table_number, dish]
        
        # Validate order data
        if dish not in OrchestrationManager.list_dishes:
            self.error_occured += 1
            self.error_messages.append(message + " , Problem: Unknown dish")
            return None
        
        if table_number not in range(1, 61):
            self.error_occured += 1
            self.error_messages.append(message + " , Problem: Unknown table number")
            return None
        
        if priority not in range(0, 3):
            self.error_occured += 1
            self.error_messages.append(message + " , Problem: Unknown priority")
            return None
        
        # Add order to queue based on priority
        if priority == 2:  # Cleaning order (highest priority)
            self.orderQueue.insert(0, order_data)
        elif priority == 1:  # Priority order (medium priority)
            self.orderQueue.insert(len(self.orderQueue)//2, order_data)
        else:  # Normal order (lowest priority)
            self.orderQueue.append(order_data)
        
        return None
    
    def manage_availability(self, msg):
        """
        Processes TIAGo robot availability updates.
        
        Parses messages of format "TIAGo X : available/occupied" and updates status tracking.
        
        Args:
            msg (String): ROS message with robot availability information
            
        Returns:
            None
        """
        parts = msg.data.split(' : ')
        if len(parts) != 2:
            rospy.logerr(f"Invalid availability message format: {msg.data}")
            return None
            
        id_part = parts[0].split(' ')
        if len(id_part) != 2 or not id_part[1].isdigit():
            rospy.logerr(f"Invalid TIAGo ID format in availability message: {parts[0]}")
            return None
            
        tiago_id = int(id_part[1])
        tiago_availabiliy = parts[1].strip()
        self.dictTIAGoAvailable[tiago_id] = tiago_availabiliy
        
        return None

    def manage_position(self, msg):
        """
        Processes TIAGo robot position updates.
        
        Stores robot position data from Point messages, where:
        - x, y: Robot position coordinates
        - z: Robot ID
        
        Args:
            msg (Point): ROS message with robot position
            
        Returns:
            None
        """
        tiago_id = int(msg.z)  # Robot ID is stored in z coordinate
        tiago_abscysse = msg.x  # x position
        tiago_ordinate = msg.y  # y position
        self.dictTIAGoPosition[tiago_id] = [tiago_abscysse, tiago_ordinate]
        
        return None
    
    def compute_distance(self, tiago_id):
        """
        Computes the Euclidean distance between a TIAGo robot and the serving area.
        
        Used to find the closest available robot for task assignment.
        
        Args:
            tiago_id (int): ID of the TIAGo robot
            
        Returns:
            float: Distance to serving area, or large default if position unknown
        """
        # Check if position data is available
        if tiago_id not in self.dictTIAGoPosition:
            rospy.logwarn(f"No position data for TIAGo {tiago_id}, using default position")
            return 1000  # Large default distance to make this robot unlikely to be chosen
            
        # Get robot and serving area positions
        tiago_x = self.dictTIAGoPosition[tiago_id][0]
        tiago_y = self.dictTIAGoPosition[tiago_id][1]
        serving_area_x = self.service_area_coords[0]
        serving_area_y = self.service_area_coords[1]
        
        # Calculate Euclidean distance
        distance = math.sqrt((tiago_x-serving_area_x)**2 + (tiago_y-serving_area_y)**2)
        
        return distance
    
    def assign_order(self):
        """
        Assigns the highest priority order to the closest available robot.
        
        This method:
        1. Checks for orders to assign and available robots
        2. Finds the closest robot to the serving area
        3. Publishes the order to the selected robot
        4. Updates robot status and removes the order from queue
        
        Returns:
            None
        """
        # Check if there are orders to assign
        if not self.orderQueue:
            rospy.logwarn("No orders available to assign")
            return None
            
        # Find available robots
        available_robots = [id for id, status in self.dictTIAGoAvailable.items() if status == "available"]
        if not available_robots:
            rospy.logwarn("No available robots to assign order")
            return None
            
        # Get the next order data
        order_data = self.orderQueue[0]
        table_number, dish = order_data[0], order_data[1]
        
        # Find the closest available robot
        tiago_id = 0
        distance_min = 1000  # Distance greater than maximum possible in restaurant
        
        for tiago_id_available in available_robots:
            distance = self.compute_distance(tiago_id_available)
            if distance < distance_min:
                tiago_id = tiago_id_available
                distance_min = distance
                
        if tiago_id == 0:
            rospy.logwarn("Failed to select a robot for order assignment")
            return None
        
        # Create and publish order message
        order_msg = f"TIAGo n°{tiago_id}, table : {table_number}, dish : {dish}"
        self.publisher_order.publish(order_msg)
        
        # Update robot status and remove order from queue
        self.dictTIAGoAvailable[tiago_id] = "occupied"
        self.orderQueue.pop(0)
        
        rospy.loginfo(f"TIAGo {tiago_id} has been assigned to the order")
        
        return None
    
    def send_error_messages(self):
        """
        Publishes accumulated error messages to staff interfaces.
        
        Processes all pending error messages in the queue and clears them after sending.
        
        Returns:
            None
        """
        while self.error_occured != 0:
            error_msg = self.error_messages[0]
            rospy.loginfo(f"Sending error message: {error_msg}")
            self.publisher_error.publish(error_msg)
            del self.error_messages[0]  # Remove the processed error message
            self.error_occured -= 1
        
        return None

    def orchestration(self):
        """
        Main operational loop for the orchestration system.
        
        Continuously:
        1. Assigns orders to available robots
        2. Sends error messages when needed
        3. Maintains timing for simulation
        
        Returns:
            None
        """
        nb_turns = 0 
        t = time.time()
        
        while (nb_turns < 2000) and (len(self.ongoingTasks) == 0):
            # Only assign orders when both orders and available robots exist
            if self.orderQueue and self.dictTIAGoAvailable:
                self.assign_order()
                
            # Process any pending error messages
            if self.error_occured > 0:
                self.send_error_messages()
                
            # Update timing for simulation
            if time.time() - t > 1:
                t = time.time()
                nb_turns += 1
                
            # Yield control briefly to avoid hogging CPU
            rospy.sleep(0.1)
        
        return None


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node("OrchestrationManager", anonymous=True)
        # Create and run the orchestration manager
        orchestrationManager = OrchestrationManager()
        orchestrationManager.orchestration()
    except rospy.exceptions.ROSInitException:
        # Handle case where node is already initialized
        rospy.logwarn("ROS node already initialized. Using existing node.")
        orchestrationManager = OrchestrationManager()
        orchestrationManager.orchestration()
