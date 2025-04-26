#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import String


class POS:
    """
    Point of Sale (POS) system for the restaurant simulation.
    
    This class simulates a point-of-sale terminal that generates customer orders 
    and publishes them to the restaurant's order management system via ROS.
    
    The POS randomly generates orders for dishes and tables, assigns priorities,
    and publishes these orders to the 'orders' topic where they can be picked up
    by the OrchestrationManager.
    
    Class Attributes:
        list_dishes (list): Available menu items that can be ordered by customers
        nb_dishes (int): Total number of dishes available on the menu
    """
    
    # Menu items available in the restaurant
    list_dishes = ["Nare", "Oshi", "Nigri", "Gunkan", "Maki", 
                   "Futo", "Temaki", "Inari", "Temari", "Sashimi", "Uramaki"]
    nb_dishes = len(list_dishes)

    def __init__(self):
        """
        Initialize the Point of Sale system.
        
        Sets up the ROS publisher for orders and initializes the restaurant configuration.
        """
        # Restaurant configuration
        self.nb_tables = 60  # Number of tables in the restaurant
        
        # Create a publisher for the 'orders' topic
        # Queue size of 10 means up to 10 messages can be stored if the subscriber
        # is not processing messages fast enough
        self.publisher = rospy.Publisher('orders', String, queue_size=10)
        
        # Initialize the ROS node
        # anonymous=True allows ROS to add a unique suffix to the node name
        # to ensure unique node names when multiple POS nodes might be running
        rospy.init_node('POS', anonymous=True)
        
        # Set the publication rate to 1 Hz (once per second)
        self.rate = rospy.Rate(1)
    
    def order_emission(self, nb_loops):
        """
        Generate and emit random food orders.
        
        This method has a 30% chance (3 out of 10) of generating an order on each call.
        When an order is generated, it includes a random table number, dish, and priority level.
        
        Message format: "Table : X, dish : Y, loop n°Z, priority : P"
        Where:
            X = Table number (1-60)
            Y = Dish name from the menu
            Z = Current simulation loop number
            P = Priority level (0 = normal, 1 = high priority)
        
        Args:
            nb_loops (int): Current simulation loop number, included in the order message
            
        Returns:
            None
        """
        # 30% chance (3 out of 10) to generate an order
        emission_possible = random.randint(1, 10)
        
        if emission_possible <= 3:
            # Generate random order details
            dish = random.choice(POS.list_dishes)  # Random dish from menu
            table_number = random.randint(1, 60)   # Random table number
            priority = random.randint(0, 1)        # Random priority level
            
            # Construct the order message
            order_msg = f"Table : {table_number}, dish : {dish}, loop n°{nb_loops}, priority : {priority}"
            
            # Log and publish the order
            rospy.loginfo(f"Message sent by POS : {order_msg}")
            self.publisher.publish(order_msg)
            
        # Sleep to maintain the publication rate
        self.rate.sleep()

    def shutdown(self):
        """
        Shut down the Point of Sale system.
        
        Signals ROS to shut down this node cleanly when the simulation is complete.
        """
        rospy.signal_shutdown("End of the simulation for this POS")


if __name__ == '__main__':
    try:
        # Create a POS instance
        pos = POS()
        
        # Run the simulation for 2000 loops (approximately 2000 seconds at 1 Hz)
        nb_loops = 0
        while nb_loops < 2000 and not rospy.is_shutdown():
            pos.order_emission(nb_loops)
            nb_loops += 1
            
        # Clean shutdown when simulation completes
        pos.shutdown()
        
    except rospy.ROSInterruptException:
        # Handle interruptions (e.g., Ctrl+C)
        rospy.loginfo("POS node terminated by user")
