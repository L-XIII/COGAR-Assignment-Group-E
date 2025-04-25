#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

from PerceptionSystem import PerceptionSystem
from OrderVerificationSystem import OrderVerificationSystem


class TIAGo():
    def __init__(self):
        """
        A TIAGo robot receives order from the orchestration manager and executes them. It send to the orchestration manager its availability and its position,
        as well as, pontentially, the information that a table needs to be cleared

        An TIAGo instance has several parameters:
        -self.id : identifier of the robot (int)
        - x : abscysse of the robot (initialized at 0)
        - y : ordinate of the robot (initialized at 0)
        - status : status of the robot, can be "available" or "occupied"
        - order_phase : phase of the order we are trying to execute (between 1 and 5 - 0 when there is no order) 
        - target_table : table at which the robot has to serve or take the empty plate (parameter set to "None" if the robot has no order to executes)
        - dish : dish to bring to the target table (parameter set to "clearing" if the robot has to perform a clearing operation)
        
        """
        try:
            rospy.init_node('TIAGo', anonymous=True)
        except rospy.exceptions.ROSInitException:
            rospy.logwarn("ROS node already initialized. Using existing node.")
            
        # Get the 'frequency' parameter from the roslaunch file, default to 0 if not set
        frequency = rospy.get_param('~frequency', 0)
        # Get the robot ID from the 'tiago_id' parameter set in the launch file
        self.id = rospy.get_param('~tiago_id', 1)
        self.x = 0
        self.y = 0
        self.status = "available"
        self.order_phase = 0
        self.target_table = None
        self.dish = None 

        # Generation of the map of the restaurant
        self.generation_map()
                
        # Generation of the different modules of the TIAGo platform
        self.perception_system = PerceptionSystem(self)
        self.order_verificatiion_system = OrderVerificationSystem(self)
        
        # Creation of the availability publisher that will publish String messages to the 'availability' topic
        self.publisher_availability = rospy.Publisher('availability', String, queue_size=10)
        
        #Creation of a publisher that will publish Point (position) messages to the 'position' topic
        self.publisher_position = rospy.Publisher('position', Point, queue_size=10)
        
        #Creation of a publisher that will publish String messages to the 'orders' topic
        self.publisher_clearing_order = rospy.Publisher('orders', String, queue_size=10)
        
        # Subscribe to order_TIAGo topic to receive orders from OrchestrationManager
        rospy.Subscriber("order_TIAGo", String, self.manage_order_request)

        self.rate = rospy.Rate(10) # 10Hz

    def generation_map(self):
        """
        Generate the map of the restaurant, same function that the one used for the orchestration manager
        """
        self.nb_tables = 60 #Number of tables 
        self.service_area_coords = [0,0,3,3] #x_coord, y_coord, length, width (in meter)
        self.washing_area_coords = [38,0,3,3]
        self.table_coords = {}
        for i in range(0,self.nb_tables):
            decade_digit = i%10
            unit_digit = i-decade_digit
            self.table_coords[i+1]=[unit_digit*3,decade_digit*4,2,1]
        return None

    def send_availability(self):
        """
        Publish the availability of the robot.
        Exemples of messages : "TIAGo 1 : occupied", "TIAGo 4 : available"
        """
        availability_message = "TIAGo " + str(self.id) + " : " + self.status 
        self.publisher_availability.publish(availability_message)

    def send_position(self):
        """
        Publish the position of the robot.
        Instead of the position on the z-axis, the z component of the message will be used to transmit the id of the robot
        """
        position_msg = Point()
        position_msg.x = self.x
        position_msg.y = self.y
        position_msg.z = self.id

        self.publisher_position.publish(position_msg)
    
    def manage_order_request(self, msg):
        """
        Manage the order request received from the orchestration manager.
        - It checks if the ID of the robot concerned is its
        - In that case, it sets the order_phase to 1 and sets the availability to "occupied", then send it to the ROS topic 
        """
        try:
            # Extract TIAGo ID from the message format "TIAGo n°X, ..."
            id_start = msg.data.find("n°") + 2
            id_end = msg.data.find(",", id_start)
            tiago_id_concerned = int(msg.data[id_start:id_end])

            # Check if this order is for this TIAGo robot
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
                self.order_phase = 1
                
                # Immediately send updated availability
                self.send_availability()
                
                rospy.loginfo(f"TIAGo {self.id} status changed to: {self.status}, serving table {self.target_table} with dish {self.dish}")
        except Exception as e:
            rospy.logerr(f"Error processing order request: {e}")
            
        return None
    
    def go_to(self,location):
        pass

    def operation(self):
        """
        Simulate the operation of the robot by applying different processes depending on the order received and the phase of the order we need to execute
        """

        if self.status == "occupied" and self.dish != "clearing":
            #The TIAGo robot is doing a serving operation
            if self.order_phase ==  1 :
                #Entering phase 1 : the TIAGo robot has to go to the service area
                self.order_phase = 2

            elif self.order_phase ==  2 :
                #Entering phase 2 : the TIAGo robot has to take the right plate
                self.order_phase = 3

            elif self.order_phase ==  3 :
                #Entering phase 3 : the TIAGo robot has to go to the table where it will have to serve the dish
                self.order_phase = 4

            elif self.order_phase ==  4 :
                #Entering phase 4 : the TIAGo robot has to serve the plate and to see if the client has some reclamationscomplaints 
                rospy.loginfo(f"TIAGo {self.id} is interacting with customer at table {self.target_table}")
                potential_problems = self.order_verificatiion_system.verify_delivery_client()
                
                placement_problem, client_problem = potential_problems
                
                if placement_problem or client_problem:
                    if placement_problem:
                        rospy.logwarn(f"TIAGo {self.id} had trouble placing dish {self.dish} at table {self.target_table}")
                    if client_problem:
                        rospy.logwarn(f"TIAGo {self.id} received complaint from customer: {client_problem}")
                else:
                    rospy.loginfo(f"TIAGo {self.id} successfully delivered {self.dish} to table {self.target_table}")
                
                self.order_phase = 5

            elif self.order_phase ==  5 :
                #Entering phase 5 : the TIAGo robot has to come back to the service area 
                rospy.loginfo(f"TIAGo {self.id} returned to service area")
                self.status = "available"
                self.order_phase = 0
                self.target_table = None
                self.dish = None
                self.send_availability()

        if self.status == "occupied" and self.dish == "clearing":
            #The TIAGo robot is doing a clearing operation
            if self.order_phase ==  1 :
                #Entering phase 1 : the TIAGo robot has to go to the table where is located the empty plate it has to remove
                pass

            elif self.order_phase ==  2 :
                #Entering phase 2 : the TIAGo robot has to take the empty plate
                pass

            elif self.order_phase ==  3 :
                #Entering phase 3 : the TIAGo robot has to go to the cleaning area
                pass

            elif self.order_phase ==  4 :
                #Entering phase 4 : the TIAGo robot has to put down the plate 
                pass

            elif self.order_phase ==  5 :
                #Entering phase 5 : the TIAGo robot has to come back to the service area 
                pass


        else:
            self.go_to(self.service_area_coords)

        return None
    

if __name__ == '__main__':
    try:
        tiago = TIAGo()
        rospy.loginfo(f"TIAGo robot {tiago.id} initialized and running")
        
        # Create a rate object to control update frequency
        rate = rospy.Rate(1)  # 1 Hz - execute once per second
        
        # Run until ROS is shut down
        while not rospy.is_shutdown():
            tiago.operation()
            
            # Send status updates periodically
            tiago.send_availability()
            tiago.send_position()
            
            # Sleep to maintain the desired update frequency
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("TIAGo node terminated by user")
    except Exception as e:
        rospy.logerr(f"TIAGo node error: {e}")
