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
        
        #Creatio of a pblisherthat will publish Point (position) messages to the 'position' topic
        self.publisher_position = rospy.Publisher('position', Point, queue_size=10)
        
        #Creatio of a pblisherthat wil publis Stringmessages to the 'orders' topic
        #(To notfy the rchestrtion maager tht thereare table to be cleaned)
        self.publisher_clearing_order = rospy.Publisher('orders', Point, queue_size=10)

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
        availability_message = "TIAGo " +self.id + " : " + self.status 
        rospy.loginfo("Availability message sent by a TIAGo platform : " + availability_message)
        self.publisher.publish(availability_message)

    def send_position(self):
        """
        Publish the position of the robot.
        Instead of the position on the z-axis, the z component of the message will be used to transmit the id of the robot
        """
        position_msg = Point()
        position_msg.x = self.x
        position_msg.y = self.y
        position_msg.z = self.id

        rospy.loginfo("Position message sent by the TIAGo platform n°%f: (%f,%f)", position_msg.z, position_msg.x, position_msg.y)
        self.publisher_position.publish(position_msg)
    
    def manage_order_request(self,msg):
        """
        Manage the order request received from the orchestration manager.
        - It checks if the ID of the robot concerned is its
        - In that case, it sets the order_phase to 1 and sets the availability to "occupied", then send it to the ROS topic 
    

        Exemple of message received : "TIAGo n°3, table : 37, dish : Gunkan" 
        """
        tiago_id_concerned = msg.data[8] #ID of the TIAGo robot concerned

        if tiago_id_concerned == self.id:
            self.target_table = int(msg.data[19])
            if msg.data[20] != " ":#The table number has two digits
                self.target_table = int(msg.data[19:21])
                two_digits = 1
            dish = msg.data[29+two_digits:-1]

            self.status = "occupied"

        return None
    
    def go_to(self,location):
        pass

    def operation(self):
        """
        Simulate the operation of the robot by applying different processes depending on the order received and the phase of the order we need to execute
        """

        if self.status == "occupied" and self.dish != "clearing":
            #The TIAGo robot are not doing a serving operation
            if self.order_phase ==  1 :
                #Entering phase 1 : the TIAGo robot has to go to the service area
                pass

            elif self.order_phase ==  2 :
                #Entering phase 2 : the TIAGo robot has to take the right plate
                pass

            elif self.order_phase ==  3 :
                #Entering phase 3 : the TIAGo robot has to go to the table where it will have to serve the dish
                pass

            elif self.order_phase ==  4 :
                #Entering phase 4 : the TIAGo robot has to serve the plate and to see if the client has some reclamationscomplaints 
                pass

            elif self.order_phase ==  5 :
                #Entering phase 5 : the TIAGo robot has to come back to the service area 
                pass

        if self.status == "occupied" and self.dish == "clearing":
            #The TIAGo robot are not doing a clearing operation
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

    tiago = TIAGo()

    tiago.operation()
