#!/usr/bin/env python

#Libraries
import math
import time 
import random
import rospy


from std_msgs.msg import String
from geometry_msgs.msg import Point

import PointOfSale
import TIAGo


#Assumption 1: The orchestration system already has the map of the restaurant 
#Assumption 2: The way the tables are positionned in the restaurant is always the same, what changes is only their numbers (see GitHub)


#We localize the different areas (tables, serving area,...) by their top left corner and their length and width
#We use the singleton pattern for the OrchestrationManager class because we want only one object of this kind


class OrchestrationManager():
    """
    The Orchestration Manager manages the order processing and repartition
    The class OrchestrationManager has one parameter:
    - list_dishes : list of dishes the restaurant serves
    An OrchestrationManager instance has several parameters:
    - orderQueue : list in which the orders are stored in a specific order
    - listPOS : list of the point of sales of the restaurant 
    - orderQueue : list where the orders are stored
    - ongoingQueue : list where the ongoing tasks permformed by the robots are stored
    - dictTIAGoAvailable : stores the identifiers of the TIAGo robots and their availability
    - dictTIAGoPosition : stores the identifiers and the positions of the TIAGo robots
    - error_occured : counts how many errors occurred and have not been transmitted to the staff
    - error_messages : stores the error_messages to be transmitted to the staff
    - publisher_order : publisher that send orders to the TIAGo robots
    - publisher_error : publisher that send the error message to the staff
    """

    list_dishes = ["Nare","Oshi","Nigri","Gunkan","Maki","Futo","Temaki","Inari","Temari","Sashimi","Uramaki"]
    
    def __new__(cls): 
        """
        The new method a method used to create a new instance of a class.

        """
        if not hasattr(cls, 'instance'):#Check if there is already an instance of this class, if there isn't the following code is executed
            cls._instance = super(OrchestrationManager, cls).__new__(cls)
            cls._instance.__init__()
        return cls._instance
    
    def __init__(self):
        """
        Method called for initializing the OrchestrationManager. 
        - It generates the map of the restaurant
        """
        self.orderQueue = []
        self.ongoingTasks = []
        self.dictTIAGoAvailable = {}
        self.dictTIAGoPosition = {}
        self.error_occured = 0
        self.error_messages = []
        self.last_order_message = None
        #Generation of the map of the restaurant
        self.generation_map()
        #Subsrciption to the ROS topic "orders"

        #Creation of the order_TIAGo publisher
        self.publisher_order = rospy.Publisher("order_TIAGo", String, queue_size=10)

        #Creation of the publisher that will notify the staff of potential errors
        self.publisher_error = rospy.Publisher("error_messages", String, queue_size=10)

        rospy.Subscriber("orders", String, self.order_storing)
        rospy.Subscriber("availability", String, self.manage_availability)
        rospy.Subscriber("position", Point, self.manage_position)

    def generation_map(self):
        """
        Generate the map of the restaurant
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
    
    def extraction_data(self, msg):
        """
        This function extract the datas of the received order 
        """
        index_order = 7 #We skip the "Table : " part of the message

        #Extraction of the table number
        index_order = 8 
        table_number = msg.data[8]

        if msg.data[9] != " " and msg.data[9] != ",":  # Check for space OR comma
            # If the next character is not space or comma, it's part of the table number
            table_number = msg.data[8:10]
            index_order = 9 
        
        # Remove any non-digit characters (like commas)
        table_number = ''.join(c for c in table_number if c.isdigit())
        table_number = int(table_number)

        index_order += 9 #We skip the ", dish : " part of the message

        #Extraction of the dish name
        index_order += 1 
        index_order_beginning_dish = index_order
        dish = msg.data[index_order]       
        while msg.data[index_order] != " ":
            dish = msg.data[index_order_beginning_dish:index_order+1]
            index_order+=1
        
        #Extraction of the priority
        priority = int(msg.data[-1])

        return table_number, dish, priority, msg.data
    
    def order_storing(self, msg):
        # Avoid duplicate logging/processing if the same message was just received
        if self.last_order_message == msg.data:
            return None
        self.last_order_message = msg.data
        rospy.loginfo("Message received by the orchetration manager : %s", msg.data)
        table_number, dish, priority, message = self.extraction_data(msg)
        
        order_data = [table_number, dish]

        if dish not in OrchestrationManager.list_dishes:
            self.error_occured+=1
            self.error_messages.append(message + " , Problem : Unknown dish")
            return None
        
        if table_number not in range(1, 61,1):
            self.error_occured+=1
            self.error_messages.append(message + " , Problem : Unknown table number")
            return None
        
        if table_number not in range(0, 3,1):
            self.error_occured+=1
            self.error_messages.append(message + " , Problem : Unknown priority")
            return None

        if priority == 2:#It is a cleaning order, it takes precendence over the other orders
            self.orderQueue.insert(0, order_data)

        if priority == 1:#It is a priority order, we place it in the middle of the queue 
            self.orderQueue.insert(len(self.orderQueue)//2, order_data)

        else:#It is a normal order, we place it at the end of the queue
            self.orderQueue.append(order_data)

        return None
    
    def manage_availability(self,msg):
        """
        Extract the availability of the TIAGo robots and verify if it is either equal to "available" or "occupied".
        If that the case, it store the id and the availability of the tiago in the dictionnary dictTIAGoAvailability.
        If that is not the case, it add an error message to the list of error messages non yet published and increase the counter of messages non published.

        Exemples of messages received: "TIAGo 1 : occupied", "TIAGo 4 : available"
        """
        tiago_id = int(msg.data[6])
        tiago_availabiliy = msg.data[10:]

        if tiago_availabiliy != "available" and tiago_availabiliy != "occupied":
            self.error_occured+=1
            self.error_messages.append(msg.data + " , Problem : Unknown status.")
            return None

        self.dictTIAGoAvailable[tiago_id] = tiago_availabiliy

        return None

    def manage_position(self,msg):
        """
        Store the id and the position of the tiago in the dictionnary dictTIAGoPosition

        Exemple of message received : "Position message sent by the TIAGo platform n°4: (14.53, 12.14)
        """
        tiago_id = msg.z
        tiago_abscysse = msg.x
        tiago_ordinate = msg.y

        self.dictTIAGoPosition[tiago_id] = [tiago_abscysse,tiago_ordinate]

        return None
    
    def compute_distance(self, tiago_id):
        """
        Compute the distance between a TIAGo robot and the serving area
        """
        tiago_x = self.dictTIAGoPosition[tiago_id][0]
        tiago_y = self.dictTIAGoPosition[tiago_id][1]

        serving_area_x = self.service_area_coords[0]
        serving_area_y = self.service_area_coords[1]
        
        distance = math.sqrt((tiago_x-serving_area_x)**2+(tiago_y-serving_area_y)**2)
        return distance
    
    def assign_order(self):
        """
        Method that takes the first order of the order queue and sends it to the available robot which is closer to the serving area.
        Example of message sent: "TIAGo n°3, table : 37, dish : Gunkan"
        """
        if not self.orderQueue:
            rospy.logwarn("No orders available to assign")
            return None
        order_data = self.orderQueue[0]
        table_number, dish = order_data[0], order_data[1]

        tiago_id = 0
        distance_min = 1000  # Distance greater than the maximum possible in the restaurant

        for tiago_id_available in self.dictTIAGoAvailable.keys():
            distance = self.compute_distance(tiago_id_available)
            if distance < distance_min:
                tiago_id = tiago_id_available
                distance_min = distance

        order_msg = "TIAGo n°" + str(tiago_id) + ", table : " + str(table_number) + ", dish : " + dish
        rospy.loginfo("Order sent by the orchestration manager : " + order_msg)
        self.publisher_order.publish(order_msg)

        return None
    
    def send_error_messages(self):
        """
        While there is unpublished error messages, it publishs them and then decreases the counter of unpublished numbers and 
        remove the error message from the list of unpublished error_message
        """
        while self.error_occured != 0:
            error_msg = self.error_messages[0]
            rospy.loginfo(error_msg)
            self.publisher_order.publish(error_msg)
            del self.error_messages[0]#Remove the error message from the list
            self.error_occured -= 1



    def orchestration(self):
        """
        Method that manages the restaurant work until the ROS node "Manager" of the Manager is shutdown.
        """
        nb_turns = 0 
        t = time.time()

        while (nb_turns < 2000) and (len(self.ongoingTasks)==0):
            # Only assign an order if there are orders in the queue and available robots
            if self.orderQueue and self.dictTIAGoAvailable:
                self.assign_order()
            if time.time()-t > 1:
                t = time.time()
                nb_turns += 1
            rospy.sleep(0.1)
            


if __name__ == '__main__':
    # Remove rospy.init_node call from the global scope
    try:
        rospy.init_node("OrchestrationManager", anonymous=True)
        orchestrationManager = OrchestrationManager()
        orchestrationManager.orchestration()
    except rospy.exceptions.ROSInitException:
        print("WARNING: ROS node already initialized. Running in non-ROS mode.")
        orchestrationManager = OrchestrationManager()
        orchestrationManager.orchestration()
