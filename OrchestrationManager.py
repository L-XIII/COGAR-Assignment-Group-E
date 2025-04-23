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
        #Generation of the map of the restaurant
        self.generation_map()
        #Subsrciption to the ROS topic "orders"
        rospy.init_node("Orchestration Manager")

        #Creation of the order_TIAGo publisher
        self.publisher_order = rospy.Publisher("order_TIAGo", String, queue_size=10)

        #Creation of the publisher that will notify the staff of potential errors
        self.publisher_error = rospy.Publisher("error_messages", String, queue_size=10)


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
    
    def extraxtion_data(self, msg):
        """
        This function extract the datas of the received order 
        """
        index_order = 7 #We skip the "Table : " part of the message

        #Extraction of the table number
        index_order = 8 
        table_number = msg.data[8]

        if msg.data[9] != " ":#if the 10th character of the message is not a space character, it means that the table number has two digits
            table_number = msg.data[8:10]#We extract the table number
            index_order = 9 
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
        """
        If the dish, the table number or the priority do not correspond to legit values, it increases the number of errors to be transmitted to the staff 
        and append the corresponding erro message to the liset error_messages
        Store the orders in the queue if it is a dish with known name, table_number and priority:
        - At the beginning of the queue if it is a cleaning order (priority 2)
        - At the middle of the queue if it is a priority order (priority 1)
        - At the end of the queue if it is a normal order
        """
        rospy.loginfo("Message received by the orchetration manager : %s", msg.data)
        table_number, dish, priority, message = self.extraction_data(msg)
        
        order_data = [table_number, dish]

        if dish not in OrchestrationManager.list_dish:
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
        Store the id and the availability of the tiago in the dictionnary dictTIAGoAvailability
        """
        tiago_id = int(msg.data[6])
        tiago_availabiliy = msg.data[10:]

        self.dictTIAGoPosition[tiago_id] = tiago_availabiliy

        return None

    def manage_position(self,msg):
        """
        Store the id and the position of the tiago in the dictionnary dictTIAGoPosition
        """
        tiago_id = msg.z
        tiago_abscysse = msg.x
        tiago_ordinate = msg.y

        self.dictTIAGoPosition[tiago_id] = [tiago_abscysse,tiago_ordinate]

        return None
    
    def compute_distance(self,table_number, tiago_id):
        """
        Compute the distance between a TIAGo robot and the table where the task has to be accomplished
        """
        tiago_x = self.dictTIAGoPosition[tiago_id][0]
        tiago_y = self.dictTIAGoPosition[tiago_id][1]

        table_x = self.table_coords[table_number][0]
        table_y = self.table_coords[table_number][1]
        
        distance = math.sqrt((tiago_x-table_x)**2+(tiago_y-table_y)**2)
        return distance
    
    def assign_order(self, tiago):
        """
        Method that takes the first order of the order queue and send it to the nearest available robot
        Exemple of message sent : "TIAGo n°3, table : 37, dish : Gunkan" 
        """

        order_data = self.orderQueue[0]
        table_number, dish = order_data[0], order_data[1]

        tiago_id = 0
        distance_min = 1000 #Distance greater than the greatest distance possible to travel in the restaurant

        for tiago_id_available in self.dictTIAGoAvailable.keys():
            distance = self.compute_distance()
            if distance<distance_min:
                tiago_id = tiago_id_available
                distance_min = distance

        order_msg = "TIAGo n°" + tiago_id + ", table : " + str(table_number) + ", dish : ", dish 
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
        - We check if there is a new order received
        - If that is the case, we store the order to the orderQueue list taking into account its priority
        - We check if there is an available robot
        - In that case we give it a new order
        - We increase the counter of turns if one second has passed
        """

        nb_turns = 0 
        t = time.time()

        #While the number of turns (one turn = 1s) is inferior to 2000 and there are still ongoing tasks and tasks in the queue   
        while (nb_turns<2000) and (len(self.ongoingTasks)==0) and (len(self.orderQueue)==0): 

            #Updating the order list
            rospy.Subscriber("orders", String, self.order_storing)

            #Updating the availability of the robots and their position
            rospy.Subscriber("available", String, self.manage_availability)
            rospy.Subscriber("position", Point, self.manage_position)
     
            while self.dictTIAGoAvailable != {}:#while there is still an available robot, we give it an order
                self.assign_order()

            if time.time()-t>1:
                t = time.time()
                nb_turns+=1


if __name__ == '__main__':
    orchestrationManager = OrchestrationManager()

    orchestrationManager.orchestration()#Simulation of the restaurant