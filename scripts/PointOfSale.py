#!/usr/bin/env python

import rospy
import random
import time
from std_msgs.msg import String



class POS():
    list_dishes = ["Nare","Oshi","Nigri","Gunkan","Maki","Futo","Temaki","Inari","Temari","Sashimi","Uramaki"]
    nb_dishes = len(list_dishes)

    def __init__(self):
        self.nb_tables = 60
        
        self.publisher = rospy.Publisher('orders', String, queue_size=10)#This line creates a publisher that will publish String messages to the 'orders' topic
        #A queue size of 10 means that up to 10 messages can be queued before the oldest messages 
        # are discarded if the subscriber is not processing messages fast enough.
        rospy.init_node('POS', anonymous=True) #ROS node of the POS, anonymous = True so ROS can add a random number after the name of the node "POS" 
        #(each node has to have a different name in ROS)
        self.rate = rospy.Rate(1) # 1Hz
    
    def order_emission(self, nb_loops):
        """
        Randomly generates emissions of orders, roughly 3 order every 200 turns
        Format of a message : 
            - Number of the table (between 1 and 60)
            - Name of the dish
            - Number of the simulation loop
            - Priority (0 for no priority, 1 for priority)
        
        Example : "Table : 42, dish : Gunkan, loop n°1214, priority : 1"

        """
        emission_possible = random.randint(1,200)
        if emission_possible<=3:
            dish = random.choice(POS.list_dishes) #Choose a random dish
            table_number = random.randint(1,60) #Choose a random table
            priority = random.randint(0,1) #Choose a random priority
            order_msg = "Table : " + str(table_number) + ", dish : " + dish + ", loop n°" + str(nb_loops) + ", priority : " + str(priority)
            rospy.loginfo("Message sent by POS : " + order_msg)
            self.publisher.publish(order_msg)
        self.rate.sleep()

        
        
    
    def shutdown(self):
        rospy.signal_shutdown("End of the simulation for this POS")

if __name__ == '__main__':
    pos = POS()
    nb_loops = 0
    while nb_loops<2000:# 2000 turns of loop, with a 1 Hz frequency
        pos.order_emission(nb_loops)
        nb_loops+=1
    pos.shutdown()
