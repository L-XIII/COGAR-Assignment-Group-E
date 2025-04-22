import rospy
import random
import numpy.random as nrd

import TIAGo


class PerceptionSystem():
    """
    This class Modelize the Perception System of a TIAGo robot.
    """

    def __init__(self, tiago_platform):
        self.tiago = tiago_platform
        self.order_phase = 0
        self.dish = None

    def bernoulli_proba(self, percentage):
        """
        This function returns "True" with "percentage" percent of probability and false if not
        """
        success = nrd.binomial(1, percentage/100)# Sucess is equal to 1 in case of success and 0 in case of failure

        if success:
            return True
        else:
            return False
        
    def perception_for_navigation(self):
        """
        Use the Bernoulli probability to generate an obstacle in front of the robot (10% chance of having an obstale in front of the robot)
        """
        obstacle_detected = self.bernoulli_proba(10)
        
        return obstacle_detected

    def perception_for_grasping_and_placement(self):
        """
        Use the Bernoulli probability to determine if the perception action is succesful or not and return the success result
        """

        if self.dish == "clearing":
            #Clearing operation
            if self.order_phase == 2: 
                #Phase 2 of the clearing order : the TIAGo Robot has to take the empty plate - 99% chance to locate the plate
                plate_located = self.bernoulli_proba(99)
                return plate_located
            else:
                #Phase 4 of the clearing order : the TIAGo Robot has put down the empty plate - 100% chance to locate a free spot in the washing_area
                free_spot_located = self.bernoulli_proba(100)
                return free_spot_located
        else:
            #Serving operation
            if self.order_phase == 2: 
                #Phase 2 of the serving order : the TIAGo Robot has to take the required_plate - 80% chance to locate the plate
                plate_located = self.bernouilli_proba(99)
                return plate_located
            else:
                #Phase 4 of the serving order : the TIAGo Robot has put down the empty plate - 70% chance to locate a free spot on the table
                free_spot_located = self.bernouilli_proba(99)
                return free_spot_located
    
    def perception(self):
        """
        Choose which kind of perception operation to effectuate depending on the order and the phase the robot is in
        Return "True" if the relevant localization has been successful and "False" if not
        """
        self.order_phase = self.tiago.order_phase
        self.dish = self.tiago.dish

        if self.order_phase in [1,3,5]:
            #Movement phases, we use the perception system for navigation 
            localization = self.perception_for_navigation()
            return localization 
        
        if self.order_phase in [2,4]:
            #Grasping phase, we use the perception system for reasoning about the food placement and 
            self.perception_for_grasping_and_placement()
            return localization

        
        
    
   