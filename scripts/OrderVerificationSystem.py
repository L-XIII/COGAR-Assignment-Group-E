#!/usr/bin/env python

from SpeechInterface import SpeechInterface

class OrderVerificationSystem:
    """
    An instance of OrderVerificationSystem enables the TIAGo robot which it is part of to check if the orders have been well executed
    It takes as parameters:
    - tiago : the TIAGo robot the instance is part of
    - speech_interface : the speech interface of the robot
    """

    def __init__(self, tiago_platform):
        self.tiago = tiago_platform
        self.order_phase = 0
        self.dish = None
        self.speech_interface = SpeechInterface()
    
    def verify_served_order(self, dish):
        """
        - Verify at the serving area that the plate it grasped corresponds to the dish it needed to using the perception system of the TIAGo platform.
        - Return "grasping_problem" if the perception system detects that the grasped dish is not the good one or if the robot failed to grasp the dish and "None" if not 
        """
        success = self.tiago.perception_system.verification_of_grasping_and_placement("grasping")
        if not success:
            return "grasping_problem"
        return None

   
    
    def verify_delivery_client(self):
        """
        - Verify that the plate has been correctly placed
        - Verify with the client that the dish has been correctly served and there is no problem
        - Return the list of potential problems whose first term regards the problems with the placement ("True" if there is a problem, "False" if not)
        and its second terms regards the problems with the client ("None" if there is no problem and the nature of the problem if there is one) 
        """
        success_placement = self.tiago.perception_system.verification_of_grasping_and_placement("placement")
        problem_client = self.speech_interface.verify_delivery_client()
        potential_problems = []
        potential_problems.append(not success_placement)
        potential_problems.append(problem_client)

        return potential_problems
