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
    
    def verify_served_order(self):
        """
        - Verify that the served plate corresponds correctly with the data recieved using the perception system of the TIAGo platform.
        - Return "served order error" if the perception system detects an error and "None" if not 
        """
        success = self.tiago.perception_system("Verified")
        if not success:
            return "served order error"
        return None

   
    
    def verify_delivery_client(self):
        """
        - Verify with the client that the dish has been correctly served and there is no problem
        - Returns "None" if there is no problem and the nature of the problem if there is one and notify the manager.
        """
        problem = self.speech_interface.verify_delivery_client()
        if problem == None:
            return None
        else:
            return problem
