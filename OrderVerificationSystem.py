import SpeechInterface

class OrderVerificationSystem():        
    """"
    An instance of OrderVerificationSystem enables the TIAGo robot which it is part of to check if the orders have been well executed
    It takes as parameters:
    - tiago : the TIAGo robot the instance is part of
    - speech_interface : the speech interface of the robot
    """

    def __new__(cls): 
        """
        Check if there is already an instance of the OrderVerificationSystem class, if there isn't, create it
        """
        if not hasattr(cls, 'instance'):
            cls._instance = super(OrderVerificationSystem, cls).__new__(cls)
            cls._instance.__init__()
        return cls._instance

    def __init__(self, tiago):
       self.tiago = tiago
       self.speech_interface = SpeechInterface()
       pass
    
    def verify_grasping(self):
        """
        - Verify that the targeted plate has been correctly grasped using the perception system of the TIAGo platform.
        - Return "grasping error" if the perception system detects an error and "None" if not 
        """
        success = self.tiago.perception_system("grasping")
        if not success:
            return "grasping error"
        return None

    def verify_placement(self):
        """
        - Verify that the plate has been placed correctly using the perception system of the TIAGo platform
        - Return "placement error" if the perception system detects an error and "None" if not 
        """
        success = self.tiago.perception_system("placement")
        if not success:
            return "placement error"
        return None
    
    def verify_delivery_client(self):
        """
        - Verify with the client that the dish has been correctly served and there is no problem
        - Returns "None" if there is no problem and the nature of the problem if there is one
        """
        problem = self.speech_interface.verify_delivery_client()
        if problem == None:
            return None
        else:
            return problem
        
