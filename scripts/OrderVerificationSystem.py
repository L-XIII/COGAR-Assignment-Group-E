#!/usr/bin/env python3

try:
    from scripts.SpeechInterface import SpeechInterface
except ImportError:
    from SpeechInterface import SpeechInterface

class OrderVerificationSystem:
    """
    The OrderVerificationSystem verifies the correctness of order processing and delivery.
    
    This class handles:
    - Verifying that the correct dish was grasped
    - Validating successful dish placement at the customer's table
    - Interfacing with customers to confirm successful delivery
    - Gathering feedback about any issues with the order
    
    It serves as a quality control system for the restaurant's service delivery process,
    ensuring that orders are correctly fulfilled by the TIAGo robots.
    
    Attributes:
        tiago (TIAGo): The TIAGo robot platform this system is part of
        order_phase (int): Current phase of order processing
        dish (str): Current dish being handled
        speech_interface (SpeechInterface): Interface for verbal customer interaction
    """
    _instance = None

    def __new__(cls, tiago_platform=None):
        """
        Create a singleton instance of OrderVerificationSystem.
        
        Args:
            tiago_platform: Reference to the TIAGo robot instance
            
        Returns:
            OrderVerificationSystem: The singleton instance
        """
        if cls._instance is None:
            cls._instance = super(OrderVerificationSystem, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, tiago_platform):
        """
        Initialize the order verification system with a TIAGo robot platform.
        
        Args:
            tiago_platform (TIAGo): The TIAGo robot this verification system belongs to
        """
        self.tiago = tiago_platform
        self.order_phase = 0
        self.dish = None
        self.speech_interface = SpeechInterface()
        self.speech_interface.tiago = tiago_platform  # Associate speech interface with this TIAGo
    
    def verify_served_order(self, dish):
        """
        Verify that the robot has grasped the correct dish at the serving area.
        
        Uses the robot's perception system to confirm that:
        1. The robot successfully grasped a dish
        2. The grasped dish matches the intended order
        
        Args:
            dish (str): The dish that should have been grasped
            
        Returns:
            str or None: Returns "grasping_problem" if there was an issue with grasping 
                        or the wrong dish was grasped, None if successful
        """
        # Use the perception system to verify successful grasping
        success = self.tiago.perception_system.verification_of_grasping_and_placement("grasping")
        
        # Return error string if unsuccessful, None if successful
        if not success:
            return "grasping_problem"
        return None

    def verify_delivery_client(self):
        """
        Verify successful order delivery and customer satisfaction.
        
        This method performs two verification steps:
        1. Physical verification: Checks if the dish was placed correctly using the perception system
        2. Customer verification: Interacts with the customer to confirm satisfaction and identify issues
        
        Returns:
            list: A list containing two elements:
                - First element (bool): True if there was a placement problem, False otherwise
                - Second element (str or None): Type of customer problem if any, None if no issues
        """
        # Verify physical placement of the dish
        success_placement = self.tiago.perception_system.verification_of_grasping_and_placement("placement")
        
        # Interact with customer to verify satisfaction
        problem_client = self.speech_interface.serving_sentences()
        
        # Compile results into a potential problems list
        potential_problems = []
        potential_problems.append(not success_placement)  # True means there IS a problem
        potential_problems.append(problem_client)  # None means no problem, otherwise contains problem type
        
        return potential_problems
