#!/usr/bin/env python3

import numpy.random as nrd


def bernoulli_proba(percentage):
    """
    Generate a boolean outcome based on a specified probability percentage.
    
    Simulates a Bernoulli trial where success occurs with the given probability.
    
    Args:
        percentage (int): Probability of success as a percentage (0-100)
        
    Returns:
        bool: True with probability of percentage/100, False otherwise
    """
    success = nrd.binomial(1, percentage/100)  # Success is 1, failure is 0
    return bool(success)


class PerceptionSystem:
    """
    PerceptionSystem handles all perception-related tasks for a TIAGo robot.
    
    This class is responsible for:
    - Detecting objects and obstacles during navigation
    - Identifying dishes and free spots for grasping and placement
    - Verifying successful grasping and placement operations
    - Processing sensory data from cameras and other sensors
    
    The system simulates real-world perception with probability-based outcomes
    to account for sensor noise and environmental uncertainties.
    
    Attributes:
        tiago (TIAGo): Reference to the parent TIAGo robot platform
        order_phase (int): Current phase of the order being executed
        dish (str): Current dish being handled
    """

    def __init__(self, tiago_platform):
        """
        Initialize the perception system for a TIAGo robot.
        
        Args:
            tiago_platform (TIAGo): The TIAGo robot this perception system belongs to
        """
        self.tiago = tiago_platform
        self.order_phase = 0
        self.dish = None

    def perception_for_navigation(self):
        """
        Detect obstacles and clear paths during robot navigation.
        
        Simulates the robot's ability to perceive its environment during movement.
        Uses probability to determine if the path is clear (99% chance).
        
        Returns:
            bool: True if path is clear, False if obstacle detected
        """
        obstacle_detected = bernoulli_proba(99)
        return obstacle_detected

    def perception_for_grasping_and_placement(self):
        """
        Detect objects for grasping or identify suitable placement locations.
        
        This method handles two distinct perception tasks:
        1. During grasping: Locating the correct dish to be picked up
        2. During placement: Finding a suitable free spot on the table
        
        The behavior differs based on the current order phase and dish type.
        
        Returns:
            bool: True if detection successful, False otherwise
        """
        if self.dish == "clearing":
            # Clearing operation: locate empty plates with 99% success rate
            plate_located = bernoulli_proba(99)
            return plate_located
        else:
            # Serving operation
            if self.order_phase == 2:
                # Phase 2: Locate the dish to be picked up (99% success rate)
                plate_located = bernoulli_proba(99)
                return plate_located
            else:
                # Phase 4: Locate free spot for dish placement (99% success rate)
                free_spot_located = bernoulli_proba(99)
                return free_spot_located
    
    def perception(self):
        """
        Coordinate perception operations based on the current order phase.
        
        This is the main entry point that selects the appropriate perception
        function based on the robot's current state and order phase:
        - Phases 1, 3, 5: Uses navigation perception (obstacle detection)
        - Phases 2, 4: Uses grasping/placement perception
        
        Returns:
            bool: True if perception successful, False otherwise
        """
        # Update current state from TIAGo robot
        self.order_phase = self.tiago.order_phase
        self.dish = self.tiago.dish

        if self.order_phase in [1, 3, 5]:
            # Movement phases: use perception for navigation
            localization = self.perception_for_navigation()
            return localization 
        
        if self.order_phase in [2, 4]:
            # Grasping or placement phases: use perception for object manipulation
            localization = self.perception_for_grasping_and_placement()
            return localization
        
        # Default return for other phases
        return True
    
    def verification_of_grasping_and_placement(self, operation):
        """
        Verify the success of grasping or placement operations.
        
        This method is used by the OrderVerificationSystem to confirm that
        dishes were correctly grasped or placed. It simulates sensor feedback
        to verify successful task completion.
        
        Args:
            operation (str): Type of operation to verify - "placement" or "grasping"
            
        Returns:
            bool or str: True if operation successful, False if failed,
                        "Target unknown" if operation type is invalid
        """
        if operation == "placement":
            # Verify successful placement (99% success rate)
            placement_successful = bernoulli_proba(99)
            return placement_successful
            
        elif operation == "grasping":
            # Verify successful grasping (99% success rate) 
            grasping_successful = bernoulli_proba(99)
            return grasping_successful
        else:
            # Invalid operation type
            return "Target unknown"
