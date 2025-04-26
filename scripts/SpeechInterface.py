#!/usr/bin/env python3

import random

try:
    from scripts.PerceptionSystem import bernoulli_proba
except ImportError:
    from PerceptionSystem import bernoulli_proba


class SpeechInterface:
    """
    SpeechInterface enables verbal communication between TIAGo robots and customers.
    
    This class is responsible for:
    - Simulating speech interactions with restaurant customers
    - Handling customer queries and complaints
    - Providing appropriate robot responses based on the situation
    - Detecting and classifying customer problems
    
    It simulates the speech recognition and generation capabilities that would
    be present in a real robot system, allowing the robot to converse with
    customers during service delivery.
    
    Attributes:
        tiago (TIAGo): Reference to the parent TIAGo robot platform
    """

    def __init__(self):
        """
        Initialize the speech interface.
        
        The TIAGo robot reference is expected to be set after initialization
        via direct attribute assignment.
        """
        # tiago reference will be set externally after initialization
        pass

    def serving_sentences(self):
        """
        Conduct a serving interaction dialog with the customer.
        
        This method simulates the robot:
        1. Announcing that the ordered dish has arrived
        2. Asking if everything is satisfactory 
        3. Handling the customer's response
        
        The interaction has a 10% chance to result in a problem reported by the customer.
        
        Returns:
            str or None: Returns problem type if a problem is reported:
                - "wrong_dish": Customer received incorrect dish
                - "empty_plates": Request to clear empty plates
                - "question_or_request": Customer has a question or request
                None if everything is satisfactory
        """
        print(f"TIAGo Robot n°{self.tiago.id} - 'Is everything alright with the food?'")

        problem_occurred = bernoulli_proba(10)  # 10% chance of a customer problem
        if problem_occurred:
            print("Client - 'No, I have a problem'")
            print(f"TIAGo Robot n°{self.tiago.id} - 'Could you describe us the nature of your problem?'")
            problem = self.nature_of_the_problem()
            return problem
        else:
            print("Client - 'Yes, everything is fine'")
            print(f"TIAGo Robot n°{self.tiago.id} - 'Buon appetito'")
            return None

    def nature_of_the_problem(self):
        """
        Determine the specific nature of a customer's problem.
        
        This method simulates the customer verbally describing their issue
        and the robot's speech recognition system classifying the problem.
        
        The problem is randomly selected from three possible types with equal probability:
        1. Wrong dish delivered (33.3%)
        2. Request to clear empty plates (33.3%)
        3. General question or request (33.3%)
        
        Returns:
            str: Type of problem identified:
                - "wrong_dish": Customer received incorrect dish
                - "empty_plates": Request to clear empty plates
                - "question_or_request": Customer has a question or request
        """
        nature = random.randint(1, 3)
        if nature == 1:
            print("Client - 'Excuse me, this isn't the dish I ordered'")
            return "wrong_dish"
        
        if nature == 2:
            print("Client - 'Excuse me, clean away the empty plates'")
            return "empty_plates"
        
        if nature == 3:
            print("Client - 'That is not to criticize the food or something, no, I just have a question/request...'")
            return "question_or_request"
