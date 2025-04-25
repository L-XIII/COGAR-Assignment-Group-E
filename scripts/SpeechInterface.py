#!/usr/bin/env python

import random

from PerceptionSystem import bernoulli_proba

class SpeechInterface():
    """
    An instance of the class SpeechInterface is used to communicate with the client using speakers and microphones
    """
    def __init__(self):
        pass

    def serving_sentences(self):
        """
        - The robot says to the customer that his command is here
        - It ask the client if everything is fine with it
        - There is 10% chance that the client has some kind of problem 
        - If there is no problem, the method return "None", if there is one, this method returns the nature of the problem
        """

        print(f"TIAGo Robot n°{self.tiago.id} - 'Is everything alright with the food?'")

        problem_occurred = bernoulli_proba(10)
        if problem_occurred:
            print("Client - 'No, I have a problem'")
            print(f"TIAGo Robot n°{self.tiago.id} - 'Could you describe us the nature of your problem ?'")
            problem = self.nature_of_the_problem()
            return problem
        else:
            print("Client - 'Yes, everything is fine'")
            print(f"TIAGo Robot n°{self.tiago.id} - 'Buon appetito'")
            return None

    def nature_of_the_problem(self):
        """
        Simulate the customer response for describing the nature of its problem
        """
        nature = random.randint(1,3)
        if nature == 1:
            print("Client - 'Excuse me, this isn't the dish I ordered'")
            return "wrong_dish"
        
        if nature == 2:
            print("Client - 'Excuse me, clean away the empty plates'")
            return "empty_plates"
        
        if nature == 3:
            print("That is not to criticize the food ore something, no, I just a question/request...")
            return "question_or_request"
