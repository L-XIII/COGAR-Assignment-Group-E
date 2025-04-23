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

        print("TIAGo Robot n°%i - 'Here is your command !'", self.tiago.id)
        print("TIAGo Robot n°%i - 'Is everything alright with the food?'", self.tiago.id)

        problem_occurred = bernoulli_proba(10)
        if problem_occurred:
            self.ask_for_problem()
            problem = self.nature_of_the_problem()
            return problem
        else:
            print("TIAGo Robot n°%i - 'Itadakamisu'", self.tiago.id)#"Buon appetito" in japanese
            return None

    def ask_for_problem(self):
        """
        Ask the client for the nature of its problem
        """
        print("TIAGo Robot n°%i - 'Could you describe us the nature of your problem ?'", self.tiago.id)

    def nature_of_the_problem():
        """
        Simulate the customer response for describing the nature of its problem
        """
        nature = random.randint(1,3)
        if nature == 1:
            print("Client - 'Excuse me, this isn't the dish I ordered'")
            return "wrong_dish"
        
        if nature == 2:
            print("Client - 'Excuse me, the cultery is dirty'")
            return "dirty_cultery"
        
        if nature == 3:
            print("That is not to criticize the food ore something, no, I just a question/request...")
            return "question_or_request"
