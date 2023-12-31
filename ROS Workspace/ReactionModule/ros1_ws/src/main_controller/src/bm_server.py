#!/usr/bin/env python3

import json
import os
import numpy as np
import random

import rospy
from dynamic_reconfigure.server import Server
from main_controller.cfg import bm_serverConfig
from main_msgs.srv import WorldModel, ActorInfoS


class BehaviouralModel:
    def __init__(self):
        # Initialize Behaviour Simulation server node
        rospy.init_node("bm_server")
        rospy.loginfo("Behavioural Model server node created")

        # Load personality and initial emotional state of the robot
        self.personality = rospy.get_param("/personality")
        self.current_state = rospy.get_param("/emotional_state")

        #Load reaction model type
        self.reaction_model = rospy.get_param("/reaction_model")

        #load the reaction model
        if self.reaction_model == "state_machine":
            # Load robot emotions and reactions transitions (state machines)
            rospy.loginfo(f"loading {self.personality} personality state machine")
            self.load_state_machine(f"../resources/personalities/{self.personality}.json")
        
            # Load robot reactions table
            self.load_reactions("../resources/csv/Reactions.csv")

        elif self.reaction_model == "cutted_tree_random":
            rospy.loginfo("loading cutted tree random model")
            self.load_emotional_reactions(f"../resources/csv/Random_Emotional_Reactions.csv")
            # Load actions per emotion
            self.load_actions_per_emotion(f"../resources/actions/actions_per_emotion.json")

        # Setup dynamic reconfigure for personality and initial emotion
        srv = Server(bm_serverConfig, self.reconfigure_callback)

        # Connect to Actor Info Service
        rospy.loginfo(">>> WAITING FOR ACTOR INFO SERVICE")
        rospy.wait_for_service("/get_actor_info")
        try:
            self.get_actor_info = rospy.ServiceProxy("/get_actor_info", ActorInfoS)
            rospy.loginfo("Actor Info Service Client created.")
        except rospy.ServiceException as err:
            rospy.logwarn("Service failed: " + str(err))

        # Create Get Behavior Service
        service = rospy.Service("/get_behavior", WorldModel, self.handle_bm)
        rospy.loginfo("Behavioral Model Service server has been started")

        # Loop waiting for requests
        rospy.spin()
        
    # ------------------------------------------------------------------
    #                     CUTTED RANDOM TREE HANDLING
    # ------------------------------------------------------------------

    def load_emotional_reactions(self, filename):
        reactions = np.genfromtxt(filename, delimiter=',', dtype=str)   # read csv into numpy array

        self.emo_states = reactions[0,1:].tolist() # first row, excluding first column
        self.actor_actions = reactions[1:,0].tolist() # first column, excluding first row
        self.emo_reactions = reactions[1:,1:].tolist() # save as list, excluding first row and column

    def load_actions_per_emotion(self, filename):
        with open(filename, 'r') as f:
            self.actions_per_emotion = json.load(f)

    # ------------------------------------------------------------------
    #                     STATE MACHINE HANDLING
    # ------------------------------------------------------------------
        
    def load_state_machine(self, filename):
        with open(filename, 'r') as f:
            self.state_machine = json.load(f)
 
 
    def load_reactions(self, filename):
        reactions = np.genfromtxt(filename, delimiter=',', dtype=str)   # read csv into numpy array

        self.emo_states = reactions[0,1:].tolist() # first row, excluding first column
        self.actor_actions = reactions[1:,0].tolist() # first column, excluding first row
        self.reactions = reactions[1:,1:].tolist() # save as list, excluding first row and column

    
    def next_state(self, current_action):
        all_transitions = self.state_machine[self.current_state]
        action_transitions = [(state,probability) for action,state,probability in all_transitions if current_action.lower() == action.lower()]
        
        if action_transitions:
            states, probability_vector = zip(*action_transitions)   # unzip the list of tuples
            try:
                next_state = np.random.choice(states, p=probability_vector).lower()     # perform non-deterministic weighted choice
            except ValueError:
                next_state = self.current_state     # probabilities do not sum to 1
        else:   
            next_state = self.current_state     # action_transitions is empty, remain in current state
 
        rospy.loginfo(f"transition: {self.current_state}, {current_action} -> {next_state}")
        
        if next_state in self.state_machine.keys():
            self.current_state = next_state
        else:
            rospy.logerror(f"state {next_state} not found, staying in {self.current_state}")


    # ------------------------------------------------------------------
    #                               CALLBACKS
    # ------------------------------------------------------------------

    def handle_bm(self, req):
        # Get Actor's Scenic Action from Actor Info Server
        rospy.loginfo(">>> WAITING FOR ACTOR INFO SERVICE RESPONSE")
        actor_info = self.get_actor_info()
        rospy.loginfo("actor infos:\n%s", actor_info)

        if self.reaction_model == "state_machine" :
            # Calculate new Emotional State
            self.next_state(actor_info.scenicAction)

            # Calculate Robot's Reaction  
            i = self.emo_states.index(self.current_state)
            j = self.actor_actions.index(actor_info.scenicAction)
            current_reaction = self.reactions[j][i]
        
        elif self.reaction_model == "cutted_tree_random" :
            #take all the plausible emotional states
            actor_action_idx = self.actor_actions.index(actor_info.scenicAction)
            emo_reactions = self.emo_reactions[actor_action_idx]
            plausible_emo_states_idxs = [i for i in range(len(emo_reactions)) if emo_reactions[i] == "1" ]
            plausible_states = [self.emo_states[i] for i in plausible_emo_states_idxs]
            #select a random emotional state
            selected_state = random.choice(plausible_states)
            #get all the possible actions associated to the selected_state
            plausible_actions = self.actions_per_emotion[selected_state]
            #select a random action
            current_reaction = random.choice(plausible_actions)


        rospy.loginfo("robot's reaction: %s", current_reaction)

        

        # Communicate Robot's Reaction to Main Controller
        return [actor_info.actorPosX, actor_info.actorPosY, actor_info.actorOrientation, actor_info.scenicAction, self.current_state, current_reaction, actor_info.endScene]


    def reconfigure_callback(self, config, level):
        # self.load_state_machine(f"../resources/personalities/{config.personality}.json")
        self.current_state = config.emotional_state
        rospy.loginfo(f"emotional state is {self.current_state}")
        
        return config


if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    rospy.loginfo(">>> CURRENT WORKING DIRECTORY: " + os.getcwd())

    try:
        BehaviouralModel()
    except rospy.ROSInterruptException:
        pass
