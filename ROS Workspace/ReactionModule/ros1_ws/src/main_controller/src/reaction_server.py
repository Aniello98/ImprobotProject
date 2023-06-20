#!/usr/bin/env python3

import rospy
from main_msgs.srv import Reaction
import numpy as np
import os

REACTION_STATES = ["attack",
                    "scolding",
                    "intimidate",
                    "grudge",
                    "sharing_happiness",
                    "happy_person",
                    "satisfaction",
                    "sharing_fear",
                    "running_away",
                    "sharing_sadness",
                    "disappointment",
                    "surprise",
                    "disbelief",
                    "astonishment",
                    "none"]

# Taking into account the distance from the base link and the farest element of the robot
DISTANCE_CORRECTION = 0.5

# proxemics zones, meters of distance between robot and actor
INTIMATE = 0.6 + DISTANCE_CORRECTION
INTIMATE_NEUTRAL = 0.8 + DISTANCE_CORRECTION
NEUTRAL = 1.2 + DISTANCE_CORRECTION
NEUTRAL_NOT_INTIMATE = 1.4 + DISTANCE_CORRECTION
NOT_INTIMATE = 1.6 + DISTANCE_CORRECTION
AWAY = 2 + DISTANCE_CORRECTION


# ------------------------------------------------------------------
#                       REACTION SERVER CALLBACK
# ------------------------------------------------------------------

def handle_reaction_request(req):
    pause_after = 1 if req.reaction != "none" else 0        # pause after the action is over
    
    if req.reaction not in REACTION_STATES:
        rospy.logwarn(f">>> UNFORSEEN REACTION: {req.reaction}")
        return 0,0,0,0,0,[],0
    
    actions_before_approach = before_app_actions(req.reaction)
    actions_while_approach = while_app_actions(req.reaction)
    approach_dir, approach_dist, approach_speed = should_approach_actor(req.reaction)
    rotate_dir, rotate_speed = should_rotate(req.reaction)
    actions = additional_actions(req.reaction)
    # Bool indicating if while approaching or not, the robot should correct its angle
    correct_move = should_correct_movement(req.reaction)

    adjust_angle = should_adjust_angle(req.reaction)

    # trembling while moving
    tremble = should_tremble(req.reaction)
    
    rospy.loginfo(f">>> REACTING '{req.reaction}'")
    rospy.loginfo(f"approach: {approach_dir}, rotate: {rotate_dir}\nactions: {actions}")
    
    return approach_dir, approach_dist, approach_speed, rotate_dir, rotate_speed, actions, pause_after, correct_move, adjust_angle, tremble, actions_before_approach, actions_while_approach


# ------------------------------------------------------------------
#                           PRIVATE METHODS
# ------------------------------------------------------------------

def should_tremble(reaction:str) -> bool:
    tremble_list = ["sharing_fear"]
    return reaction in tremble_list

def should_adjust_angle(reaction:str) ->bool:
    adjust_angle_list = [   "happy_person",
                            "surprise",
                            "disbelief",
                            "sharing_happiness",
                            "sharing_fear",
                            "astonishment"]
    return reaction in adjust_angle_list

def should_correct_movement(reaction :str) -> bool:
    correct_move_dict = ["attack",
                         "scolding",
                        "intimidate",
                        "satisfaction",
                        "sharing_happiness",
                        "sharing_fear",
                        "sharing_sadness",
                        "astonishment"

    ]

    return reaction in correct_move_dict

def should_approach_contemporary(reaction :str) -> bool:

    approach_cont_dict = [
        "satisfaction",
        "surprise",
        "sharing_sadness",
        "sharing_happiness",
        "sharing_fear",
        "scolding",
        "running_away",
        "grudge"
    ]

    return reaction in approach_cont_dict

def should_approach_actor(reaction :str) -> tuple:

    default = (0, NEUTRAL, 0.5) # default values
    
    approach_dict = {
        "sharing_happiness"  : ( 1, NEUTRAL, 0.75),
        "surprise"           : ( 0, NEUTRAL , 0),
        "sharing_fear"       : ( 1, NEUTRAL, 0.6),
        "sharing_sadness"    : ( 1, NEUTRAL, 0.3),
        "attack"             : ( 1, INTIMATE , 1),
        "intimidate"         : ( 1, INTIMATE , 0.8),
        "scolding"           : ( 1, NEUTRAL, 0.65),
        "astonishment"       : ( 1,  NEUTRAL_NOT_INTIMATE, 0.75),
        "satisfaction"       : ( 1, NOT_INTIMATE , 0.5),
        "disappointment"     : (-1, NOT_INTIMATE, 0.4),
        "running_away"       : ( 0, AWAY , 0.9),
        "grudge"             : ( 0, NEUTRAL_NOT_INTIMATE, 0.5)
    }

    return approach_dict[reaction] if reaction in approach_dict else default


def should_rotate(reaction :str) -> tuple:


    rotate_dict = {"attack"             : (1, 1.5), # (towards/backwards actor, rotation speed)
                    "scolding"          : (1, 1),
                    "intimidate"        : (1, 1),
                    "grudge"            : (1, 1),
                    "sharing_happiness" : (1, 1.5),
                    "happy_person"      : (1, 1.5),
                    "satisfaction"      : (1, 1),
                    "sharing_fear"      : (1, 1.5),
                    "running_away"      : (1, 1),
                    "sharing_sadness"   : (1, 1),
                    "disappointment"    : (-1, 1),
                    "surprise"          : (1, 1),
                    "disbelief"         : (1, 1),
                    "astonishment"      : (1, 1.5),
                    "none"              : (0, 0)
    }
    
    return rotate_dict[reaction]

def before_app_actions(reaction :str) -> list:
    actions = { # reaction -> actions
        "sharing_happiness":    [],
        "happy_person":         [],
        "satisfaction":         [],
        "surprise":             [],
        "disappointment":       [],
        "sharing_fear":         ["sharing_fear_before1", "sharing_fear_before2" ],
        "astonishment":         [],
        "running_away":         [],
        "sharing_sadness":      [],
        "attack":               ["attack_before_app"],
        "scolding":             ["scolding_before_app"], 
        "intimidate":           [],
        "grudge":               [],
        "disbelief":            [],
        "none":                 [],
    }

    return actions[reaction]

#actions during approaching phase
def while_app_actions(reaction :str) -> list:
    actions = { # reaction -> actions
        "sharing_happiness":    ["sharing_happiness_while_approach"],
        "happy_person":         [],
        "satisfaction":         ["satisfaction_while_app"],
        "surprise":             [],
        "disappointment":       [],
        "sharing_fear":         ["sharing_fear_while_app"],
        "astonishment":         [],
        "running_away":         [],
        "sharing_sadness":      ["sharing_sadness_while_app"],
        "attack":               [],
        "scolding":             ["scolding_while_app"], 
        "intimidate":           [],
        "grudge":               [],
        "disbelief":            [],
        "none":                 [],
    }

    return actions[reaction]    

# actions after approaching phase
def additional_actions(reaction :str) -> list:

    # NEW: in order to allow parallel action execution
    actions = { # reaction -> actions
        "sharing_happiness":    ["sharing_happiness_actions"],
        "happy_person":         ["happy_person_actions"],
        "satisfaction":         [],
        "surprise":             ["surprise_actions"],
        "disappointment":       ["disappointment_actions"],
        "sharing_fear":         [],
        "astonishment":         ["astonishment_actions1"],
        "running_away":         ["running_away_actions1", "running_away_actions2"],
        "sharing_sadness":      [],
        "attack":               ["attack_actions"],
        "scolding":             [], 
        "intimidate":           ["intimidate_actions1", "move_back_intimidate", "intimidate_actions2"],
        "grudge":               ["grudge_actions1", "grudge_actions2"],
        "disbelief":            ["disbelief_actions1", "disbelief_actions2", "disbelief_actions3"],
        "none":                 [],
    }

    return actions[reaction]

# ------------------------------------------------------------------
#                               MAIN
# ------------------------------------------------------------------

if __name__ == '__main__':

    # Initialize Reaction Server node
    rospy.init_node("reaction_server")
    rospy.loginfo("Reaction server node created")

    # Start Reaction Service
    service = rospy.Service("/get_reaction", Reaction, handle_reaction_request)
    rospy.loginfo("Reaction Service server has been started")

    # Loop waiting for requests
    rospy.spin()
    
