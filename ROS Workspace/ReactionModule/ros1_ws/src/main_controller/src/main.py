#!/usr/bin/env python3
# license removed for brevity
from collections import namedtuple
import json
from datetime import datetime
import os
from math import atan2, pi
from threading import Thread, Lock, Condition
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.server import Server
from main_controller.cfg import main_controllerConfig
from main_msgs.srv import WorldModel, Reaction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Float32MultiArray, Int8, Int8MultiArray, Empty
from triskarone_msgs.msg import *
from utils.functions import compute_orient

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

POSITIVE_REACTIONS = ["sharing_happiness",
                    "happy_person",
                    "satisfaction",
                    "surprise",
                    "astonishment"]

NEGATIVE_REACTIONS = ["attack",
                    "scolding",
                    "intimidate",
                    "grudge",
                    "sharing_fear",
                    "running_away",
                    "sharing_sadness",
                    "disappointment",
                    "disbelief"]

SX_ROTATION_ERROR = 0.3
DX_ROTATION_ERROR = 0.3

def read_float(input_message :str) -> float:
    """read a float from stdin"""
    while True:
        try:
            return float(input(input_message))
        except Exception:   # if the user inserts literals or empty string, ask again
            continue

class MainController:
    def __init__(self):
        rospy.loginfo(">>> START INIT")

        # init node
        rospy.init_node('main_controller', anonymous=True)

        # initialize dynamic reconfigure
        srv = Server(main_controllerConfig, self.reconfigure_callback)

        # get parameters
        rospy.loginfo(">>> GET PARAMETERS")
        self.script_path = rospy.get_param("script_path")
        self.move_base_recovery = rospy.get_param("move_base_recovery")
        self.section_number = rospy.get_param("section_to_start")
        self.MAP_SIZE = rospy.get_param("map_size")
        self.is_debug = rospy.get_param("is_debug") # in debug mode skip action checks
        self.scene_length = rospy.get_param("max_scene_duration")
        # get initial position
        self.position = [rospy.get_param("initial_x"), rospy.get_param("initial_y")] # position (x,y)
        
        # NEW dictionary for the smart checker
        # TODO: handle audio_player, and move_base if needed.
        self.actions_status = {
            # "action"                : [state, last_update_time]  
            "speak"           : [True, 0], 
            "move_base"       : [True, 0],
            "move_eyes"       : [True, 0],
            "move_body"       : [True, 0],
            "manual_move"     : [True, 0],
            "tremble"         : [True, 0]
            }
        
        # world model infos and routine parameter
        self.wm = None
        self.ready_to_react = False

        # A simple resource to lock on, if needed
        self._lock = Lock()
        self._condition = Condition()

        self._speak_lock = Lock()
        self._move_base_lock = Lock()
        self._move_eyes_lock = Lock()
        self._move_body_lock = Lock()
        self._manual_move_lock = Lock()
        self._tremble_lock = Lock()

        # initialize subscribers
        rospy.loginfo(">>> INITIALIZE SUBSCRIBERS")
        rospy.Subscriber("next_section", Bool, self.next_section_callback)
        rospy.Subscriber("move_base_recovery", Int8, self.recovery_move_base_callback)
        rospy.Subscriber("body_status", Bool, self.update_body_status)
        rospy.Subscriber("eyes_status", Bool, self.update_eyes_status)
        rospy.Subscriber("manual_move_status", Bool, self.update_manual_move_status)
        rospy.Subscriber("tremble_status", Bool, self.tremble_status)

        # initialize publishers
        self.is_moving_publisher = rospy.Publisher("/is_moving", Bool, queue_size=10)
        self.rst_ai_publisher = rospy.Publisher("/reset_ai_server", Empty, queue_size=1)

        # initialize flags

        # flags for next section
        self.enable_after_command = True
        self.trigger_ok = False

        # flags for move_base recovery
        self.move_base_failure = False
        self.up_pressed = False
        self.down_pressed = False

        # action_lib_clients
        rospy.loginfo(">>> INITIALIZE ACTIONLIB")
        self.speech_client = actionlib.SimpleActionClient('speech_monitor', triskarone_msgs.msg.speech_monitorAction)
        self.audio_client = actionlib.SimpleActionClient('audio_player_actionlib', triskarone_msgs.msg.play_audioAction)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.eyes_manager_client = actionlib.SimpleActionClient('eyes_manager', triskarone_msgs.msg.move_eyesAction)
        self.body_manager_client = actionlib.SimpleActionClient('body_manager', triskarone_msgs.msg.move_bodyAction)
        self.cmd_vel_client=actionlib.SimpleActionClient('move', triskarone_msgs.msg.manual_move_baseAction)
        self.cmd_vel_client_tremble=actionlib.SimpleActionClient('tremble', triskarone_msgs.msg.manual_move_trembleAction)

        # wait for servers to start
        rospy.loginfo(">>> WAIT FOR TRISKARONE SERVERS")
        self.speech_client.wait_for_server()
        self.audio_client.wait_for_server()
        self.eyes_manager_client.wait_for_server()
        self.body_manager_client.wait_for_server()

        # Connect to Behavior Model server
        rospy.loginfo(">>> WAITING FOR BEHAVIOR MODEL SERVICE")
        rospy.wait_for_service("/get_behavior")
        try:
            self.get_behavior = rospy.ServiceProxy("/get_behavior", WorldModel)
            rospy.loginfo("Get Behavior Model Client created.")
        except rospy.ServiceException as err:
            rospy.logwarn("Service failed: " + str(err))
        
        # Connect to Reaction server
        rospy.loginfo(">>> WAITING FOR REACTION SERVICE")
        rospy.wait_for_service("/get_reaction")
        try:
            self.get_reaction = rospy.ServiceProxy("/get_reaction", Reaction)
            rospy.loginfo("Reaction Client created.")
        except rospy.ServiceException as err:
            rospy.logwarn("Service failed: " + str(err))

        # wait for 2 second before start
        rospy.loginfo(">>> READY TO START")
        r = rospy.Rate(0.5)
        r.sleep()

        # run
        rospy.loginfo(">>> START RUN")
        self.run()

    #------------------------- SCRIPT HANDLING -------------------------

    def run(self):
        # this is the structure which allows the robot to follow a script of predefined actions  
        f = open(self.script_path)
        data = json.load(f)
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Starting section number %d", self.section_number)
                trigger, trigger_data = self.read_trigger(data, self.section_number)
            except KeyError:
                rospy.loginfo("The show is finished, bye!")
                rospy.signal_shutdown("The show is finished")
            self.wait_for_trigger(trigger, trigger_data)
            actions = self.read_actions(data, self.section_number)
            self.handle_actions(actions)
            self.check_actions_status(actions)
            self.section_number = self.section_number + 1

    def update_body_status(self, msg):
        status = msg.data
        self._move_body_lock.acquire()
        rospy.loginfo(f"updating is_move_body_done to {status}")
        
        self.actions_status["move_body"][0]=status
        t = time.localtime()
        current_time = int(time.strftime("%H%M%S", t))
        self.actions_status["move_body"][1]=current_time
        self._move_body_lock.release()

    def update_eyes_status(self, msg):
        status = msg.data
        self._move_eyes_lock.acquire()
        rospy.loginfo(f"updating is_move_eyes_done to {status}")
        
        self.actions_status["move_eyes"][0]=status
        t = time.localtime()
        current_time = int(time.strftime("%H%M%S", t))
        self.actions_status["move_eyes"][1]=current_time
        self._move_eyes_lock.release()

    def update_manual_move_status(self, msg):
        status = msg.data
        self._manual_move_lock.acquire()
        rospy.loginfo(f"updating is_manual_move_done to {status}")
        
        self.actions_status["manual_move"][0]=status
        t = time.localtime()
        current_time = int(time.strftime("%H%M%S", t))
        self.actions_status["manual_move"][1]=current_time
        self._manual_move_lock.release()

    def tremble_status(self, msg):
        status = msg.data
        self._tremble_lock.acquire()
        self.actions_status["tremble"][0]=status
        t = time.localtime()
        current_time = int(time.strftime("%H%M%S", t))
        self.actions_status["tremble"][1]=current_time
        self._tremble_lock.release()

    def reconfigure_callback(self, config, level):
        # rospy.loginfo("section_number set to %d", config.section_number)
        # self.section_number = config.section_number
        self.MAP_SIZE = config.map_size
        self.scene_length = config.max_scene_duration
        # rospy.loginfo(f"reconfigure param OK")
        return config

    def next_section_callback(self, data):
        if self.enable_after_command:
            self.trigger_ok = True
            self.enable_after_command = False
            rospy.loginfo("Go to next session")

    def recovery_move_base_callback(self, data):
        if self.move_base_recovery:
            if data.data == 1:
                self.up_pressed = True
                rospy.loginfo("up pressed")
            if data.data == 2:
                rospy.loginfo("down pressed")
                self.down_pressed = True
            self.move_base_recovery = False

    def read_trigger(self, data, section_number):
        section = 'section' + str(section_number)
        trigger = data[section]['trigger']
        trigger_data = data[section]['trigger_data']
        return trigger, trigger_data

    def read_actions(self, data, section_number):
        section = 'section' + str(section_number)
        actions = data[section]['actions']
        return actions

    def wait_for_trigger(self, trigger, trigger_data):
        if trigger == "after_speech":
            goal = triskarone_msgs.msg.speech_monitorGoal(wait_time=trigger_data)
            rospy.loginfo("Waiting for others to speak")
            self.speech_client.send_goal(goal)
            self.speech_client.wait_for_result()
            return
        if trigger == "after_precedent":
            return
        if trigger == "after_command":
            self.enable_after_command = True;
            self.trigger_ok = False
            rospy.loginfo("Waiting for joystick command")
            while not self.trigger_ok:
                continue
            return

    #------------------------- ATOMIC ACTIONS -------------------------

    def move_base(self, array):
        """array -> [position_x, position_y, orientation_z, orientation_w]"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = array[0]
        goal.target_pose.pose.position.y = array[1]
        goal.target_pose.pose.orientation.z = array[2]
        goal.target_pose.pose.orientation.w = array[3]
        self.move_base_client.send_goal(goal)
        rospy.loginfo("Moving to position: x:%f y:%f z:%f w:%f", array[0], array[1], array[2], array[3])

    def move_eyes(self, array):
        """array -> [pause, move_type, speed]"""
        rospy.loginfo(">>> MOVE EYES: %s", array)
        data_to_send = Int8MultiArray()
        data_to_send.data = array
        goal = triskarone_msgs.msg.move_eyesGoal(goal=data_to_send)
        self.eyes_manager_client.send_goal(goal)            

    def move_body(self, array):
        """array -> [pause, move_type, speed]"""
        #rospy.loginfo(">>> MOVE BODY: %s", array)
        data_to_send = Int8MultiArray()
        data_to_send.data = array
        goal = triskarone_msgs.msg.move_bodyGoal(goal=data_to_send)
        self.body_manager_client.send_goal(goal)

    def tremble(self, array):
        '''array --> [duration, pause_after, intensity]'''
        #send message to cmd_vel_manager
        lock = Lock()
        lock.acquire()
        data_to_send = Float32MultiArray()
        data_to_send.data = array
        msg = triskarone_msgs.msg.manual_move_trembleGoal(data_to_send)
        self.cmd_vel_client_tremble.send_goal(msg)
        lock.release()


    def manual_move(self, array, correct = False, skip_checks=False, ar_goal = 0, adjust_angle = False, tremble = False):
        """array -> [distance_x, distance_y, distance_theta, vel_x, vel_y, vel_theta, waiting_time]"""
        # if self.try_smooth_moves and not jump_smooth_check:
        #     self.smooth_manual_move(array)
        #     return
        rospy.loginfo(">>> MOVE: %s", array)
        # NEW: Now the check is done with /scan data in cmd_vel_manager
        #if not skip_checks:
            #self.manual_move_checker(array)
        self._lock.acquire()
        data_to_send = Float32MultiArray()
        data_to_send.data = array
        msg = triskarone_msgs.msg.manual_move_baseGoal(goal=data_to_send, correct=correct, ar_goal=ar_goal, adjust_angle = adjust_angle, tremble_while_approach = tremble)
        self.cmd_vel_client.send_goal(msg)
        self._lock.release()

    def adjust_rotation(self):
        # The control is done automatically, so we send a null control vector 
        array = [0, 0, 0, 0, 0, 0, 0]
        self.manual_move(array, adjust_angle= True)


    # def smooth_manual_move(self, array):
    #     """array -> [distance_x, distance_y, distance_theta, vel_x, vel_y, vel_theta, waiting_time]"""
    #     rospy.loginfo(">>> SMOOTH MANUAL MOVE: %s", array)
    #     dx, dy, dtheta = array[:3]
    #     vx, vy, vtheta = array[3:6]
    #     factor = 0.05
    #     while dx > 0 or dy > 0 or dtheta > 0:
    #         data = list(map(lambda v, f=factor: v*f, [dx, dy, dtheta, vx, vy, vtheta, 0])) # smoothed data
    #         self.manual_move(data, True)
    #         dx, dy, dtheta = map(lambda v, f=factor: max(0, v - v*f), [dx,dy,dtheta])   # update distances
    #         factor = min(1, 2*factor)   # update factor
    #         # while 0 == self.check_single_action_status(self.cmd_vel_client, "smooth_manual_move"):
    #         #     continue    # wait

    #------------------------- SCENIC ROUTINES -------------------------

    def start_scene(self):
        rospy.loginfo(">>> STARTING SCENE")

        my_debug = False
        ans = input("DEBUG MOVEMENTS AND REACTIONS (yes/no)? ").lower()
        if ans == "yes":
            my_debug = True
        # self.try_smooth_moves = input("SMOOTH MOVES? (yes/no)? ").upper()[0] != "N" # TODO debugging


        #self.init_rotation = rospy.get_param("initial_orient")
        #self.reach_center_of_stage()
        self.acts = 0

        interaction_sequence = []

        
        # Start acting loop
        while self.acts < self.scene_length:
            interaction = {}
            thread_list = []

            # -- debug --
            if my_debug:
                #wm = namedtuple("debug_obj_1", "actorPosX actorPosY actorOrientation")
                # reaction = namedtuple("debug_obj_2", "approach_dir approach_dist approach_speed rotate_dir rotate_speed actions pause_after")                

                #wm.actorPosX = read_float("insert actor pos x: ")
                #wm.actorPosY = read_float("insert actor pos y: ")
                #wm.actorOrientation = read_float("insert actor orientation (rad, 0 = facing the robot): ")
                #wm.endScene = False
                
                #ar_distance, ar_angle, ar_facing = self.compute_orient(wm)
                #rospy.loginfo(">>> compute orient results (distance, angle, facing): %s, %s, %s", ar_distance, ar_angle, ar_facing)

                # reaction.rotate_dir = int(read_float("insert rotate (-1,0,1): "))
                # reaction.rotate_speed = read_float("insert rotate speed: ")
                # reaction.approach_dir = int(read_float("insert approach (-1,0,1): "))
                # reaction.approach_dist = read_float("insert distance of approach: ")
                # reaction.approach_speed = read_float("insert approach speed: ")
                # reaction.actions = input("insert actions to perform (whitespace separated): ").split()
                # reaction.pause_after = read_float("insert pause after action: ")
                #done = False

                t1 = Thread(target=self._ask_reaction)
                t1.start()
                t2 = Thread(target=self._start_routine)
                t2.start()

                '''
                while(not done):
                    robot_react = int(input('Choose between: 1->"attack", 2->"scolding", 3->"intimidate", 4->"grudge", 5->"sharing_happiness", 6->"happy_person", 7->"satisfaction", 8->"sharing_fear", 9->"running_away", 10->"sharing_sadness", 11->"disappointment", 12->"surprise", 13->"disbelief", 14->"astonishment", 15->"none" \n Robot reaction: '))
                    done = robot_react in range(1,16)
                '''

                t1.join()
                t2.join()

                #ar_distance, ar_angle = self.compute_orient(wm)
                ar_distance, ar_angle = compute_orient(self.wm.actorPosX, self.wm.actorPosY)
                #reaction = self.get_reaction(robot_react, ar_distance, ar_facing)
                #robot_react = REACTION_STATES[robot_react - 1]
                #reaction = self.get_reaction(robot_react, ar_distance)
                reaction = self.get_reaction(self.wm.robotReaction, ar_distance)
                
            # -- end debug --

            else:
                '''
                # Get Robot's Behavior
                rospy.loginfo(">>> WAITING FOR BEHAVIOR MODEL RESPONSE")
                wm = self.get_behavior()
                rospy.loginfo("world model:\n%s", wm)
                '''

                t1 = Thread(target=self._get_world_model)
                t2 = Thread(target=self._start_routine)

                t1.start()
                t2.start()

                t1.join()
                t2.join()

                # Analyze Actor Behaviour from world model
                #ar_distance, ar_angle = self.compute_orient(wm)
                ar_distance, ar_angle = compute_orient(self.wm.actorPosX, self.wm.actorPosY)
                #rospy.loginfo(f"distance: {ar_distance}\nangle: {ar_angle}°\nactor is facing the robot: {ar_facing}")

                # Fetch Robot's Reaction
                rospy.loginfo(">>> WAITING FOR REACTION SERVER RESPONSE")
                #reaction = self.get_reaction(wm.robotReaction, ar_distance, ar_facing)
                reaction = self.get_reaction(self.wm.robotReaction, ar_distance)

            with open("resources/reactions/eyes_reset.json", 'r') as f:
                self.handle_actions(json.load(f))   # resetting eyes
            with open("resources/reactions/body_reset.json", 'r') as f:
                self.handle_actions(json.load(f))   # resetting body

            '''
            # face or not the actor
            if reaction.rotate_dir:
                corrected_angle = ar_angle - SX_ROTATION_ERROR * ar_angle if ar_angle > 0 else ar_angle - DX_ROTATION_ERROR * ar_angle
                rospy.loginfo(f"I ar angle is: {ar_angle}")
                rospy.loginfo(f"I am rotating of {corrected_angle}")
                rospy.loginfo(f"computed with x = {wm.actorPosX}, and y = {wm.actorPosY}")
                #self.rotate(reaction.rotate_dir, reaction.rotate_speed, ar_angle - 0.2  if ar_angle > 0 else ar_angle + 0.2)
                self.rotate(reaction.rotate_dir, reaction.rotate_speed, corrected_angle )
            else:
                reaction.rotate_dir = 1 # in order not to influence the approaching phase
            '''

            if reaction.rotate_dir:
                self.rotate_new(reaction.rotate_dir, reaction.rotate_speed)
            else:
                reaction.rotate_dir = 1 # In order not to influence the approaching phase
            
            if len(reaction.actions_before_approach) != 0 :
                # handle some action before approaching the actor
                self.handle_additional_actions(reaction.actions_before_approach)

                # if we have lost the orientation, adjust the angle again
                if reaction.rotate_dir:
                    rospy.loginfo("readjusting the angle")
                    self.rotate_new(reaction.rotate_dir, reaction.rotate_speed)
                else:
                    reaction.rotate_dir = 1 # In order not to influence the approaching phase

            # move forward/away
            if not reaction.approach_dir == 0 :
                if len(reaction.actions_while_approach) != 0 :
                    t_aa = Thread(target=self.approach_actor, args=(reaction, ar_distance,))
                    t_aa.start()
                else:
                    self.approach_actor(reaction, ar_distance)  
                # additional note: if the robot has already rotated, we need to tweak the approaching speed, considering the inverted frame axis xy

            if len(reaction.actions_while_approach) != 0 :
                t_ha = Thread(target=self.handle_additional_actions, args=(reaction.actions_while_approach,))
                t_ha.start()

            if len(reaction.actions_while_approach) != 0:
                t_aa.join()
                t_ha.join()

            self.handle_additional_actions(reaction.actions)

            self.ready_to_react = False

            # Adjust angle
            if reaction.adjust_angle:
                self.adjust_rotation()
            
            # NEW: save history of sequences
            actor_scenicAction = self.wm.scenicAction
            robot_reaction = self.wm.robotReaction
            interaction = { "sequence_number": self.acts,
                            "actor_scenicAction": actor_scenicAction,
                            "robot_reaction": robot_reaction}  
            
            interaction_sequence.append(interaction)          
            
            r = rospy.Rate(1 / reaction.pause_after) if reaction.pause_after > 0 else rospy.Rate(10) 
            r.sleep()

            self.acts +=1

            # Verify if scene is terminated
            if self.wm.endScene:
                rospy.loginfo(">>> SCENE IS OVER")
                break
        
        msg = Empty()
        self.rst_ai_publisher.publish(msg)


        # Save interaction sequence
        rospy.loginfo("saving interaction sequence as json")
        jsonStr = json.dumps(interaction_sequence)
        with open(f"/home/airlab/Documents/merge2/interaction_sequences/sequence_of_{datetime.now()}", "w") as file:
            file.write(jsonStr)



    def _ask_reaction(self):
        #rospy.loginfo("in ask reaction")
        done = False
        
        while not done:
            #rospy.loginfo("in while loop")

            ans = input('Choose between: 1->"attack", 2->"scolding", 3->"intimidate", 4->"grudge", 5->"sharing_happiness", 6->"happy_person", 7->"satisfaction", 8->"sharing_fear", 9->"running_away", 10->"sharing_sadness", 11->"disappointment", 12->"surprise", 13->"disbelief", 14->"astonishment", 15->"none" \n Robot reaction: ')
            try:
                robot_react = int(ans)
                done = robot_react in range(1,16)
            except ValueError:
                print("please enter an integer from 1 to 15")
                done = False

        wm = self.get_behavior()
        robot_react = REACTION_STATES[robot_react - 1]
        wm.robotReaction = robot_react
        self.wm = wm
        rospy.loginfo("world model:\n%s", self.wm)
        self.ready_to_react = True

    def _get_world_model(self):
        rospy.loginfo(">>> WAITING FOR BEHAVIOR MODEL RESPONSE")
        self.wm = self.get_behavior()
        rospy.loginfo("world model:\n%s", self.wm)
        self.ready_to_react = True

    def _start_routine(self):
        while(not self.ready_to_react):
            self._handle_routine()

    def _handle_routine(self):
        # BY DEFAULT EXECUTE A POSITIVE ROUTINE
        routine = ["positive_routine_action"]
        if not self.wm == None:
            last_reaction = self.wm.robotReaction
            if last_reaction in POSITIVE_REACTIONS:
                routine = ["positive_routine_action"]
            elif last_reaction in NEGATIVE_REACTIONS:
                routine = ["negative_routine_action"]

        self.handle_additional_actions(routine)   # resetting eyes

    def handle_additional_actions(self,actions):
        # perform set of actions
        for action in actions:
            rospy.loginfo(f">>> PERFORMING REACTION: {action}")
            with open(f"resources/reactions/{action}.json", 'r') as f:
                a = json.load(f)
                self.handle_actions(a)
                self.smart_check_actions_status(a)

    ''' ---DEPRECATED---
    def reach_center_of_stage(self):
        # TODO test this method
        origin = namedtuple("world_model", "actorPosX actorPosY actorOrientation")
        origin.actorPosX = 0
        origin.actorPosY = 0
        origin.actorOrientation = 0
        #d, a, f = self.compute_orient(origin)
        d, a = self.compute_orient(origin)
        self.rotate(1, 1, a)
        self.approach_actor(1, 1, 1, 0.5, d)
    '''
    
    def rotate(self, rotate_dir, rotate_speed, angle):
        lock = Lock()
        lock.acquire()

        """perform a rotation of the robot in order to face the actor or give the back to him"""
        
        if rotate_dir == 1:        # the robot should face the actor
            theta = abs(angle)                      # angle to space
            clockwise = -1 if angle < 0 else 1      # clockwise (-1) or counterclockwise (+1) rotation

            array = [0, 0, theta, 0, 0, clockwise*rotate_speed, 0]

        elif rotate_dir == -1:
            theta = pi - abs(angle)                 # angle to space
            clockwise = -1 if angle > 0 else 1      # clockwise (-1) or counterclockwise (+1) rotation

            array = [0, 0, theta, 0, 0, clockwise*rotate_speed, 0]

        self.manual_move(array) # perform atomic action
        self.smart_check_actions_status({"manual_move" : array}) # check action status
        lock.release()

    def rotate_new(self, rotate_dir, rotate_speed):

        array = [0, 0, 0, 0, 0, 0, 0]

        data_to_send = Float32MultiArray()
        data_to_send.data = array
        msg = triskarone_msgs.msg.manual_move_baseGoal(goal=data_to_send, correct=False, ar_goal=0, adjust_angle = False, rotate_dir = rotate_dir, rotate_speed = rotate_speed)
        self.cmd_vel_client.send_goal(msg)
        self.smart_check_actions_status({"manual_move" : array}) # check action status


    def approach_actor(self, reaction, ar_distance):
        rotate_dir = reaction.rotate_dir
        direction = reaction.approach_dir
        distance_goal = reaction.approach_dist
        speed = reaction.approach_speed
        correct = reaction.correct_move
        tremble = reaction.tremble

        """approach the actor (direction == 1) or run away from him (direction == -1)"""

        
        if direction ==1:
            if ar_distance <= distance_goal: #if the robot wants to go forward to a proxemic zone, but it is already in a more intimate proxemic zone
                move = 0
            else :
                move = ar_distance - distance_goal
        else :
            if ar_distance >= distance_goal: #if the robot wants to go backward to a proxemic zone, but it is already in a less intimate proxemic zone
                move = 0
            else:
                move = distance_goal - ar_distance

        array = [move, 0, 0, rotate_dir * direction * speed, 0, 0, 0]   # perform action
        self.manual_move(array, correct, ar_goal = distance_goal, tremble=tremble)
        self.smart_check_actions_status({"manual_move" : array})

    '''
    def compute_orient(self, world_model):
        #FACING_CONST = 45 * pi / 180
        ax, ay = world_model.actorPosX, world_model.actorPosY
        distance = ( ax**2 + ay**2 ) ** 0.5         # actor-robot distance
        angle = atan2(ay, ax)                       # actor-robot angle
        # assert abs(angle) <= pi
        #is_facing = abs(angle + world_model.actorOrientation) <= FACING_CONST    # actor is facing the robot?
        return distance, angle #, is_facing
    '''
    #------------------------- ACTIONS HANDLING -------------------------
    
    def manual_move_checker(self, array):
        """recovery behaviour for manual move"""
        
        def sign(x :int) -> int:
            if x > 0: return 1
            if x < 0: return -1
            return 0
        
        px, py = self.position
        dx, dy = array[0], array[1]
        vx, vy = sign(array[3]), sign(array[4])
        
        if abs(dx*vx + px) >= self.MAP_SIZE / 2 or abs(dy*vy + py) >= self.MAP_SIZE / 2:        # if action out of bound return to center
            new_dx, new_dy = abs(px), abs(py)
            new_vx, new_vy = -sign(px), -sign(py)
            rospy.loginfo(f"MANUAL MOVE RECOVERY -> {self.position}, ({new_dx*new_vx}, {new_dy*new_vy})")
            self.manual_move([new_dx, new_dy, 0, new_vx, new_vy, 0, 1], skip_checks =True)   # recovery doubt:it does not return to center(?)
            self.position = [0,0]
        
        # continue old movement
        self.position[0] += dx*vx
        self.position[1] += dy*vy
        rospy.loginfo(f"current position -> {self.position}")

    def smart_check_actions_status(self, actions):
        r = rospy.Rate(3)
        r.sleep()
        self._lock.acquire()
        rospy.loginfo(">>> SMART CHECK ACTIONS STATUS")

        '''
        actions_updated = {k: False for k in actions.keys()}
        rospy.loginfo(f"wait started with the following actions to update: {actions_updated}")
        #while lambda actions_updated: all(valore ==False for valore in actions_updated.values()):
        while False in actions_updated.values():
            for action in actions:
                actions_updated[action] = self.check_updated(action)   
            #rospy.loginfo(f"in wait with the following actions to update: {actions_updated}")

        rospy.loginfo(f"updated wait ended with the following actions to update: {actions_updated}")
        '''

        rospy.loginfo(f"wait started with the following status: {self.actions_status}")
        count = 0
        while not (self.actions_status["speak"][0] and self.actions_status["move_base"][0] and self.actions_status["move_eyes"][0] and self.actions_status["move_body"][0] and self.actions_status["manual_move"][0] and self.actions_status["tremble"][0] ):
            #wait for them
            if (count % 15) == 0:
                #print(f"here is actions_status: {self.actions_status}")
                pass
            #rospy.loginfo("in waiting")
            count+=1
            pass
        rospy.loginfo("wait ended")

        self._lock.release()

    def check_updated(self, action):
        t = time.localtime()
        current_time = int(time.strftime("%H%M%S", t))
        #rospy.loginfo(f"in check_updated, current time: {current_time}")
        #rospy.loginfo(f"in check_updated, last_update: {self.actions_status[action][1]}")
        if self.actions_status[action][1] >= current_time - 2:
            return True
        return False        

    def check_single_action_status(self, action_client, description) -> int:
        """check atomic action success/failure"""
        
        status = action_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"action {description} SUCCEEDED")
            return 1
        elif status == GoalStatus.ABORTED:
            rospy.loginfo(f"action {description} FAILED")
            return -1
        return 0


    def check_actions_status(self, actions):
        """check multiple actions success/failure"""

        if self.is_debug:
            rospy.loginfo("debugging, skipping action check")
            return

        # publish on /is_moving topic
        is_moving_msg = Bool()
        is_moving_msg.data = True
        self.is_moving_publisher.publish(is_moving_msg)
        
        rospy.loginfo(">>> CHECK ACTIONS STATUS")
        has_to_speak = False
        move_base = False
        move_base_error = False
        move_eyes = False
        move_body = False
        manual_move=False

        if "speak" in actions:
            has_to_speak = True

        if "move_base" in actions:
            move_base = True

        if "move_eyes" in actions:
            move_eyes = True

        if "move_body" in actions:
            move_body = True

        if "manual_move" in actions:
            manual_move=True

        while has_to_speak or move_base or move_eyes or move_body or manual_move:
            if has_to_speak:
                is_done = self.check_single_action_status(self.audio_client, "speak")
                has_to_speak = (is_done == 0)
            
            if move_base:
                is_done = self.check_single_action_status(self.move_base_client, "move_base")
                move_base_error = (is_done == -1)
                move_base = (is_done == 0)
            
            if move_eyes:
                is_done = self.check_single_action_status(self.eyes_manager_client, "move_eyes")
                move_eyes = (is_done == 0)

            if move_body:
                is_done = self.check_single_action_status(self.body_manager_client, "move_body")
                move_body = (is_done == 0)

            if manual_move:
                is_done = self.check_single_action_status(self.cmd_vel_client, "manual_move")
                manual_move = (is_done == 0)
            
        while move_base_error:
            rospy.loginfo(">>> CHECKING MOVE_BASE ERROR")
            rospy.loginfo("Failed to get in position, starting recovery")
            rospy.loginfo("Move the robot and press up for retry to move in position or down to go to next_section")
            self.up_pressed = False
            self.down_pressed = False
            self.move_base_recovery = True
            while not (self.up_pressed or self.down_pressed):
                continue
            if self.up_pressed:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = actions['move_base'][0]
                goal.target_pose.pose.position.y = actions['move_base'][1]
                goal.target_pose.pose.orientation.z = actions['move_base'][2]
                goal.target_pose.pose.orientation.w = actions['move_base'][3]
                self.move_base_client.send_goal(goal)
                rospy.loginfo("Moving to position: x:%f y:%f z:%f w:%f", actions['move_base'][0],
                              actions['move_base'][1],
                              actions['move_base'][2], actions['move_base'][3])
                move_base = True
                while move_base:
                    if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
                        move_base = False
                        move_base_error = False
                        rospy.loginfo("Correctly moved to the desired position")
                    if self.move_base_client.get_state() == GoalStatus.ABORTED:
                        move_base = False
                        move_base_error = True
            if self.down_pressed:
                move_base_error = False
        
        rospy.loginfo("actions finished")
        
        # publish on /is_moving topic
        is_moving_msg.data = False
        self.is_moving_publisher.publish(is_moving_msg)


    def handle_actions(self, actions):
        """dispatch actions to their handlers"""
        thread_list =[]
        
        if "tremble" in actions:
            t = Thread(target=self.tremble, args=(actions['tremble'],))
            thread_list.append(t)

        if "move_eyes" in actions:
            t = Thread(target=self.move_eyes, args=(actions['move_eyes'],))
            thread_list.append(t)
        
        if "move_body" in actions:
            t = Thread(target=self.move_body, args=(actions['move_body'],))
            thread_list.append(t)
        
        if "speak" in actions:
            file_to_play = actions['speak']
            goal = triskarone_msgs.msg.play_audioGoal(filename=file_to_play)
            self.audio_client.send_goal(goal)
            rospy.loginfo("Playing audio: %s", file_to_play)
        
        if "manual_move" in actions:
            t = Thread(target=self.manual_move, args=(actions['manual_move'],))
            thread_list.append(t)
        
        if "move_base" in actions:
            self.move_base(actions["move_base"])
        
        if "do_nothing" in actions:
            time = int(actions['do_nothing'])
            rospy.loginfo("Do nothing for %d seconds", time)
            rospy.sleep(time)
        
        if "start_scene" in actions:
            self.start_scene()
        
        if "end" in actions:
            rospy.loginfo("The show is finished, bye!")
            rospy.signal_shutdown("The show is finished")

        # not in "old robocchio code"
        if "get_command" in actions:
            self.get_command()

        for t in thread_list:
            t.start()

        for t in thread_list:
            t.join()        



    def get_command(self):
        """issue single commands to test the robot features"""
        
        rospy.loginfo(">>> ISSUE SINGLE COMMANDS")
        all_commands = ["tremble", "move_eyes", "move_body", "speak", "manual_move", "move_base", "start_scene", "stop"]
        
        read_command = True
        while read_command:
            command = ""
            while command not in all_commands:
                command = input(f"Enter command {all_commands}: ").lower()
            rospy.loginfo("Executing: %s", command)
            
            if command == "tremble":
                array = []
                answer = "Y"
                while answer == "Y":
                    duration = int(read_float("Enter duration: "))
                    pause = int(read_float("Enter pause: "))
                    intensity = int(read_float("Enter intensity: "))
                    array += [duration, pause, intensity]
                    answer = input("More values ? (y/n): ").upper()
                self.tremble(array)


            if command == "move_eyes":
                array = []
                answer = "Y"
                while answer == "Y":
                    pause = int(read_float("Enter pause: "))
                    move_type = int(read_float("Enter type of movement: "))
                    speed = int(read_float("Enter speed: "))
                    array += [pause, move_type, speed]
                    answer = input("More values ? (y/n): ").upper()
                self.move_eyes(array)
            
            if command == "move_body":
                array = []
                answer = "Y"
                while answer == "Y":
                    pause = int(read_float("Enter pause: "))
                    move_type = int(read_float("Enter type of movement: "))
                    speed = int(read_float("Enter speed: "))
                    array += [pause, move_type, speed]
                    answer = input("More values ? (y/n): ").upper()
                self.move_body(array)
            
            if command == "speak":
                file_to_play = input("Enter file to play: ")
                goal = triskarone_msgs.msg.play_audioGoal(filename=file_to_play)
                self.audio_client.send_goal(goal)
                rospy.loginfo("Playing audio: %s", file_to_play)
            
            if command == "manual_move":
                array = []
                answer = "Y"
                while answer == "Y":
                    posX = read_float("Enter X: ")
                    posY = read_float("Enter Y: ")
                    posTheta = read_float("Enter Theta (rad): ")
                    velX = read_float("Enter velX: ")
                    velY = read_float("Enter velY: ")
                    velTheta = read_float("Enter velTheta: ")
                    waiting_time = read_float("Enter waiting time: ")
                    array += [posX, posY, posTheta, velX, velY, velTheta, waiting_time]
                    answer = input("More values ? (y/n): ").upper()
                self.manual_move(array)
            
            if command == "move_base":
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = float(input("Enter X position: "))
                goal.target_pose.pose.position.y = float(input("Enter Y position: "))
                goal.target_pose.pose.orientation.z = float(input("Enter Z orientation: "))
                goal.target_pose.pose.orientation.w = float(input("Enter W orientation: "))
                self.move_base_client.send_goal(goal)

            if command == "start_scene":
                self.start_scene()

            if command == "stop":
                read_command = False


if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir("..")
    rospy.loginfo(">>> CURRENT WORKING DIRECTORY: " + os.getcwd())
    try:
        MainController()
    except rospy.ROSInterruptException:
        pass
