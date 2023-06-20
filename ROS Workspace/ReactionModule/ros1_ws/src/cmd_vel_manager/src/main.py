#!/usr/bin/env python
import actionlib
import rospy

from triskarone_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, String, Int16, Bool
from math import sin, cos, pi, atan2
from utils.functions import compute_orient
from datetime import datetime
import math, random, json, time
import numpy as np

MAXIMUM_DISTANCE = 1 # meters
MAXIMUM_LT_ERROR = 0.0875 # rads = 5 degrees
MAXIMUM_WB_ERROR = 25 # frame width units
WB_FRAME_RANGE = 480
MUL_FACTOR = 1.5
MIN_DIST_TO_APPROX_ROTATION = 2.5

#seconds in which I consider updated position data
WB_HISTORY_PARAMETER = 5
LT_HISTORY_PARAMETER = 5

class CmdVelAction(object):
    # create messages that are used to publish feedback/result
    feedback = manual_move_baseFeedback()
    result = manual_move_baseResult()

    def __init__(self, name):
        self._action_name = name
        self.nearer_obstacle = float('inf')
        self.shortest_back = float('inf')
        self.shortest_right = float('inf')
        self.shortest_front = float('inf')
        self.shortest_left = float('inf')

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.obstacle_info_pub = rospy.Publisher('obstacle_info', String, queue_size = 3)#for debugging
        self.manual_move_status_pub = rospy.Publisher('manual_move_status', Bool, queue_size=10)
        self.tremble_status_pub = rospy.Publisher('tremble_status', Bool, queue_size=10)


        self.odom_sub = rospy.Subscriber('vel', Twist, self.vel_callback)
        #self.odom = rospy.Subscriber('/odom', Odometry, self.get_rotation)
        # To update the closest obstacles info
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.onScan_callback)
        # To update the actor position from the leg tracker
        self.actor_pos_sub = rospy.Subscriber("actor_pos_info", String, self.onActPos_callback)
        # To update the actor orientation from the webcam
        self.actor_pos_sub = rospy.Subscriber("/actor_orientation", Int16, self.onActOrientation_callback)

        self.last_time = rospy.get_rostime()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.yaw = 0.0
        self.actor_pos_info = {}
        self.actor_orientation = {}

        self.counter = 0

        ##self._as = actionlib.SimpleActionServer(self._action_name, manual_move_baseAction, execute_cb=self.execute_cb,
        ##                                        auto_start=False)
        self._as = actionlib.SimpleActionServer('move', manual_move_baseAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()

        self._as_tremble = actionlib.SimpleActionServer('tremble', manual_move_trembleAction, execute_cb=self.execute_tremble,
                                                auto_start=False)
        self._as_tremble.start()

        rospy.loginfo("%s is started", rospy.get_name())
    
    def execute_tremble(self, msg):
        self.tremble_status_pub.publish(False)
        cmd_array = msg.goal.data
        duration = cmd_array[0]
        pause_after = cmd_array[1]
        intensity = cmd_array[2]
        #timer loop in which i tremble the robot
        start_time = time.time()    
        rospy.loginfo("trembling")    

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time > duration:
                break
            data_to_send = Twist()
            data_to_send.angular.z = np.sign(np.random.uniform(-1, 1))* intensity * np.random.uniform(0.3, 1)
            #data_to_send.angular.z = np.random.uniform(-1.5, 1.5)
            self.cmd_vel_pub.publish(data_to_send)


        if pause_after != 0:
            r = rospy.Rate(1 / pause_after)   # sleep 'pause' seconds
            r.sleep()

        self.result.response = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as_tremble.set_succeeded(self.result)
        self.tremble_status_pub.publish(True)
        rospy.loginfo("tremble done")



    def onActOrientation_callback(self, msg):
        x = msg.data
        now = datetime.now()
        current_time = int(now.strftime("%H%M%S"))

        self.actor_orientation = { "orientation" : x,
                                    "time" : current_time}

    def onActPos_callback(self, msg):
        m = json.loads(msg.data)
        ax = round(m["actorPosX"], 5)
        ay = round(m["actorPosY"], 5)
        #self.ar_distance, self.ar_angle = compute_orient(ax, ay)
        now = datetime.now()
        current_time = int(now.strftime("%H%M%S"))

        ar_distance, ar_angle = compute_orient(ax, ay)
        self.actor_pos_info = { "ar_distance"   : ar_distance,
                                "ar_angle"      : ar_angle,
                                "time"          : current_time}
        #rospy.loginfo(f"here is the ar_angle: {self.ar_angle}")

    '''
    def get_rotation(self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.w, orientation_q.z]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # update angle which goes from 0 to 2pi in counterclockwise verse
        self.angle = yaw if yaw>0 else (pi + (pi - abs(yaw)))
    '''

    def onScan_callback(self, msg):
        ranges = msg.ranges
        self.shortest_back = float('inf')
        self.shortest_right = float('inf')
        self.shortest_front = float('inf')
        self.shortest_left = float('inf')

        for i in range(len(ranges)):
            angle = msg.angle_min + i*msg.angle_increment
            if angle <= -3/4*pi or angle >= 3/4*pi:
                if ranges[i] < self.shortest_back:
                    self.shortest_back = ranges[i]
            if angle >= 1/4*pi and angle < 3/4*pi:
                if ranges[i] < self.shortest_left:
                    self.shortest_left = ranges[i]
            if angle >= -1/4*pi and angle < 1/4*pi:
                if ranges[i] < self.shortest_front:
                    self.shortest_front = ranges[i]
            if angle > -3/4*pi and angle < -1/4*pi:
                if ranges[i] < self.shortest_right:
                    self.shortest_right = ranges[i]
        
        # for debugging
        #self.obstacle_info_pub.publish(String("nearer front: " + str(self.shortest_front) + "\n nearer left: " + str(self.shortest_left) + "\n nearer right: " + str(self.shortest_right) + "\n nearer back: " + str(self.shortest_back)))


    def vel_callback(self, data):
        vx = data.linear.x
        vy = data.linear.y
        vth = data.angular.z
        current_time = rospy.get_rostime()
        dt = (current_time - self.last_time).to_sec()
        delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
        delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
        delta_th = vth * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.last_time = current_time

    def execute_cb(self, msg):
        self.manual_move_status_pub.publish(False)
        movements = msg.goal.data
        should_correct_angle = msg.correct
        ar_goal = msg.ar_goal
        should_adjust_angle = msg.adjust_angle
        tremble = msg.tremble_while_approach
        # new infos added to message in order to improve the rotation phase
        rotate_dir = msg.rotate_dir
        rotate_speed = msg.rotate_speed
        rospy.loginfo("cmd_vel_manager executing -> %s" % str(movements))
        rospy.loginfo("with correction -> %s" % str(msg.correct))
        rospy.loginfo("with angle adjustment -> %s" % str(msg.adjust_angle))
        rospy.loginfo("with tremble -> %s" % str(msg.tremble_while_approach))
        self.counter = 0
        has_finished = False
        while self.counter < len(movements):
            # check that preempt has not been requested by the client
            #if self._as.is_preempt_requested():
            #    rospy.loginfo('%s: Preempted' % self._action_name)
            #    self._as.set_preempted()
            #    break

            array = [round(movements[self.counter], 2), round(movements[self.counter + 1], 2),
                     round(movements[self.counter + 2], 2)]
            x_speed = movements[self.counter + 3]
            y_speed = movements[self.counter + 4]
            angular_speed = movements[self.counter + 5]

            pause = movements[self.counter + 6]

            self.x = 0.0
            self.y = 0.0
            self.th = 0.0


            x_start = self.x
            y_start = self.y
            th_start = self.th
            #angle_start = self.angle
            now = datetime.now()
            start_mov_time = int(now.strftime("%H%M%S"))


            x_done = False
            y_done = False
            yaw_done = False
            while not (x_done and y_done and yaw_done):
                data_to_send = Twist()
                
                if not x_done:
                    # continue to give commands if there aren't obstacles in the direction in which I am moving
                    should_i_move_x = False  
                    if x_speed > 0 and self.shortest_front > MAXIMUM_DISTANCE:
                        should_i_move_x = True
                    elif x_speed <= 0 and self.shortest_back > MAXIMUM_DISTANCE:
                        should_i_move_x = True
                    
                    dist_to_move = 0
                    if not ar_goal == 0:
                        dist_to_move = self.actor_pos_info["ar_distance"] - ar_goal if x_speed > 0 else ar_goal - self.actor_pos_info["ar_distance"]

                    if (abs(self.x - x_start) < array[0] or dist_to_move > 0) and should_i_move_x:
                        data_to_send.linear.x = x_speed
                    else:
                        if(not should_i_move_x):
                            rospy.loginfo("stopped to go in the x direction because a near obstacle detected")
                        data_to_send.linear.x = 0.0
                        x_done = True
                
                if not y_done:
                    should_i_move_y = False
                    
                    if y_speed > 0 and self.shortest_left > MAXIMUM_DISTANCE:
                        should_i_move_y = True              
                    elif y_speed <= 0 and self.shortest_right > MAXIMUM_DISTANCE:
                        should_i_move_y = True
                    
                    if (abs(self.y - y_start) < array[1]) and should_i_move_y:
                        data_to_send.linear.y = y_speed
                    else:
                        if(not should_i_move_y):
                            rospy.loginfo("stopped to go in the y direction because a near obstacle detected")
                        data_to_send.linear.y = 0.0
                        y_done = True
                
                if not yaw_done:

                    # NEW rotation algorithm
                    if rotate_dir != 0:
                        [yaw_done, data_to_send.angular.z] = self.rotate(rotate_dir, start_mov_time, speed=rotate_speed)

                    else:

                        # NEW: case in which at the end of the scenic action I should adjust the angle
                        if should_adjust_angle:
                            yaw_done, data_to_send.angular.z = self.rotate(direction=1,start_mov_time=start_mov_time,speed=1)

                        # Normal rotation phase (OLD rotation algorithm)
                        else:
                            if abs(self.th - th_start) < array[2]:
                                data_to_send.angular.z = angular_speed
                            else:
                                data_to_send.angular.z = 0.0
                                yaw_done = True
                
                should_rotate_while_moving = should_correct_angle or tremble

                if should_rotate_while_moving:
                    [_, data_to_send.angular.z] = self.rotate(1, start_mov_time, 1, tremble=tremble)

                self.cmd_vel_pub.publish(data_to_send)
                

            if pause != 0:
                r = rospy.Rate(1 / pause)   # sleep 'pause' seconds
                r.sleep()
            self.counter = self.counter + 7

            # se abbiamo finito, passa al successivo
        self.result.response = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self.result)
        self.manual_move_status_pub.publish(True)


    def rotate(self, direction, start_mov_time, speed, tremble = False):
        yaw_done = False
        webcam_ar_angle = self.actor_orientation["orientation"]        
        webcam_time = self.actor_orientation["time"]    
        leg_tracker_time = self.actor_pos_info["time"] 
        leg_track_ar_angle = self.actor_pos_info["ar_angle"]
        leg_track_ar_distance = self.actor_pos_info["ar_distance"]

        if direction == 1: # towards the actor
            if webcam_time>=start_mov_time and webcam_ar_angle != 3000: # we have updated and available data from the webcam
                time = webcam_time 
                ar_angle = webcam_ar_angle
                normalized_wb_angle = abs(ar_angle)/(WB_FRAME_RANGE/2) # normalized by the width of the webcam frame divided by two (due to the transformation)
                # Apply smoothing by normalized_wb_angle in rotation when the device to detect the angle is swithced
                normalized_angle = (1 - normalized_wb_angle) * abs(ar_angle)/(480/2) + (normalized_wb_angle) * abs(leg_track_ar_angle)/pi # normalized by the width of the webcam frame divided by two (due to the transformation)
                
                # In order to remove the robot to dandling while rotationg, we add tolerance in the rotation phase when the actor is far from the robot
                approx_parameter = 15 if leg_track_ar_distance>4 else 0
                
                if(abs(ar_angle) > MAXIMUM_WB_ERROR + approx_parameter and time >= start_mov_time - WB_HISTORY_PARAMETER):
                    #apply correction
                    data_to_send = speed * math.copysign(1, ar_angle) * direction * normalized_angle * MUL_FACTOR
                    #rospy.loginfo(f"I am using rotate_new, sending angular speed equal to : {data_to_send}")
                    #rospy.loginfo(f"Ar_angle in rotate_new webcam is : {ar_angle}")
                else:
                    #rospy.loginfo(f"sending 0 rotation from webcam data")
                    data_to_send = 0.0
                    yaw_done = True
                '''
            # If the actor is far enough from the robot, then leg_tracker should be used in open system, without feedback. this because Leg_tracker does not update the actor position.
            elif self.actor_pos_info["ar_distance"] > MIN_DIST_TO_APPROX_ROTATION:
                rospy.loginfo(f"into elif, considering leg tracker to approximate rotation")
                time = leg_tracker_time
                angle_to_move = abs(leg_track_ar_angle)
                if round(self.th < angle_to_move - MAXIMUM_LT_ERROR and time  >= start_mov_time - LT_HISTORY_PARAMETER):
                    data_to_send = speed * (1- abs(self.th )/pi) * MUL_FACTOR
                else:
                    rospy.loginfo(f"into else, sending null angular velocity")
                    data_to_send = 0.0
                    yaw_done = True
                '''


            else: # use leg tracker to rotate
                time = leg_tracker_time
                ar_angle = leg_track_ar_angle
                normalized_angle = abs(ar_angle)/pi

                #rospy.loginfo(f"leg_tracker_time: {time}")
                #rospy.loginfo(f"leg_tracker_ar_angle: {leg_track_ar_angle}")
                #rospy.loginfo(f"start_moving_time: {start_mov_time}")
                if(abs(ar_angle) > MAXIMUM_LT_ERROR and time  >= start_mov_time - LT_HISTORY_PARAMETER):
                    #apply correction
                    data_to_send = speed * math.copysign(1, ar_angle) * direction * normalized_angle * MUL_FACTOR
                    #rospy.loginfo(f"I am using rotate_new, sending angular speed equal to : {data_to_send}")
                    #rospy.loginfo(f"Ar_angle in rotate_new leg_tracker is : {ar_angle}")
                else:
                    #rospy.loginfo(f"sending 0 rotation from leg tracker data")
                    data_to_send = 0.0
                    yaw_done = True

        elif direction == -1: # recto the actor
            time = leg_tracker_time
            angle_to_move = pi - abs(leg_track_ar_angle)
            if round(self.th < angle_to_move - MAXIMUM_LT_ERROR and time  >= start_mov_time - LT_HISTORY_PARAMETER):
                data_to_send = speed * (1- abs(self.th )/pi) * MUL_FACTOR
                #rospy.loginfo(f"into if, sending angular velocity of: {data_to_send}")
                #rospy.loginfo(f"into if, here is rounded leg track angle: {round(abs(leg_track_ar_angle),2)}")
                #rospy.loginfo(f"into if, here is pi - error: {pi - MAXIMUM_LT_ERROR}")
            else:
                rospy.loginfo(f"into else, sending null angular velocity")
                data_to_send = 0.0
                yaw_done = True
            '''
            time = leg_tracker_time
            ar_angle = leg_track_ar_angle
            normalized_angle = abs(ar_angle)/pi
            if(math.pi - abs(ar_angle) > MAXIMUM_LT_ERROR and time >= start_mov_time - LT_HISTORY_PARAMETER):
                #apply correction
                data_to_send = speed * math.copysign(1, ar_angle) * direction * normalized_angle
                rospy.loginfo(f"I am using rotate_new, sending angular speed equal to : {data_to_send}")
                rospy.loginfo(f"Ar_angle in rotate_new leg_tracker is : {ar_angle}")
            else:
                data_to_send = 0.0
                yaw_done = True
            '''
        data_to_send = round(data_to_send, 2)
        noise = np.sign(np.random.uniform(-1, 1)) * np.random.uniform(0.3, 1)
        rand_tremble = noise if tremble else 0
        data_to_send = data_to_send + rand_tremble
        return yaw_done, data_to_send




if __name__ == '__main__':
    rospy.init_node("cmd_vel_manager")
    server = CmdVelAction(rospy.get_name())
    rospy.spin()
