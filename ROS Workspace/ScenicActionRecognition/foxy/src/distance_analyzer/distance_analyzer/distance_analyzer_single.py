
from typing import Optional

import rclpy
from common.constants import *
from rclpy.node import Node
from std_msgs.msg import String, Bool

from proxemics_analizer.ProssemicAnalyzer import ProssemicAnalyzer
from proxemics_analizer.ProxemicData import ProxemicData
from proxemics_analizer.WindowProssemicAnalyzer import WindowProssemicAnalyzer
from proxemics_analizer.ProxemicAnalysisHandler import ProxemicAnalysisHandler
from impro_msgs.msg import ProxemicZone, ProxemicMovement, ActorPosition

from bridge_msgs.msg import PersonArray

"""
here we have the 'problem' that multiple people could be tracked.
The module already gives us a tracking id for every person.
We can have an object that collects data for each individual and then prunes those 
who didn't receive enough data (maybe look at percentage of collected timesteps and length of data)
"""

class PeopleTrackingReceiver(Node, ProxemicAnalysisHandler):
    prossemic_analyzer: Optional[ProssemicAnalyzer] = None

    __enabled: Bool

    def __init__(self):
        super().__init__("people_tracking_receiver")
        super(Node, self).__init__()

        self.__enabled = True

        # Subscribe to the people tracker module
        self.tracker_sub = self.create_subscription(
            # TODO: use message from people tracker
            PersonArray,
            "people_tracked",
            self.on_tracking_received,
            10)

        # Then subscribe to the control signals
        self.control_sub = self.create_subscription(
            Bool,
            "is_moving",
            self.on_control_signal,
            10
        )

        # Create the analyzer passing ourselves as the handler
        self.prossemic_analyzer = WindowProssemicAnalyzer(self)
        # And create the publishers
        self.__prox_zone_publisher = self.create_publisher(ProxemicZone, PROX_ZONE_TOPIC, 10)
        self.__actor_position_publisher = self.create_publisher(ActorPosition, ACTOR_POSITION_TOPIC, 10)
        self.__prox_mvmt_publisher = self.create_publisher(ProxemicMovement, PROX_MVMT_TOPIC, 10)

        # NEW: HERE I SAVE THE LAST POSITION OF THE HUMAN ACTOR
        self.person_detected = {}


    def on_tracking_received(self, data):
        """Handle data received from people tracking module

        Args:
            data ([PersonArray ROS message]): Message containing the pose of every person tracked

            NEW: here we will directly publish pos_x and pos_y on person detected
        """
        
        # Check if input is enabled
        if self.__enabled:

            ##self.get_logger().info('I heard tracker result')

            if len(data.people) == 0:
                ##self.get_logger().info("No person detected")
                # Here ignore data
                self.prossemic_analyzer.on_no_person_detected()
            else:
                #self.get_logger().info(f"{len(data.people)} people detected")
                person = data.people[0]
                ##self.get_logger().info(f"Handling person {person.id}")
                # Create an object representing prossemic data for person
                ##self.get_logger().info(f"Received coordinate x: {person.pose.position.x}, y: {person.pose.position.y}")
                prossemic_data = ProxemicData(
                    person.pose.position.x,
                    person.pose.position.y
                )
                # And add to tracked values
                self.prossemic_analyzer.on_data_received(prossemic_data)

                #NEW 
                #self.person_detected["position_x"] = person.pose.position.x
                #self.person_detected["position_y"] = person.pose.position.y
                actor_position = ActorPosition()
                actor_position.actor_x = person.pose.position.x
                actor_position.actor_y = person.pose.position.y
                self.__actor_position_publisher.publish(actor_position)

        else:
            #self.get_logger().info("Ignored incoming data, node is disabled")
            pass

    def on_control_signal(self, msg):
        self.__enabled = not msg.data
        # Here we are interested in messages to trigger our turn
        if msg.data:
            # Here the robot started moving, hence we should clear data collected so far 
            self.prossemic_analyzer.flush_collected_data()

    ''' proxemic movement and proxemic_zone are published each 0.5 sec, actor position now will be published directly in on_tracking_received'''
    def handle_proxemic_data(self, result) -> None:
        # Create the message objects
        #actor_position = ActorPosition()
        #actor_position.actor_position = result.actor_position
        #actor_position.actor_x = self.person_detected["position_x"]
        #actor_position.actor_y = self.person_detected["position_y"]

        proxemic_zone = ProxemicZone()
        proxemic_zone.zone = result.zone
        # TODO: Check whether real-valued distance is needed

        proxemic_movement = ProxemicMovement()
        proxemic_movement.linear_movement = result.linear_movement
        proxemic_movement.lateral_movement = result.lateral_movement
        proxemic_movement.linear_speed = result.linear_speed
        proxemic_movement.angular_speed = result.angular_speed

        self.__prox_mvmt_publisher.publish(proxemic_movement)
        self.__prox_zone_publisher.publish(proxemic_zone)
        #self.__actor_position_publisher.publish(actor_position)


def main(args=None):
    rclpy.init(args=args)

    receiver = PeopleTrackingReceiver()
    rclpy.spin(receiver)


if __name__ == "__main__":
    main()
