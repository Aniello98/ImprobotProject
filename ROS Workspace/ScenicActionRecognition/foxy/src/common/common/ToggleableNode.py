from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool



class ToggleableNode(Node):
    """
    An interface describing a node that can be enabled or disabled
    """

    _is_enabled: bool

    def __init__(self, node_name):
        super().__init__(node_name)

        # Initialize the state
        self._is_enabled = True
        # Subscribe to the feedback topic
        self.create_subscription(Bool, "is_moving", self.__on_feedback, 10)

        #self.get_logger().info("Created toggleable node")

    def __on_feedback(self, msg):
        # Here simply call the methods of the interface
        # The enable flag will be accessed in a separate thread to perform analysis
        is_moving = msg.data
        #self.get_logger().info(f"Received feedback message: {is_moving}")
        if is_moving:
             if self._is_enabled:
                self._is_enabled = False
                #self.get_logger().info("Node disabled")
                # And call a method that flushes data collected so far
                self.flush_state_before_disable()
        else:
            # Here the robot is not moving anymore 
            if not self._is_enabled:
                self._is_enabled = True
                #self.get_logger().info("Node enabled")
           
    @property
    def is_enabled(self) -> bool:
        return self._is_enabled
