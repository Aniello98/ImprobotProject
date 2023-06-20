import json

import rclpy
from impro_msgs.msg import ActorPosition, ScenicAction
from rclpy.node import Node
from std_msgs.msg import String

ACTIONS = [
    "attack",
    "intimidation",
    "refuse",
    "grudge",
    "scolding",
    "disappointment",
    "sad_person",
    "share_sadness",
    "share_fear",
    "share_surprise",
    "share_joy",
    "greet",
    "happiness_person",
    "satisfaction",
    "caution",
    "disbelief",
    "hesitancy",
    "perplexity",
    "astonishment",
    "shock",
    "escape",
    "none"
]

INTEGRATED_ACTIONS = {
    "attack" : "attack", 
    "intimidation" : "intimidate",
    "refuse" : "disappointment", 
    "grudge" : "grudge", 
    "scolding" : "scolding", 
    "disappointment" : "disappointment",  
    "sad_person" : "sharing_sadness", 
    "share_sadness" : "sharing_sadness", 
    "share_fear" : "sharing_fear", 
    "share_surprise" : "surprise", 
    "share_joy" : "sharing_happiness", 
    "greet" : "sharing_happiness",
    "happiness_person" : "happy_person", 
    "satisfaction" : "satisfaction",  
    "caution" : "sharing_fear", 
    "disbelief" : "disbelief", 
    "hesitancy" : "sharing_fear", 
    "perplexity" : "surprise", 
    "astonishment" : "astonishment", 
    "shock" : "sharing_fear", 
    "escape" : "running_away", 
    "none" : "none"
}


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('my_node')
        self.ai_publisher = self.create_publisher(String, '/my_actor_info', 10)    # publishes ros1 json
        self.pos_publisher = self.create_publisher(String, '/actor_pos_info', 10)    # publishes ros1 json with actor position infos
        
        self.subscription_1 = self.create_subscription(ActorPosition, '/actor_position', self.position_callback, 10) # listens for actor pos (Lorenzo)
        self.subscription_2 = self.create_subscription(ScenicAction, '/scenic_action', self.action_callback, 10)       # listens for scenic action (Lorenzo)

        self.info = {}
        self.update_pos = False
        self.update_act = False
        
        self.timer = self.create_timer(0.25, self.publish)   # check new data every 0.25 seconds
        self.get_logger().info("MinimalPublisher started, waiting for messages...")


    def position_callback(self, msg):
        self.get_logger().info(f"I received {msg}")
        self.info["actorPosX"] = msg.actor_x
        self.info["actorPosY"] = msg.actor_y
        self.update_pos = True
        pos_info = {}
        pos_info["actorPosX"] = msg.actor_x
        pos_info["actorPosY"] = msg.actor_y    
        msg = String()
        msg.data = json.dumps(pos_info)
        self.pos_publisher.publish(msg)    


    def action_callback(self, msg):
        self.get_logger().info(f"I received {msg} associated to action {ACTIONS[msg.action]}")
        # NEW: send the new set of actions
        action = ACTIONS[msg.action]
        self.get_logger().info(f"I will wrap it as {INTEGRATED_ACTIONS[action]}")
        self.info["scenicAction"] = INTEGRATED_ACTIONS[action]
        self.update_act = True


    def publish(self):
        if not (self.update_pos and self.update_act):
            return
        msg = String()
        msg.data = json.dumps(self.info)
        self.ai_publisher.publish(msg)
        self.update_pos = False
        self.update_act = False
        self.get_logger().info(f"I published {msg}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
