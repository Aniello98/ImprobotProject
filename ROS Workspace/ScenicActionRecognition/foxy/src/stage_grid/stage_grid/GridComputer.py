from ast import In
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Int8

class GridComputer(Node):

    def __init__(self):
        super().__init__("stage_grid_computer")

        self.__alive = True

        # ===== SUBSCRIBERS ===== #
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Get TF looup every second
        self.timer = self.create_timer(1.0, self.on_timer)
        
        # Create publisher of classification result
        self.__stage_grid_pub = self.create_publisher(Int8, "stage_grid", 10)

    def on_timer(self):

        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform(
                        "map",
                        "base_link",
                        now)
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        grid_index  = self.__compute_grid_index(x, y)
        # Reply with the classification
        msg = Int8()
        msg.data = grid_index
        self.__stage_grid_pub.publish(msg)

    def __compute_grid_index(self, x, y):
        # Create division in grid
        if 0 <= x < 1.3:
            if 0 <= y < 1.3:
                return 0
            elif 1.3 <= y < 2.7:
                return 3
            else:
                return 6
        elif 1.3 <= x < 2.7:
            if 0 <= y < 1.3:
                return 1
            elif 1.3 <= y < 2.7:
                return 4
            else:
                return 7
        else:
            if 0 <= y < 1.3:
                return 2
            elif 1.3 <= y < 2.7:
                return 5
            else:
                return 8

def main(args=None):
    rclpy.init(args=args)

    action_classifier = GridComputer()
    rclpy.spin(action_classifier)


if __name__ == "__main__":
    main()
