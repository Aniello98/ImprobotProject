from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    vision_node = Node(
        package="vision",
        executable="cascade_mp",
        output="screen"
    )
    emotion_aggregator = Node(
        package="emotion_aggregator",
        executable="aggregator", 
        output="screen"
    )
    body_emotion = Node(
        package="emo_body",
        executable="detector", 
        output="screen"
    )
    proxemics = Node(
        package="distance_analyzer", 
        executable="dist_an", 
        output="screen"
    )
    stage_grid = Node(
        package="stage_grid",
        executable="grid_comp", 
        output="screen"
    )
    action_classifier = Node(
        package="action_classifier",
        executable="classifier", 
        output="screen"
    )

    ld.add_action(vision_node)
    ld.add_action(body_emotion)
    ld.add_action(proxemics)
    ld.add_action(emotion_aggregator)
    ld.add_action(stage_grid)
    ld.add_action(action_classifier)

    return ld
