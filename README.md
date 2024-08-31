# ImprobotProject
Research Project in the field of Social Robotics led at the AIRLab (Artificial Intelligence &amp; Robotics Lab) of Politecnico di Milano. 
The project aims to develop a robot system that can improvise in a controlled environment with an actor.

## My contribute

The whole existing system has been done by several Polimi students (Lorenzo Bonetti, Lorenzo Farinelli and Claudia Chiroli) with the supervision of Professor Andrea Bonarini. 

My contribution to the project consists of:
* [ReactionModule](ROS%20Workspace/ReactionModule) improvements in terms of obstacle detection algorithms, concurrency handling of movements, and improvements in navigation algorithms facilitating the robot to reach the interlocutor.
* Design of non-verbal reactions as a set of atomic movements performed by the system, and a following analysis of results through the use of a [ReactionsSurvey](ReactionsSurvey)
* Implementation of a behavioural model through Reinforcement Learning techniques in [RobocchioRL](RobocchioRL). Before applying these algorithms, preprocessing and acquisition of data has been done through the use of an online survey in [ImprovSurvey](ImprovSurvey).

To read the whole summary go to [Executive Summary](Executive_Summary)
