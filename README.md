# ImprobotProject
Research Project Social Robotics led at the AIRLab (Artificial Intelligence &amp; Robotics Lab) of Politecnico di Milano. 
The aim of the project is to develop a robot system able to improvise in a controlled environment with an actor.

## My contribute

The whole existing system has been done by several Polimi students (Lorenzo Bonetti, Lorenzo Farinelli and Claudia Chiroli) with the supervision of Professor Andrea Bonarini. 
My contribution to the project consists of:
* [ReactionModule](ROS%20Workspace/ReactionModule) improvements in terms of obstacle detection algorithms, concurrency handling of movements, and improvements in navigation algorithms to allow the robot to reach the interlocutor in a more accurate way.
* Design of non-verbal reactions as a set of atomic movements performed by the system, and following validation through the use of a [ReactionsSurvey](ReactionsSurvey)
* Implementation of a behavioural model through Machine Learning techniques. Before applying these algorithms, preprocessing and acquisition of data has been done through the use of an online survey in [ImprovSurvey](ImprovSurvey).
