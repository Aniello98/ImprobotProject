# Map creation instructions for robocchio:
- launch chiroli_start1.sh and chiroli_start2.sh commenting in slam.launch the map_server node launch
- launch gmapping node from the triskar_navigation pkg
- control the robot to acquire data
- save the created map with "rosrun map_server map_saver -f <MAP NAME>   the map will be saved in the current directory
