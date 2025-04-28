### About Files

The following files were used to test certain functionalities before implementation in the main control flow. 

To use the same Gazebo Harmonic simulation, first you have to git clone the ardupilot_gazebo repository.

git clone https://github.com/ArduPilot/ardupilot_gazebo.git

Once cloned, copy the Test_Files/iris_flight_testing.sdf into ardupilot_gazebo/worlds folder and Test_Files/testing folder into ardupilot_gazebo/models.

To run the simulation, within home/path/ardupilot_gazebo: 
gz sim -v4 -r iris_flight_testing.sdf

`iris_flight_testing.sdf` is the world in which the control flow was testing in and features a green field with a circular, red and black blob representing the letter. 
