# antsy_control

Control logic for ANTSY. It uses the antsy_kinematics to perform all the movements shenanigans.
Just run `ros2 launch antsy_control follow_velocity_rectangle.launch.xml` to start reacting to velocity commands.
We can test this with the hello world script `ros2 run antsy_control walk_sideways_while_rotating`. It just sends comand velocities to make ANTSY walk sideways while rotating. I guess the name explains it quite well. 