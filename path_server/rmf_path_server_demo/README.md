# rmf_path_server_test package
With reference to the test in test_path_server_scenario,
can you create a simple web dashboard with roslibjs that can be 
served. Include a launch file for starting the rosbridge.
The main ui should be a html canvas where we can add robots at their
start positions and transmit their goals directly to them.
Use the Destination message for the goals and the Odom message for visullizing current robot pose.
We should set up a scenario first then press send. You can use the mock_robot_sim
for initially testing the path server's behaviour. robots can use simple squares. keep things simple.