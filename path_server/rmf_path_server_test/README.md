This package provides some simple test scripts for the path server.

The first test we have is a simple we have two agents publish their appearance (using rmf_mock_robot_sim). One starts at (0,0), the other at (3,0). We run the path server. We publish potential destinations of (5,0) and (3,0) for each agent. The mock robot_sim_agents should receive their path and head towards their goal.