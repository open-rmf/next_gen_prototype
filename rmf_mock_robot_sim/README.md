This package contains a simple node that publishes the odom for an individual robot. When robots receive a ~/path on the topic the odom will be updated accordingly.

At startup the robot should also publish itself in the participant_discovery topic (read the details of the spec [here](../../discourse/2-traffic-management-interfaces.md)).