Defining ROS topic patterns for the Next Generation of Open-RMF and looking for feedback from the community.

Current ideas include:

Agent-specific topics will begin with a namespace that provides a unique ID for that agent, e.g. the destination topic for delivery_bot_1 will be /delivery_bot_1/destination
Groups allow for nested namespaces, e.g. robot_1 belonging to the security_bots group will own the topic /security_bots/robot_1/destination.
Components or third-party extensions related to a topic will be appended at the end, e.g. the goal (but not immediate) destination of delivery_bot_1 will be published to /delivery_bot_1/destination/goal
Certain pre-defined topic components may always exist, like /destination/discovery and -/destination/errors
Most messages will contain a UUID to represent a session ID, tying together activities that are spread across multiple topics.
Background
In the original implementation of Open-RMF, the APIs for integrating into the framework were largely based around C++ and Python with some JSON interfaces for web integration. The C++ APIs were spread across many libraries like rmf_traffic, rmf_battery, and rmf_task with yet more libraries to manage a ROS 2 layer to connect these libraries all together. In theory it was meant to be possible for users to choose how they integrate these libraries together for the particular needs of their deployments, but in practice it is very difficult to put together the kind of highly concurrent reactive system that’s needed to tie these libraries together.

Ultimately the vast majority of users end up doing their integration through rmf_fleet_adapter or (more commonly) its python bindings. These libraries provide a narrow, stable API that takes care of wrangling all the other framework components into a coherent reactive system. However this leads to a highly vertical integration where users have very limited opportunity to customize behavior or opt in and out of various features.

There are two particular reasons that Open-RMF was originally designed this way:

In the early years of Open-RMF, we encountered many issues while using ROS 2 that essentially boiled down to scaling issues in the discovery mechanism of DDS. We needed to put a substantial amount of complex business logic on top of the traditional ROS 2 pub/sub in order to compensate for these issues.
At the start, we didn’t know exactly what would be needed out of our pub/sub interfaces, and we wanted the freedom to expand on those interfaces over time without creating code instability for users. Maintaining a stable API is much more tractable with C++ and Python than it is with ROS interfaces.
At the very least (2) was a success. In the 6+ years that the project has existed, we have never needed to break the user API, even as we have added on massive features that we never originally planned for. But overall, for Open-RMF to evolve into a project that serves the scope of users that we want it to, and which nurtures diverse integration solutions instead of funneling systems into a narrow profile, we need to dismantle our vertical stack and make it into a broad foundation.

Two important things have changed since the start of the Open-RMF project:

The ROS 2 community has addressed many of the problems we encountered with using DDS as a middleware. There are discovery servers and vendor-specific configurations that can be used to address the scalability issues that we ran into. There is also rmw_zenoh as a viable alternative middleware layer which seems to scale more easily to large numbers of topics.
We have learned a great deal about what Open-RMF really needs to be, what interfaces it needs to provide, and how it may be structured in order to meet the needs of those who can benefit from it.
Leveraging these two advantages, our goal for the Next Generation of Open-RMF is to use ROS 2 topics, services, and actions as the primary basis for integrating with Open-RMF components. We will be designing these interfaces in the open via discourse and other formal channels meant to gather stakeholder feedback. We hope that these open, community-oriented, feedback-driven processes will help us design interfaces that meet the needs of all stakeholders that see value in Open-RMF. We also hope that this process will help us keep the interfaces as stable and future-proof as possible.

This post is meant to introduce some of the broad ideas we have around how topics, services, and actions might be structured as part of this Next Generation initiative. Future posts will get into specific categories of interfaces, such as traffic management and task management. We would appreciate feedback on these broad ideas while we flesh out the details of the interfaces that will be used for specific capabilities.

Namespacing
Since Open-RMF deals primarily with multi-agent systems, we will need clear and consistent conventions around namespacing.

A common practice in ROS is to use namespaces to isolate sub-systems from each other when those systems may have overlapping topic names that could contaminate each other if they are being used by different agents at the same time.

For example, a mobile robot might have a topic named /map that contains 2D occupancy grid data. Different mobile robots may want to use different occupancy data at the same time because of differences in their sizes or shapes or where they are allowed to travel. Therefore if you have multiple mobile robots in one ROS system, you might prepend a unique namespace to this topic for the name of each mobile robot. For example, /delivery_bot_1/map will be for the robot named delivery_bot_1 while /delivery_bot_2/map will be for delivery_bot_2, and neither topic will interfere with the other.

We will follow this convention for agent-specific topics, for example topics that allow an agent to report its current destination, the path it’s following, or the task that it’s performing. A base topic name will be defined based on the purpose of the topic, and then a unique identifier for the agent will be prepended to it as a namespace.

Rationale
Historically Open-RMF used string robot_name fields across virtually all messages to keep track of where a message was coming from. This was done to minimize how many distinct topics are needed, which helped make DDS discovery scaling more manageable. However, this came with some crucial disadvantages:
The additional string field adds unnecessary bandwidth to each message.
It is harder to route the messages where they need to go. Virtually every subscription callback in the rmf_fleet_adapter library needs to filter by the string fields inside these messages.
Some quality of service parameters, like history depth, became sensitive to how many devices you need to support in your deployment since all devices will be publishing to the same topic. Giving each agent its own topic means history depth can generally be 1 across most publishers and subscribers without any fear of losing important information.
Using this convention makes it straightforward for independent agents to treat the Open-RMF topics as their own internal topic and then apply normal ROS namespacing practices to make it fit the convention expected by Open-RMF.
Some middleware implementations, such as Zenoh, can subscribe to topic patterns, e.g. /**/destination/ to listen for topic updates from all agents at once. This feature is not (yet) supported by ROS, but may be useful for the middleware implementations that can take advantage of it.
Groups
Guaranteeing unique names for every agent in a system can pose a challenge, so we will support a “grouping” pattern by allowing nested namespaces for individual agents. Open-RMF deployments often involve multiple fleets of mobile robots where each fleet is managed by a different vendor. If the two vendors can agree on one unique “group name” for each of them to own, then they no longer have to worry about name clashes for their devices.

For example, if a deployment has one fleet of delivery robots and one fleet of security robots, then each fleet can be assigned a unique group name, such as delivery_bots and security_bots. That group name will then be prepended to the namespace of Open-RMF topics for the agents in that group, allowing the groups to duplicate agent names without fear of collision. For example if both fleets happen to name one of their robots robot_1 then the map topic for each will be /delivery_bots/robot_1/map and /security_bots/robot_1/map, avoiding a collision despite having an agent with the same name.

Groups can be nested inside each other as needed. For example suppose there are multiple buildings in a deployment and multiple vendors. You can have namespaces for both the building and the vendor, for example /tower_1/security_bots/robot_3/map.

Topic Components / Extensions
One of the most crucial goals for Next Generation Open-RMF is to be modular and extensible. The ROS IDL does not currently support extension points inside of message definitions, so instead we will use topic name conventions as a way to define modular components and possible extension points for Open-RMF interfaces.

For example, suppose Open-RMF defines a -/destination topic that lets the operators know where the robot’s immediate destination is. Robot destinations are an important thing for operators to know, so this topic is likely to be mandatory for all robots integrated into the Open-RMF system. However, some robots might want help from Open-RMF to decide their current destination. There may be external considerations to be made before a robot approaches a certain destination, such as whether other robots are simultaneously trying to reach that same destination, or if another robot is already occupying it.

As a component of the mandatory -/destination topic we can also define a -/destination/goal topic. A robot can post what destination(s) it would like to reach, and then an external system can evaluate whether that destination is currently available and then publish to the -/destination topic on that robot’s behalf, either to temporarily send the robot to a waiting area or to send the robot to its goal. This -/destination/goal component would be optional, and vendors would choose to use it or not depending on how their system would best integrate with the overall Open-RMF system.

The term “components” will apply to sub-topics following this pattern that are officially defined by the Open-RMF project, whereas the term “extensions” will apply to sub-topics defined by third-parties.

-/errors component
In complex distributed physical systems, errors are inevitable. If a robot posts a -/destination/goal that is unknown or that can never be reached, it is important to convey that information back to the robot that posted it, and also allow that information to be discovered by operators. Recognizing this, most (if not all) topics and their components will additionally have an -/errors component.

Any given foo/errors topic is directly reporting to one or more errors that occurred due to the value that is currently posted on foo. For example -/destination/goal/errors will likely be published by a server that that listens to -/destination/goal posts and evaluates where the robot should be sent. If a destination cannot be determined due to the value that was posted to -/destination/goal, then the server will publish an error describing the problem to -/destination/goal/errors. This is separate from -/destination/errors which would likely be published by a path planner that is unable to find a route from the robot’s current location to its most recently posted -/destination.

-/discovery component
ROS currently does not have the ability to subscribe to topic patterns, which creates some friction for the per-agent namespacing pattern mentioned in the previous section. To accommodate this, each topic will have a -/discovery component which groups can use to advertise the namespaces of the devices they provide that will be publishing and/or subscribing to the relevant topic.

For the sake of efficiency, it is expected that a single transient local -/discovery message will be published for each independent group of devices. For example, the delivery_bots fleet should publish a single persistent message to /destination/discovery to advertise all the mobile robots in its delivery_bots group.

It will generally be assumed that all agents mentioned in /foo/discovery are also applicable to all components and extensions of -/foo, but this is not always guaranteed to be the case.

Session IDs
There are many types of activities within an Open-RMF system that need to be kept consistent across multiple topics simultaneously. For example, if you have one system that sets a destination on the -/destination topic and another that calculates a path and publishes it to -/path, it’s important to know that the latest -/path value is related to the latest -/destination value and not a late-arriver for a previous -/destination.

To prevent that ambiguity, we will use unique_identifier_msgs/UUID in all message definitions that are part of a multi-topic activity.

In the original implementation of Open-RMF we variously used strings or integers to represent sessions, but both had drawbacks. Strings are usually based on the unique name of a single agent, which means one agent cannot have multiple independent ongoing sessions for any activity where a string value is being used. Integers are very memory efficient but are not robust to crashes where the source-of-truth node needs to be restarted. UUIDs solve both of these problems.

So when a robot posts a goal to -/destination/goal it will contain a UUID to uniquely identify the session that the goal belongs to. Then the destination server will publish to the -/destination topic with a session UUID that matches that -/destination/goal that it is intended for. After that the path server will publish a path to the -/path topic that also contains the UUID based on the -/destination message.

Version number
Sometimes a message needs to go through several iterations or stages while serving the same session, and it may be important to distinguish between those iterations. In that case, the message may contain a uint32 version field in addition to its session field. For each new session value, the version value will start at 0 and increment upwards from there.

In general, version numbers are expected to be short-lived, scoped to a specific component, and potentially incrementing at a rapid rate, whereas session numbers are more long-lived and shared across many components of an activity.

-/session_refresh component
If a node crashes and needs to be restarted, it is expected to publish a message to the -/session_refresh component topic of any potentially impacted topic names. That will allow the system to reset itself and restore internal consistency. In some cases the time needed to restart all sessions may be disruptive, so this should only be used when needed.

For example, a path server may listen to -/destination topics in order to publish to a -/path topic where the paths have version numbers that increment as the robots get rerouted based on changing destinations or changing traffic conditions. If the path server crashes, it should post a message to -/destination/session_refresh for all agents that it is responsible for. That will prompt those agents to begin a new session for their -/destination messages. Then the path server can begin incrementing its version numbers back up from 0 without a risk that the unique (session, version) value of its messages will conflict with its earlier messages.

In the unlikely event that an extremely long-running session causes a version number to overflow, a -/session_refresh can be used for that specific agent to allow the version count to return to zero without needing to worry about handling integer wrap-around logic elsewhere in the system.

Singular vs Plural Topic Names
English grammar is inconsistent even at the best of times, but as a general rule, topic names will take a singular form when they are providing something where only the most recent message is relevant for the system to function as intended, and will take a plural form if it is important to receive all or most messages that are being published on the topic.

With this rule, the name of the topic should give a rough indication of whether a history depth of 1 is sufficient for the quality of service or if a longer history depth is advisable. There may be exceptions to this pattern, and the importance of history depth may be situational between different deployments, but the singular vs plural form of the topic name should provide at least a context clue and sanity check for the expected behavior of the topic.
