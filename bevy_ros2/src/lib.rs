/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

use bevy::prelude::*;
use futures::Future;
use rclrs::{
    ActionClientState, ActionIDL, ActionServerState, ClientState, Context, CreateBasicExecutor,
    ExecutorCommands, IntoPrimitiveOptions, MessageIDL, NodeState, PublisherState, RclrsError,
    RequestedGoal, RequestedGoalClient, ServiceIDL, SpinOptions, Subscription, TerminatedGoal,
};
use std::{
    fmt::Debug,
    sync::{Arc, Mutex},
    thread,
};

#[derive(Resource, Deref)]
pub struct RclrsNode(Arc<NodeState>);

impl FromWorld for RclrsNode {
    fn from_world(world: &mut World) -> Self {
        let executor_commands = world.resource::<RclrsExecutorCommands>();
        let node_name = "nav_traffic_node".to_string();
        let node = executor_commands.create_node(&node_name).unwrap();
        RclrsNode(node.clone())
    }
}

#[derive(Resource, Deref)]
pub struct RclrsExecutorCommands(Arc<ExecutorCommands>);

#[derive(Default)]
pub struct RclrsPlugin {}

impl Plugin for RclrsPlugin {
    fn build(&self, app: &mut App) {
        let mut executor = Context::default_from_env().unwrap().create_basic_executor();
        app.insert_resource(RclrsExecutorCommands(Arc::clone(executor.commands())));

        thread::spawn(move || {
            let r = executor.spin(SpinOptions::default());
            for err in r {
                error!("An error occurred in rclrs: {err}");
            }
        });

        app.init_resource::<RclrsNode>();
    }
}

// Template for creating ROS 2 subscribers
pub struct RosSubscription<T: MessageIDL + Debug> {
    _subscriber: Arc<Subscription<T>>,
    data: Arc<Mutex<Option<T>>>,
}

impl<T: MessageIDL + Debug> RosSubscription<T> {
    pub fn new(node: &Arc<NodeState>, topic: String) -> Self {
        let data = Arc::new(Mutex::new(None));
        let data_clone: Arc<Mutex<Option<T>>> = Arc::clone(&data);

        let subscriber = node
            .create_subscription(
                topic.clone().reliable().transient_local().keep_last(10),
                move |msg: T| {
                    debug!("Received a message on [{}]: {:?}", topic, msg);
                    *data_clone.lock().unwrap() = Some(msg.clone());

                    // TODO(@xiyuoh) allow customized qos
                },
            )
            .unwrap();

        Self {
            _subscriber: Arc::new(subscriber),
            data,
        }
    }

    pub fn data_callback(&self) -> Option<T> {
        self.data.lock().unwrap().as_ref().cloned()
    }
}

// Template for creating ROS 2 publishers
pub struct RosPublisher<T: MessageIDL + Debug> {
    publisher: Arc<PublisherState<T>>,
}

impl<T: MessageIDL + Debug> RosPublisher<T> {
    pub fn new(node: &Arc<NodeState>, topic: String) -> Self {
        let publisher = node.create_publisher(&topic).unwrap();

        Self { publisher }
    }

    pub fn new_transient_local(node: &Arc<NodeState>, topic: String) -> Self {
        let publisher = node
            .create_publisher(topic.reliable().transient_local().keep_last(10))
            .unwrap();

        Self { publisher }
    }

    pub fn publish(&self, msg: T) -> Result<(), RclrsError> {
        self.publisher.publish(msg)
    }
}

// Template for creating ROS 2 service clients
pub struct RosServiceClient<T: ServiceIDL> {
    client: Arc<ClientState<T>>,
}

impl<T: ServiceIDL> RosServiceClient<T> {
    pub fn new(node: &Arc<NodeState>, service_name: String) -> Self {
        let client = node.create_client::<T>(&service_name).unwrap();

        Self { client }
    }
}

// Template for creating ROS 2 action clients
pub struct RosActionClient<T: ActionIDL> {
    action_client: Arc<ActionClientState<T>>,
    action_name: String,
}

impl<T: ActionIDL> RosActionClient<T> {
    pub fn new(node: &Arc<NodeState>, action_name: String) -> Self {
        let action_client = node.create_action_client::<T>(&action_name).unwrap();

        Self {
            action_client,
            action_name,
        }
    }

    pub fn request_goal(&self, goal: T::Goal) -> RequestedGoalClient<T> {
        self.action_client.request_goal(goal)
    }
}

// Template for creating ROS 2 action servers
pub struct RosActionServer<A: ActionIDL> {
    action_server: Arc<ActionServerState<A>>,
    action_name: String,
}

impl<A: ActionIDL> RosActionServer<A> {
    pub fn new<Task>(
        node: &Arc<NodeState>,
        action_name: String,
        callback: impl FnMut(RequestedGoal<A>) -> Task + Send + Sync + 'static,
    ) -> Self
    where
        Task: Future<Output = TerminatedGoal> + Send + Sync + 'static,
    {
        let action_server = node.create_action_server(&action_name, callback).unwrap();

        Self {
            action_server,
            action_name,
        }
    }
}
