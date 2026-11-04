use log::debug;
use rclrs::{
    IntoPrimitiveOptions, MessageIDL, NodeState, PublisherState, RclrsError, Subscription,
};
use std::{
    collections::VecDeque,
    fmt::Debug,
    sync::{Arc, Mutex},
};

pub const DEFAULT_SUBSCRIPTION_QUEUE_LEN: usize = 64;

pub struct RosSubscription<T: MessageIDL + Debug> {
    _subscriber: Arc<Subscription<T>>,
    data: Arc<Mutex<VecDeque<T>>>,
    max_queue_len: usize,
}

impl<T: MessageIDL + Debug> RosSubscription<T> {
    pub fn new(node: &Arc<NodeState>, topic: String) -> Self {
        Self::with_capacity(node, topic, DEFAULT_SUBSCRIPTION_QUEUE_LEN)
    }

    pub fn with_capacity(node: &Arc<NodeState>, topic: String, max_queue_len: usize) -> Self {
        let max_queue_len = max_queue_len.max(1);
        let data = Arc::new(Mutex::new(VecDeque::with_capacity(max_queue_len)));
        let data_clone: Arc<Mutex<VecDeque<T>>> = Arc::clone(&data);

        let subscriber = node
            .create_subscription(
                topic.clone().reliable().transient_local().keep_last(10),
                move |msg: T| {
                    debug!("Received a message on [{}]: {:?}", topic, msg);
                    let mut queue = data_clone.lock().unwrap();
                    while queue.len() >= max_queue_len {
                        queue.pop_front();
                    }
                    queue.push_back(msg);
                },
            )
            .unwrap();

        Self {
            _subscriber: Arc::new(subscriber),
            data,
            max_queue_len,
        }
    }

    pub fn max_queue_len(&self) -> usize {
        self.max_queue_len
    }

    pub fn data_callback(&self) -> Option<T> {
        self.data.lock().unwrap().back().cloned()
    }

    pub fn take_data(&self) -> Option<T> {
        self.data.lock().unwrap().pop_front()
    }
}

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
