// Copyright 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use rclrs::{Context, InitOptions};
use std::net::TcpListener;
use std::ops::Deref;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Base TCP port used by ROS 2's `domain_coordinator` (`32768 + domain_id`)
/// to reserve unique `ROS_DOMAIN_ID`s (`1..=100`) across threads and processes.
const DOMAIN_COORDINATOR_PORT_BASE: u16 = 32768;

static NEXT_OFFSET: AtomicUsize = AtomicUsize::new(0);

/// RAII guard that reserves an exclusive ROS 2 domain ID (`1..=100`) for the
/// lifetime of a test and initializes an isolated [`Context`] on that domain
/// via [`InitOptions::with_domain_id`].
///
/// Field order matters: `context` is dropped before `_port_lock` so the DDS
/// domain participant shuts down before the reservation port is released.
pub struct IsolatedContext {
    context: Context,
    _port_lock: TcpListener,
}

impl Deref for IsolatedContext {
    type Target = Context;

    fn deref(&self) -> &Self::Target {
        &self.context
    }
}

pub fn isolated_context() -> Result<IsolatedContext, Box<dyn std::error::Error>> {
    let pid_seed = std::process::id() as usize;
    let offset = NEXT_OFFSET.fetch_add(1, Ordering::Relaxed);
    for i in 0..100 {
        let domain_id = ((pid_seed + offset + i) % 100) + 1;
        let port = DOMAIN_COORDINATOR_PORT_BASE + domain_id as u16;
        if let Ok(listener) = TcpListener::bind(("0.0.0.0", port)) {
            let context = Context::from_env(InitOptions::new().with_domain_id(Some(domain_id)))?;
            return Ok(IsolatedContext {
                context,
                _port_lock: listener,
            });
        }
    }
    Err("Failed to reserve a unique ROS_DOMAIN_ID in 1..=100".into())
}
