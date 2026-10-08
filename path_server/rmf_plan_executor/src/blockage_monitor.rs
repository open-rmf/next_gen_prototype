// Copyright 2026 OSRA
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

use ros_env::nav_msgs::msg::OccupancyGrid;
use ros_env::rmf_prototype_msgs::msg::PlanId;
use std::time::{Duration, Instant};

pub(crate) const BLOCKAGE_DEBOUNCE: Duration = Duration::from_millis(300);
pub(crate) const REPLAN_COOLDOWN: Duration = Duration::from_secs(2);
const OCCUPIED_THRESHOLD: i8 = 50;

#[derive(Default)]
pub(crate) struct BlockageMonitor {
    blocked_since: Option<Instant>,
    reported_plan: Option<PlanId>,
    last_reported_at: Option<Instant>,
}

impl BlockageMonitor {
    pub(crate) fn begin_plan(&mut self) {
        self.blocked_since = None;
    }

    pub(crate) fn observe(&mut self, blocked: bool, plan_id: &PlanId, now: Instant) -> bool {
        if !blocked {
            self.blocked_since = None;
            return false;
        }

        if self.reported_plan.as_ref() == Some(plan_id) {
            return false;
        }

        let blocked_since = self.blocked_since.get_or_insert(now);
        if now.duration_since(*blocked_since) < BLOCKAGE_DEBOUNCE {
            return false;
        }

        if self
            .last_reported_at
            .is_some_and(|last| now.duration_since(last) < REPLAN_COOLDOWN)
        {
            return false;
        }

        self.reported_plan = Some(plan_id.clone());
        self.last_reported_at = Some(now);
        self.blocked_since = None;
        true
    }
}

pub(crate) fn route_intersects_map(map: &OccupancyGrid, route: &[(f32, f32)], radius: f32) -> bool {
    if route.len() < 2 || map.info.resolution <= 0.0 {
        return false;
    }

    let width = map.info.width as isize;
    let height = map.info.height as isize;
    if width == 0 || height == 0 {
        return false;
    }

    let resolution = map.info.resolution;
    let q = &map.info.origin.orientation;
    let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z)) as f32;
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();
    let origin_x = map.info.origin.position.x as f32;
    let origin_y = map.info.origin.position.y as f32;
    let clearance = radius.max(0.0) + resolution * std::f32::consts::FRAC_1_SQRT_2;
    let clearance_squared = clearance * clearance;

    let to_map = |(x, y): (f32, f32)| {
        let dx = x - origin_x;
        let dy = y - origin_y;
        (cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy)
    };

    for segment in route.windows(2) {
        let start = to_map(segment[0]);
        let end = to_map(segment[1]);
        let min_x = (((start.0.min(end.0) - clearance) / resolution).floor() as isize).max(0);
        let max_x =
            (((start.0.max(end.0) + clearance) / resolution).floor() as isize).min(width - 1);
        let min_y = (((start.1.min(end.1) - clearance) / resolution).floor() as isize).max(0);
        let max_y =
            (((start.1.max(end.1) + clearance) / resolution).floor() as isize).min(height - 1);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let index = y as usize * width as usize + x as usize;
                if map.data.get(index).copied().unwrap_or(-1) <= OCCUPIED_THRESHOLD {
                    continue;
                }

                let center = ((x as f32 + 0.5) * resolution, (y as f32 + 0.5) * resolution);
                if distance_squared_to_segment(center, start, end) <= clearance_squared {
                    return true;
                }
            }
        }
    }

    false
}

fn distance_squared_to_segment(point: (f32, f32), start: (f32, f32), end: (f32, f32)) -> f32 {
    let segment = (end.0 - start.0, end.1 - start.1);
    let length_squared = segment.0 * segment.0 + segment.1 * segment.1;
    if length_squared <= f32::EPSILON {
        return (point.0 - start.0).powi(2) + (point.1 - start.1).powi(2);
    }

    let offset = (point.0 - start.0, point.1 - start.1);
    let t = ((offset.0 * segment.0 + offset.1 * segment.1) / length_squared).clamp(0.0, 1.0);
    let closest = (start.0 + t * segment.0, start.1 + t * segment.1);
    (point.0 - closest.0).powi(2) + (point.1 - closest.1).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn occupied_cell_on_remaining_route_is_blocked() {
        let mut map = OccupancyGrid::default();
        map.info.resolution = 1.0;
        map.info.width = 10;
        map.info.height = 10;
        map.info.origin.orientation.w = 1.0;
        map.data = vec![0; 100];
        map.data[5 * 10 + 5] = 100;

        assert!(route_intersects_map(&map, &[(1.5, 5.5), (8.5, 5.5)], 0.25));
        assert!(!route_intersects_map(&map, &[(1.5, 8.5), (8.5, 8.5)], 0.25));
    }

    #[test]
    fn blockage_must_persist_before_reporting() {
        let mut monitor = BlockageMonitor::default();
        let plan_id = PlanId::default();
        let start = Instant::now();

        assert!(!monitor.observe(true, &plan_id, start));
        assert!(!monitor.observe(true, &plan_id, start + BLOCKAGE_DEBOUNCE / 2));
        assert!(monitor.observe(true, &plan_id, start + BLOCKAGE_DEBOUNCE));
        let later = start + BLOCKAGE_DEBOUNCE + REPLAN_COOLDOWN;
        assert!(!monitor.observe(true, &plan_id, later));
    }

    #[test]
    fn clear_route_resets_the_debounce_window() {
        let mut monitor = BlockageMonitor::default();
        let plan_id = PlanId::default();
        let start = Instant::now();

        assert!(!monitor.observe(true, &plan_id, start));
        assert!(!monitor.observe(false, &plan_id, start + BLOCKAGE_DEBOUNCE));
        assert!(!monitor.observe(true, &plan_id, start + BLOCKAGE_DEBOUNCE));
        let before_debounce = start + BLOCKAGE_DEBOUNCE * 3 / 2;
        assert!(!monitor.observe(true, &plan_id, before_debounce));
        assert!(monitor.observe(true, &plan_id, start + BLOCKAGE_DEBOUNCE * 2));
    }

    #[test]
    fn cooldown_delays_a_blockage_on_the_next_plan() {
        let mut monitor = BlockageMonitor::default();
        let first_plan = PlanId::default();
        let second_plan = PlanId {
            plan_version: 1,
            ..Default::default()
        };
        let start = Instant::now();

        assert!(!monitor.observe(true, &first_plan, start));
        assert!(monitor.observe(true, &first_plan, start + BLOCKAGE_DEBOUNCE));
        monitor.begin_plan();
        let during_cooldown = start + BLOCKAGE_DEBOUNCE * 2;
        assert!(!monitor.observe(true, &second_plan, during_cooldown));
        let after_debounce = start + BLOCKAGE_DEBOUNCE * 3;
        assert!(!monitor.observe(true, &second_plan, after_debounce));
        assert!(monitor.observe(
            true,
            &second_plan,
            start + BLOCKAGE_DEBOUNCE + REPLAN_COOLDOWN,
        ));
    }
}
