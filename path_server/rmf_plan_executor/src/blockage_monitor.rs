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
    let half_cell = 0.5 * resolution;
    let radius = radius.max(0.0);
    let radius_squared = radius * radius;
    let search_margin = radius + half_cell;

    let to_map = |(x, y): (f32, f32)| {
        let dx = x - origin_x;
        let dy = y - origin_y;
        (cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy)
    };

    for segment in route.windows(2) {
        let start = to_map(segment[0]);
        let end = to_map(segment[1]);
        let min_x =
            (((start.0.min(end.0) - search_margin) / resolution).floor() as isize).max(0);
        let max_x = (((start.0.max(end.0) + search_margin) / resolution).ceil() as isize)
            .min(width - 1);
        let min_y =
            (((start.1.min(end.1) - search_margin) / resolution).floor() as isize).max(0);
        let max_y = (((start.1.max(end.1) + search_margin) / resolution).ceil() as isize)
            .min(height - 1);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let index = y as usize * width as usize + x as usize;
                if map.data.get(index).copied().unwrap_or(-1) <= OCCUPIED_THRESHOLD {
                    continue;
                }

                let cx = x as f32 * resolution;
                let cy = y as f32 * resolution;
                let box_min = (cx - half_cell, cy - half_cell);
                let box_max = (cx + half_cell, cy + half_cell);
                if segment_distance_squared_to_aabb(start, end, box_min, box_max) <= radius_squared
                {
                    return true;
                }
            }
        }
    }

    false
}

fn point_distance_squared_to_aabb(
    point: (f32, f32),
    box_min: (f32, f32),
    box_max: (f32, f32),
) -> f32 {
    let dx = (box_min.0 - point.0).max(0.0).max(point.0 - box_max.0);
    let dy = (box_min.1 - point.1).max(0.0).max(point.1 - box_max.1);
    dx * dx + dy * dy
}

fn segment_intersects_aabb(
    start: (f32, f32),
    end: (f32, f32),
    box_min: (f32, f32),
    box_max: (f32, f32),
) -> bool {
    let mut t_min = 0.0_f32;
    let mut t_max = 1.0_f32;
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;

    for (p, d, min_b, max_b) in [
        (start.0, dx, box_min.0, box_max.0),
        (start.1, dy, box_min.1, box_max.1),
    ] {
        if d.abs() <= f32::EPSILON {
            if p < min_b || p > max_b {
                return false;
            }
        } else {
            let inv_d = 1.0 / d;
            let mut t1 = (min_b - p) * inv_d;
            let mut t2 = (max_b - p) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            t_min = t_min.max(t1);
            t_max = t_max.min(t2);
            if t_min > t_max {
                return false;
            }
        }
    }
    true
}

fn segment_distance_squared_to_aabb(
    start: (f32, f32),
    end: (f32, f32),
    box_min: (f32, f32),
    box_max: (f32, f32),
) -> f32 {
    if segment_intersects_aabb(start, end, box_min, box_max) {
        return 0.0;
    }

    let mut min_dist_sq = point_distance_squared_to_aabb(start, box_min, box_max)
        .min(point_distance_squared_to_aabb(end, box_min, box_max));

    for corner in [
        (box_min.0, box_min.1),
        (box_min.0, box_max.1),
        (box_max.0, box_min.1),
        (box_max.0, box_max.1),
    ] {
        min_dist_sq = min_dist_sq.min(distance_squared_to_segment(corner, start, end));
    }

    min_dist_sq
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
    fn route_adjacent_to_occupied_walls_is_not_blocked() {
        let mut map = OccupancyGrid::default();
        map.info.resolution = 1.0;
        map.info.width = 14;
        map.info.height = 9;
        map.info.origin.position.x = -5.0;
        map.info.origin.position.y = -4.0;
        map.info.origin.orientation.w = 1.0;
        // Mark all cells occupied except 3x8 corridor (x in 3..=10, y in 3..=5) and dock at (4, 6)
        map.data = vec![100; 14 * 9];
        for y in 3..=5 {
            for x in 3..=10 {
                map.data[y * 14 + x] = 0;
            }
        }
        map.data[6 * 14 + 4] = 0;

        // Route from corridor (-2.0, 0.0) -> (-1.0, 0.0) -> (-1.0, 1.0) -> dock (-1.0, 2.0) with radius 0.49
        let route = [(-2.0, 0.0), (-1.0, 0.0), (-1.0, 1.0), (-1.0, 2.0)];
        assert!(!route_intersects_map(&map, &route, 0.49));
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
