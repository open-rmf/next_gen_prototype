use crate::{
    testing::{RequestPlan, SafeZoneReceived},
    AmclPose, CurrentSafeZone, Nav2Agent, NavigationCompleted, RclrsNode, RosSubscription,
};
use bevy::prelude::*;
use crossflow::{prelude::*, service::Service};
use reqwest::blocking::Client;
use ros_env::{
    nav2_msgs::msg::Costmap,
    rmf_prototype_msgs::msg::{
        DestinationConstraints, PlanId, Region, SafeZone, SafeZoneId, TargetOrientation,
        TargetRegion,
    },
    unique_identifier_msgs::msg::UUID as RosUuid,
};
use rosidl_runtime_rs::BoundedSequence;
use serde::{Deserialize, Serialize};
use std::{collections::HashSet, sync::Arc};
use uuid::Uuid;

#[derive(Clone, Debug)]
pub struct AgentRequest {
    agent: Entity,
    request: AgentPoseRequest,
}

#[derive(Resource)]
pub struct MapfPostClient {
    client: Client,
    url: String,
}

// --- API Request/Response Structs from mapf_post ---
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentPoseRequest {
    pub agent_id: usize,
    pub x: f32,
    pub y: f32,
    pub angle: f32, // Added angle for Isometry2
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentAllocationResponse {
    pub agent_id: usize,
    pub cell_size: f32,
    pub allocated_free_space: Vec<(f32, f32)>,
    pub next_goal: (f32, f32),
    pub remaining_traj: Vec<(f32, f32)>,
}

#[derive(Component)]
pub struct CostmapSubscription {
    pub subscriber: Arc<RosSubscription<Costmap>>,
}

#[derive(Default)]
pub struct MapfBridgePlugin {}

impl Plugin for MapfBridgePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MapfPostClient {
            client: Client::new(),
            url: "http://127.0.0.1:3000/update_pose".into(),
        })
        .add_observer(on_request_new_plan)
        .add_observer(create_costmap_subscriber);

        let allocation_services = MapfAllocationServices::from_app(app);
        app.insert_resource(allocation_services);
    }
}

fn create_costmap_subscriber(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/inner/global_costmap/costmap_raw";
    let subscription = Arc::new(RosSubscription::<Costmap>::new(&node, topic.clone()));
    commands.entity(e).insert(CostmapSubscription {
        subscriber: Arc::clone(&subscription),
    });
}

#[derive(Resource)]
pub struct MapfAllocationServices {
    allocation: Service<Entity, (), ()>,
}

impl MapfAllocationServices {
    pub fn from_app(app: &mut App) -> Self {
        let request_mapf_plan_service = app.spawn_continuous_service(Update, request_mapf_plan);
        let convert_to_safe_zone_service = app.spawn_service(mapf_plan_to_safe_zone);
        let update_safe_zone_service = app.spawn_service(update_safe_zone);

        let allocation = app.world_mut().spawn_workflow(|scope, builder| {
            let request = builder
                .chain(scope.start)
                .then_node(request_mapf_plan_service);

            let convert_to_safe_zone = builder.create_node(convert_to_safe_zone_service);
            let update_safe_zone = builder.create_node(update_safe_zone_service);

            builder.connect(request.streams, convert_to_safe_zone.input);

            let (convert_fork_result_input, convert_fork_result) = builder.create_fork_result();
            builder.connect(convert_to_safe_zone.output, convert_fork_result_input);
            builder.connect(convert_fork_result.ok, update_safe_zone.input);

            builder.connect(request.output, scope.terminate);
        });

        Self { allocation }
    }
}

fn request_mapf_plan(
    srv: ContinuousService<Entity, (), StreamOf<(AgentRequest, AgentAllocationResponse)>>,
    mut orders: ContinuousQuery<Entity, (), StreamOf<(AgentRequest, AgentAllocationResponse)>>,
    mut nav_completed: EventReader<NavigationCompleted>,
    agent_poses: Query<(&Nav2Agent, &AmclPose)>,
    mapf_post_client: Res<MapfPostClient>,
) {
    let Some(mut orders) = orders.get_mut(&srv.key) else {
        return;
    };
    if orders.is_empty() {
        return;
    }
    let client = mapf_post_client.client.clone();

    // TODO(@xiyuoh) validate using the plan id in this NavigationCompleted event
    let completed_agents: HashSet<Entity> = nav_completed.read().map(|event| event.agent).collect();
    orders.for_each(|order| {
        let request = order.request();

        // If overall navigation has completed, end workflow
        if completed_agents.contains(&request) {
            info!(
                "Agent {:?} has completed navigation! Ending workflow",
                request
            );
            order.respond(());
            return;
        }

        // Create AgentRequest using the current AmclPose
        let Ok((agent, amcl_pose)) = agent_poses.get(*request) else {
            return;
        };
        let pose = amcl_pose.0.pose.pose.clone();
        let agent_request = AgentRequest {
            agent: *request,
            request: AgentPoseRequest {
                agent_id: agent.id as usize,
                x: pose.position.x as f32,
                y: pose.position.y as f32,
                angle: 0.0,
            },
        };

        debug!(
            "Sending POST request payload to {}...",
            mapf_post_client.url
        );

        if let Ok(response) = client
            .post(mapf_post_client.url.clone())
            .json(&agent_request.request)
            .send()
        {
            if response.status().is_success() {
                if let Ok(server_response) = response.json::<AgentAllocationResponse>() {
                    debug!("Response received from mapf_post: {:?}", server_response);
                    order
                        .streams()
                        .send((agent_request.clone(), server_response));
                }
            }
        }
    });
}

fn mapf_plan_to_safe_zone(
    Blocking { request, .. }: Blocking<(AgentRequest, AgentAllocationResponse)>,
    costmap_sub: Query<(&Nav2Agent, &CostmapSubscription)>,
) -> Result<(AgentRequest, SafeZone), ()> {
    let Ok((agent, subscription)) = costmap_sub.get(request.0.agent) else {
        return Err(());
    };

    let Some(latest_costmap) = subscription.subscriber.data_callback() else {
        return Err(());
    };

    let allocation = request.1;
    if allocation.agent_id as i32 != agent.id {
        return Err(());
    }

    let mut costmap = latest_costmap.clone();
    let mut costmap_data = latest_costmap.data.clone();
    let mut safe_indices = HashSet::with_capacity(allocation.allocated_free_space.len());
    let metadata = latest_costmap.metadata;
    let costmap_origin_x = metadata.origin.position.x;
    let costmap_origin_y = metadata.origin.position.y;

    let costmap_res = metadata.resolution as f64;
    let half_cell = (allocation.cell_size / 2.0) as f64;

    // Assign allocated free space to the costmap with a bounding box
    for &(world_x, world_y) in &allocation.allocated_free_space {
        let world_min_x = world_x as f64 - half_cell;
        let world_max_x = world_x as f64 + half_cell;
        let world_min_y = world_y as f64 - half_cell;
        let world_max_y = world_y as f64 + half_cell;

        let start_cell_x = ((world_min_x - costmap_origin_x) / costmap_res).floor() as i32;
        let end_cell_x = ((world_max_x - costmap_origin_x) / costmap_res).floor() as i32;
        let start_cell_y = ((world_min_y - costmap_origin_y) / costmap_res).floor() as i32;
        let end_cell_y = ((world_max_y - costmap_origin_y) / costmap_res).floor() as i32;

        let min_x = start_cell_x.max(0).min(metadata.size_x as i32 - 1);
        let max_x = end_cell_x.max(0).min(metadata.size_x as i32 - 1);
        let min_y = start_cell_y.max(0).min(metadata.size_y as i32 - 1);
        let max_y = end_cell_y.max(0).min(metadata.size_y as i32 - 1);

        for cell_y in min_y..=max_y {
            for cell_x in min_x..=max_x {
                let index = (cell_y * metadata.size_x as i32 + cell_x) as usize;
                safe_indices.insert(index);
            }
        }
    }

    const LETHAL_OBSTACLE: u8 = 254;
    const FREE_SPACE: u8 = 0;
    // Mark any non-free space as LETHAL_OBSTACLE
    for index in 0..costmap_data.len() {
        if safe_indices.contains(&index) {
            costmap_data[index] = FREE_SPACE;
        } else {
            costmap_data[index] = LETHAL_OBSTACLE;
        }
    }
    costmap.data = costmap_data;

    let safe_zone = SafeZone {
        incremental_target: DestinationConstraints {
            regions: vec![TargetRegion {
                tolerance: 0.0,
                region: Region {
                    points: vec![allocation.next_goal.0, allocation.next_goal.1],
                    hint: Region::HINT_POINT,
                },
                orientations: vec![TargetOrientation {
                    orientation_radians: 0.0, // TODO(@xiyuoh)
                    ..default()
                }],
            }],
            nodes: vec![],
        },
        costmap,
        target_waypoint: BoundedSequence::<u64, 1>::new(1), // TODO(@xiyuoh)
        last_waypoint: 0,                                   // TODO(@xiyuoh)
        target_progress: 0.0,                               // TODO(@xiyuoh)
        // SafeZoneId to be populated downstream
        ..default()
    };

    Ok((request.0, safe_zone))
}

fn update_safe_zone(
    Blocking { request, .. }: Blocking<(AgentRequest, SafeZone)>,
    mut commands: Commands,
    mut agents: Query<(&mut Nav2Agent, &CurrentSafeZone)>,
) {
    let Ok((mut agent, current_safe_zone)) = agents.get_mut(request.0.agent) else {
        return;
    };

    // If SafeZone is the same, do not publish
    if let Some(current_safe_zone) = &current_safe_zone.0 {
        if current_safe_zone.incremental_target == request.1.incremental_target
            && current_safe_zone.costmap.data == request.1.costmap.data
        {
            return;
        }
    }

    // Increment SafeZoneId for both agent and to-be-published msg here
    let safe_zone_id = agent.last_safe_zone_id.get_or_insert(SafeZoneId {
        plan_id: PlanId {
            destination_session: new_uuid(),
            plan_version: 0,
        },
        safe_zone_version: 0,
    });
    safe_zone_id.safe_zone_version = safe_zone_id.safe_zone_version + 1;

    let mut safe_zone = request.1.clone();
    safe_zone.id = safe_zone_id.clone();

    // Trigger a SafeZoneReceived event instead of directly publishing it; we
    // allow all publishing to be consolidated in the observer
    commands.trigger(SafeZoneReceived {
        agent: request.0.agent,
        safe_zone,
    });
}

fn on_request_new_plan(
    trigger: Trigger<RequestPlan>,
    mut commands: Commands,
    allocation_services: Res<MapfAllocationServices>,
) {
    let e = trigger.event().0;
    let _ = commands
        .request(e, allocation_services.allocation.clone())
        .detach();
}

fn new_uuid() -> RosUuid {
    let random_uuid = Uuid::new_v4();
    let bytes: [u8; 16] = *random_uuid.as_bytes();
    RosUuid { uuid: bytes }
}
