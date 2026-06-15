// RMF Path Server Demo Application Logic - SSE/REST Edition

// State variables
let isConnected = false;
let config = {
  default_radius: 0.49,
  use_destination_server: false
};
let robots = [];
let reservationConfig = null;
let selectedRobotName = null;
let interactionMode = 'normal'; // 'normal', 'add-robot', 'set-goal'
let systemMode = 'setup'; // 'setup', 'live'
let pendingGoalReset = false;
let eventSource = null;
let mouseX = 0;
let mouseY = 0;

// Canvas config
const canvas = document.getElementById('grid-canvas');
const ctx = canvas.getContext('2d');
const scale = 25; // pixels per meter
const centerX = canvas.width / 2;
const centerY = canvas.height / 2;
const REGION_HINT_POINT = 1;
const REGION_HINT_AXIS_ALIGNED_RECTANGLE = 2;

// Color palette for robots
const robotColors = [
  '#00f0ff', // Neon Blue
  '#b026ff', // Neon Purple
  '#ff9900', // Neon Orange
  '#00ff87', // Neon Green
  '#ff005b', // Neon Pink
  '#ffea00', // Neon Yellow
];

// Initialize Server-Sent Events connection
function initSSE() {
  const statusIcon = document.querySelector('#connection-status .status-icon');
  const statusText = document.getElementById('status-text');

  statusText.textContent = 'Connecting...';
  statusIcon.className = 'fa-solid fa-circle-dot status-icon offline';

  eventSource = new EventSource('/stream');

  eventSource.onopen = () => {
    isConnected = true;
    statusText.textContent = 'Connected';
    statusIcon.className = 'fa-solid fa-circle-dot status-icon online';
    console.log('Connected to SSE stream successfully.');
    updateRobotListUI();
  };

  eventSource.onerror = (error) => {
    isConnected = false;
    statusText.textContent = 'Disconnected';
    statusIcon.className = 'fa-solid fa-circle-dot status-icon offline';
    console.error('SSE stream connection failed or lost. Retrying...');
    updateRobotListUI();
  };

  eventSource.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === 'reservation_config') {
        reservationConfig = msg;
        console.log(`Received reservation config: ${msg.safe_sets.length} safe set(s), ${msg.parking_spots.length} parking spot(s).`);
      } else if (msg.type === 'odom') {
        const r = robots.find(robot => robot.name === msg.name);
        if (r) {
          r.current_x = msg.x;
          r.current_y = msg.y;

          // Update status based on destination progress
          if (r.goal_x !== null && r.goal_y !== null) {
            const dist = Math.hypot(r.current_x - r.goal_x, r.current_y - r.goal_y);
            if (dist < 0.2) {
              r.status = 'Reached';
            } else if (systemMode === 'live' && (r.status === 'Pending' || r.status === 'Planning')) {
              r.status = 'Moving';
            }
          }
          updateRobotListUI();
        }
      } else if (msg.type === 'plan') {
        const r = robots.find(robot => robot.name === msg.name);
        if (r) {
          r.waypoints = msg.waypoints;
          console.log(`Received plan for ${msg.name} with ${msg.waypoints.length} waypoints.`);
        }
      } else if (msg.type === 'progress') {
        const r = robots.find(robot => robot.name === msg.name);
        if (r) {
          r.progress = msg.progress;
        }
      } else if (msg.type === 'active_destination') {
        const r = robots.find(robot => robot.name === msg.name);
        if (r) {
          r.active_goal_x = msg.x;
          r.active_goal_y = msg.y;
          r.goal_x = msg.x;
          r.goal_y = msg.y;
          updateRobotListUI();
        }
      }
    } catch (e) {
      // Ignore keepalive or parse errors
    }
  };
}

// Coordinate mapping helpers
function toGrid(px, py) {
  return {
    x: Math.round((px - centerX) / scale),
    y: Math.round((centerY - py) / scale)
  };
}

function toPixel(gx, gy) {
  return {
    x: centerX + gx * scale,
    y: centerY - gy * scale
  };
}

// Helper to show instructions
function showInstruction(text) {
  const banner = document.getElementById('instruction-banner');
  if (text) {
    banner.textContent = text;
    banner.classList.add('visible');
  } else {
    banner.classList.remove('visible');
  }
}

// Spawns a robot Simulator Node by calling HTTP REST /spawn
function sendSpawnRequest(robot) {
  const url = `/spawn?name=${robot.name}&x=${robot.start_x}&y=${robot.start_y}`;
  fetch(url)
    .then(res => res.json())
    .then(data => {
      console.log('Spawn response:', data);
      if (data.radius !== undefined) {
        robot.radius = parseFloat(data.radius);
      }
    })
    .catch(err => console.error('Spawn request failed:', err));
}

// Sends Goal Destination to the backend REST /destination
function sendGoalRequest(robot) {
  let url;
  if (config.use_destination_server) {
    const xs = (robot.goals || []).map(g => g.x).join(',');
    const ys = (robot.goals || []).map(g => g.y).join(',');
    url = `/destination?name=${robot.name}&x=${xs}&y=${ys}`;
  } else {
    url = `/destination?name=${robot.name}&x=${robot.goal_x}&y=${robot.goal_y}`;
  }
  fetch(url)
    .then(res => res.json())
    .then(data => console.log('Goal set response:', data))
    .catch(err => console.error('Goal request failed:', err));
}

// Dynamic GUI operations
function updateRobotListUI() {
  const listDiv = document.getElementById('robot-list');
  const emptyMsg = document.getElementById('empty-list-msg');

  if (robots.length === 0) {
    emptyMsg.style.display = 'flex';
    listDiv.innerHTML = '';
    document.getElementById('btn-send').disabled = true;
    return;
  }

  emptyMsg.style.display = 'none';
  
  // Check if all robots have goals to enable Send Scenario
  const allHaveGoals = config.use_destination_server
    ? robots.every(r => r.goals && r.goals.length > 0)
    : robots.every(r => r.goal_x !== null && r.goal_y !== null);
  document.getElementById('btn-send').disabled = !allHaveGoals || systemMode === 'live' || !isConnected;

  let html = '';
  robots.forEach(r => {
    const isSelected = selectedRobotName === r.name ? 'selected' : '';
    let goalStr = 'Not Set';
    if (config.use_destination_server) {
      if (r.active_goal_x !== undefined && r.active_goal_y !== undefined) {
        goalStr = `Chosen: (${r.active_goal_x}, ${r.active_goal_y})`;
      } else if (r.goals && r.goals.length > 0) {
        goalStr = `Goals: ` + r.goals.map(g => `(${g.x}, ${g.y})`).join(', ');
      }
    } else {
      goalStr = r.goal_x !== null ? `(${r.goal_x}, ${r.goal_y})` : 'Not Set';
    }
    
    let statusClass = '';
    if (r.status === 'Moving') statusClass = 'moving';
    else if (r.status === 'Reached') statusClass = 'reached';

    html += `
      <div class="robot-item ${isSelected}" data-robot-name="${r.name}">
        <div class="robot-item-top">
          <span class="robot-item-name">
            <span class="robot-dot-color" style="background-color: ${r.color}; box-shadow: 0 0 6px ${r.color}"></span>
            ${r.name}
          </span>
          <span class="robot-item-status ${statusClass}">${r.status}</span>
        </div>
        <div class="robot-item-coords">
          <span><span class="coord-label">Pos:</span> (${r.current_x.toFixed(1)}, ${r.current_y.toFixed(1)})</span>
          <span><span class="coord-label">Goal:</span> ${goalStr}</span>
        </div>
      </div>
    `;
  });

  listDiv.innerHTML = html;
}

// Delegate selection because robot rows are redrawn as state changes.
document.getElementById('robot-list').addEventListener('pointerdown', event => {
  const item = event.target.closest('.robot-item');
  if (!item) return;
  event.preventDefault();
  selectRobot(item.dataset.robotName);
});

function selectRobot(name) {
  if (selectedRobotName === name) {
    selectedRobotName = null;
    interactionMode = 'normal';
    showInstruction(null);
    updateRobotListUI();
    return;
  }
  selectedRobotName = name;
  interactionMode = 'set-goal';
  pendingGoalReset = systemMode === 'live' && config.use_destination_server;
  const verb = systemMode === 'live' ? 'NEW Goal' : 'Goal';
  if (config.use_destination_server) {
    showInstruction(`Click the grid to add alternative goals for ${name}. Use the robot list to switch, or select ${name} again in the list to finish.`);
  } else {
    showInstruction(`Click on the grid to place the ${verb} for ${name}.`);
  }
  updateRobotListUI();
}

// Interactive canvas click/hover bindings
canvas.addEventListener('mousemove', (e) => {
  const rect = canvas.getBoundingClientRect();
  // Map display coordinates to the canvas drawing buffer.
  const sx = canvas.width / rect.width;
  const sy = canvas.height / rect.height;
  mouseX = (e.clientX - rect.left) * sx;
  mouseY = (e.clientY - rect.top) * sy;

  const gridPos = toGrid(mouseX, mouseY);
  document.getElementById('coords-display').textContent = `X: ${gridPos.x} | Y: ${gridPos.y}`;
});

canvas.addEventListener('click', (e) => {
  if (!isConnected) {
    alert('Connecting/Not connected to spawner web server! Please wait.');
    return;
  }

  const rect = canvas.getBoundingClientRect();
  const sx = canvas.width / rect.width;
  const sy = canvas.height / rect.height;
  const mx = (e.clientX - rect.left) * sx;
  const my = (e.clientY - rect.top) * sy;
  const gridPos = toGrid(mx, my);

  if (interactionMode === 'add-robot') {
    // Place robot at gridPos
    const idNum = robots.length + 1;
    const name = 'robot_' + idNum;
    const color = robotColors[(idNum - 1) % robotColors.length];

    const newRobot = {
      idNum: idNum,
      name: name,
      color: color,
      start_x: gridPos.x,
      start_y: gridPos.y,
      current_x: gridPos.x,
      current_y: gridPos.y,
      goal_x: null,
      goal_y: null,
      goals: [],
      status: 'Pending',
      waypoints: [],
      progress: 0.0,
      radius: 0.49
    };

    robots.push(newRobot);
    sendSpawnRequest(newRobot);
    
    // Automatically shift to Goal setting mode for this robot
    selectRobot(name);
    
    document.getElementById('btn-add-robot').classList.remove('active');
  } 
  else if (interactionMode === 'set-goal') {
    const robot = robots.find(r => r.name === selectedRobotName);
    if (robot) {
      if (!config.use_destination_server) {
        // Goal is strictly checked to prevent overlaying on active poses or other goals
        const cellOccupied = robots.some(r => 
          (r.name !== robot.name) && 
          (
            (systemMode === 'setup' && r.start_x === gridPos.x && r.start_y === gridPos.y) || 
            (r.goal_x === gridPos.x && r.goal_y === gridPos.y)
          )
        );
        
        if (cellOccupied) {
          alert('This grid coordinate is already occupied by another active goal!');
          return;
        }
      }

      if (config.use_destination_server) {
        if (!robot.goals) {
          robot.goals = [];
        }
        // A live re-selection starts a new set of goal alternatives.
        if (pendingGoalReset) {
          robot.goals = [];
          robot.active_goal_x = undefined;
          robot.active_goal_y = undefined;
          pendingGoalReset = false;
        }
        if (robot.goals.some(g => g.x === gridPos.x && g.y === gridPos.y)) {
          alert('This goal is already added for this robot!');
          return;
        }
        robot.goals.push({x: gridPos.x, y: gridPos.y});
        robot.status = systemMode === 'live' ? 'Planning' : 'Pending';
        
        sendGoalRequest(robot);
        updateRobotListUI();
      } else {
        robot.goal_x = gridPos.x;
        robot.goal_y = gridPos.y;
        robot.status = systemMode === 'live' ? 'Planning' : 'Pending';
        
        sendGoalRequest(robot);

        selectedRobotName = null;
        interactionMode = 'normal';
        showInstruction(null);
        updateRobotListUI();
      }
    }
  } 
  else {
    // Select robot if clicked near one
    const clickedRobot = robots.find(r => {
      const pix = toPixel(r.current_x, r.current_y);
      const dist = Math.hypot(mx - pix.x, my - pix.y);
      return dist < 15; // within 15 pixels radius
    });

    if (clickedRobot) {
      selectRobot(clickedRobot.name);
    }
  }
});

// Setup Button Action Listeners
document.getElementById('btn-add-robot').addEventListener('click', () => {
  if (systemMode === 'live') return;
  
  interactionMode = 'add-robot';
  selectedRobotName = null;
  document.getElementById('btn-add-robot').classList.add('active');
  showInstruction('Click on the grid to place the robot\'s starting position.');
  updateRobotListUI();
});

document.getElementById('btn-send').addEventListener('click', () => {
  if (systemMode === 'live' || robots.length === 0) return;

  systemMode = 'live';
  interactionMode = 'normal';
  showInstruction(null);
  
  document.getElementById('mode-badge').textContent = 'Live Mode';
  document.getElementById('mode-badge').classList.add('live');
  document.getElementById('btn-add-robot').disabled = true;
  document.getElementById('btn-send').disabled = true;

  // Trigger ROS 2 scenario publishing over backend HTTP API
  fetch('/send_scenario')
    .then(res => res.json())
    .then(data => {
      console.log('Scenario start response:', data);
      robots.forEach(r => r.status = 'Planning');
      updateRobotListUI();
    })
    .catch(err => console.error('Failed to start scenario:', err));
});

document.getElementById('btn-reset').addEventListener('click', () => {
  // 1. Trigger server reset over backend REST API
  fetch('/reset')
    .then(res => res.json())
    .then(data => console.log('Reset response:', data))
    .catch(err => console.error('Reset request failed:', err));

  // 2. Close EventSource and reopen to clean SSE state
  if (eventSource) {
    eventSource.close();
  }

  // 3. Clear UI States
  robots = [];
  selectedRobotName = null;
  interactionMode = 'normal';
  systemMode = 'setup';

  document.getElementById('mode-badge').textContent = 'Setup Mode';
  document.getElementById('mode-badge').classList.remove('live');
  document.getElementById('btn-add-robot').disabled = false;
  document.getElementById('btn-add-robot').classList.remove('active');
  showInstruction(null);
  
  // Re-init SSE stream connection
  initSSE();
  fetchConfig();
  fetchReservationConfig();
});

// Helper to draw dependency arrow with optional dash animation
function drawDependencyArrow(ctx, fromX, fromY, toX, toY, isBlocking) {
  const headlen = 8; // length of head in pixels
  const dx = toX - fromX;
  const dy = toY - fromY;
  const angle = Math.atan2(dy, dx);
  
  ctx.save();
  ctx.shadowBlur = 0; // Disable shadow glow for dependency lines to keep canvas clean
  
  if (isBlocking) {
    // Neon Orange pulsing animated flow
    ctx.strokeStyle = '#ff5500';
    ctx.lineWidth = 2.0;
    ctx.setLineDash([6, 4]);
    ctx.lineDashOffset = -Date.now() / 50 % 100;
  } else {
    // Faded translucent green static dash
    ctx.strokeStyle = 'rgba(0, 255, 135, 0.25)';
    ctx.lineWidth = 1.0;
    ctx.setLineDash([4, 4]);
  }
  
  ctx.beginPath();
  ctx.moveTo(fromX, fromY);
  ctx.lineTo(toX, toY);
  ctx.stroke();
  
  // Arrow head
  ctx.fillStyle = isBlocking ? '#ff5500' : 'rgba(0, 255, 135, 0.4)';
  ctx.beginPath();
  ctx.moveTo(toX, toY);
  ctx.lineTo(toX - headlen * Math.cos(angle - Math.PI / 12), toY - headlen * Math.sin(angle - Math.PI / 12));
  ctx.lineTo(toX - headlen * Math.cos(angle + Math.PI / 12), toY - headlen * Math.sin(angle + Math.PI / 12));
  ctx.fill();
  
  ctx.restore();
}

function regionBounds(region) {
  const points = region.points || [];
  if (points.length < 2) return null;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (let i = 0; i + 1 < points.length; i += 2) {
    minX = Math.min(minX, points[i]);
    maxX = Math.max(maxX, points[i]);
    minY = Math.min(minY, points[i + 1]);
    maxY = Math.max(maxY, points[i + 1]);
  }
  return { minX, minY, maxX, maxY };
}

function addRegionPath(region) {
  const points = region.points || [];
  if (points.length < 2) return false;

  if (region.hint === REGION_HINT_POINT) {
    const position = toPixel(points[0], points[1]);
    ctx.arc(position.x, position.y, 3, 0, 2 * Math.PI);
    return true;
  }

  if (region.hint === REGION_HINT_AXIS_ALIGNED_RECTANGLE) {
    const bounds = regionBounds(region);
    if (!bounds) return false;
    const topLeft = toPixel(bounds.minX, bounds.maxY);
    ctx.rect(
      topLeft.x,
      topLeft.y,
      (bounds.maxX - bounds.minX) * scale,
      (bounds.maxY - bounds.minY) * scale
    );
    return true;
  }

  if (points.length < 6) return false;
  const first = toPixel(points[0], points[1]);
  ctx.moveTo(first.x, first.y);
  for (let i = 2; i + 1 < points.length; i += 2) {
    const position = toPixel(points[i], points[i + 1]);
    ctx.lineTo(position.x, position.y);
  }
  ctx.closePath();
  return true;
}

function drawReservationConfig() {
  if (!reservationConfig) return;

  (reservationConfig.safe_sets || []).forEach(set => {
    const bounds = regionBounds(set.region);
    if (!bounds) return;
    const labelPosition = toPixel(bounds.minX, bounds.maxY);

    ctx.save();
    ctx.shadowBlur = 0;
    ctx.beginPath();
    if (!addRegionPath(set.region)) {
      ctx.restore();
      return;
    }
    ctx.fillStyle = 'rgba(0, 255, 135, 0.06)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 255, 135, 0.5)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]);
    ctx.stroke();

    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(0, 255, 135, 0.8)';
    ctx.font = '600 11px Outfit';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(
      `safe: ${set.name}`,
      labelPosition.x + 4,
      labelPosition.y + 4
    );
    ctx.restore();
  });

  (reservationConfig.parking_spots || []).forEach(spot => {
    const bounds = regionBounds(spot.region);
    if (!bounds) return;
    const position = toPixel(
      (bounds.minX + bounds.maxX) / 2,
      (bounds.minY + bounds.maxY) / 2
    );

    ctx.save();
    ctx.shadowBlur = 0;

    ctx.beginPath();
    if (addRegionPath(spot.region)) {
      ctx.fillStyle = 'rgba(255, 153, 0, 0.10)';
      ctx.fill();
      ctx.strokeStyle = 'rgba(255, 153, 0, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    ctx.fillStyle = '#ff9900';
    ctx.font = 'bold 12px Outfit';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('P', position.x, position.y);

    ctx.fillStyle = 'rgba(255, 153, 0, 0.85)';
    ctx.font = '600 10px Outfit';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillText(spot.name, position.x + 8, position.y - 4);
    ctx.restore();
  });
}

// Rendering grid background and actors
function drawGrid() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // 1. Draw fine grid
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
  ctx.lineWidth = 1;
  for (let x = 0; x < canvas.width; x += scale) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }
  for (let y = 0; y < canvas.height; y += scale) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  // 2. Draw axis lines
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
  ctx.lineWidth = 1.5;
  
  // Horizontal X axis
  ctx.beginPath();
  ctx.moveTo(0, centerY);
  ctx.lineTo(canvas.width, centerY);
  ctx.stroke();

  // Vertical Y axis
  ctx.beginPath();
  ctx.moveTo(centerX, 0);
  ctx.lineTo(centerX, canvas.height);
  ctx.stroke();

  // 3. Axis ticks/labels
  ctx.fillStyle = '#475569';
  ctx.font = '10px JetBrains Mono';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';

  for (let x = -10; x <= 10; x += 2) {
    if (x === 0) continue;
    const pix = toPixel(x, 0);
    ctx.fillText(x.toString(), pix.x, centerY + 6);
  }

  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let y = -10; y <= 10; y += 2) {
    if (y === 0) continue;
    const pix = toPixel(0, y);
    ctx.fillText(y.toString(), centerX - 8, pix.y);
  }

  // Draw the reservation config behind plans and robots.
  drawReservationConfig();

  // 4. Draw plans, actual routes, and dependencies
  robots.forEach(r => {
    if (r.waypoints && r.waypoints.length > 0) {
      // 4a. Draw actual scheduled path computed by mapf
      ctx.save();
      ctx.strokeStyle = r.color;
      ctx.lineWidth = 2.0;
      ctx.shadowColor = r.color;
      ctx.shadowBlur = 4;
      
      ctx.beginPath();
      const startPix = toPixel(r.waypoints[0].x, r.waypoints[0].y);
      ctx.moveTo(startPix.x, startPix.y);
      for (let i = 1; i < r.waypoints.length; i++) {
        const wpPix = toPixel(r.waypoints[i].x, r.waypoints[i].y);
        ctx.lineTo(wpPix.x, wpPix.y);
      }
      ctx.stroke();
      ctx.restore();

      // 4b. Draw waypoint marker dots
      r.waypoints.forEach((wp, idx) => {
        const wpPix = toPixel(wp.x, wp.y);
        ctx.save();
        ctx.fillStyle = r.color;
        ctx.beginPath();
        ctx.arc(wpPix.x, wpPix.y, 4, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();

        // 4c. Draw departure dependencies for this waypoint
        if (wp.departure_blockers) {
          wp.departure_blockers.forEach(dep => {
            const dependedRobot = robots.find(bot => bot.name === dep.name);
            if (dependedRobot) {
              // Check if the dependency is active or resolved
              const isBlocking = dependedRobot.progress === undefined || dependedRobot.progress < dep.required_progress;

              // Find physical coordinate of the dependency trigger waypoint on the depended robot's path
              let depX = dependedRobot.current_x;
              let depY = dependedRobot.current_y;
              if (dependedRobot.waypoints) {
                const matchWp = dependedRobot.waypoints.find(w => Math.abs(w.progress - dep.required_progress) < 1e-2);
                if (matchWp) {
                  depX = matchWp.x;
                  depY = matchWp.y;
                }
              }

              const depPix = toPixel(depX, depY);
              drawDependencyArrow(ctx, wpPix.x, wpPix.y, depPix.x, depPix.y, isBlocking);
            }
          });
        }
      });
    } else if (config.use_destination_server) {
      // Fallback straight dotted lines to candidate or active goals
      const curPix = toPixel(r.current_x, r.current_y);
      const goals = (r.active_goal_x !== undefined && r.active_goal_y !== undefined)
        ? [{x: r.active_goal_x, y: r.active_goal_y}]
        : (r.goals || []);

      goals.forEach(g => {
        const goalPix = toPixel(g.x, g.y);
        ctx.save();
        ctx.strokeStyle = r.color;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(curPix.x, curPix.y);
        ctx.lineTo(goalPix.x, goalPix.y);
        ctx.stroke();
        ctx.restore();
      });
    } else if (r.goal_x !== null) {
      // Fallback to straight dotted goal route in setup mode before plan is generated
      const curPix = toPixel(r.current_x, r.current_y);
      const goalPix = toPixel(r.goal_x, r.goal_y);

      ctx.save();
      ctx.strokeStyle = r.color;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(curPix.x, curPix.y);
      ctx.lineTo(goalPix.x, goalPix.y);
      ctx.stroke();
      ctx.restore();
    }
  });

  // 4.5. Draw active dynamic target guide line if in set-goal mode
  if (interactionMode === 'set-goal' && selectedRobotName) {
    const r = robots.find(robot => robot.name === selectedRobotName);
    if (r) {
      const curPix = toPixel(r.current_x, r.current_y);
      ctx.strokeStyle = r.color;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(curPix.x, curPix.y);
      ctx.lineTo(mouseX, mouseY);
      ctx.stroke();
      ctx.setLineDash([]); // Reset
    }
  }

  // 5. Draw goals
  robots.forEach(r => {
    if (config.use_destination_server) {
      const goals = r.goals || [];
      goals.forEach((g, idx) => {
        const isChosen = r.active_goal_x !== undefined && r.active_goal_y !== undefined &&
                         r.active_goal_x === g.x && r.active_goal_y === g.y;
        const hasChosenAny = r.active_goal_x !== undefined && r.active_goal_y !== undefined;
        
        const pix = toPixel(g.x, g.y);
        
        const opacity = (!hasChosenAny || isChosen) ? 'ff' : '44';
        const glowOpacity = (!hasChosenAny || isChosen) ? '33' : '0f';
        const colorWithOpacity = r.color + opacity;
        
        ctx.save();
        ctx.strokeStyle = colorWithOpacity;
        ctx.lineWidth = isChosen ? 4.0 : 2.5;
        
        // Draw goal cross marker
        ctx.beginPath();
        ctx.moveTo(pix.x - 8, pix.y - 8);
        ctx.lineTo(pix.x + 8, pix.y + 8);
        ctx.moveTo(pix.x + 8, pix.y - 8);
        ctx.lineTo(pix.x - 8, pix.y + 8);
        ctx.stroke();

        // Draw glowing halo
        ctx.strokeStyle = r.color + glowOpacity;
        ctx.lineWidth = isChosen ? 8 : 5;
        ctx.beginPath();
        ctx.arc(pix.x, pix.y, 12, 0, 2 * Math.PI);
        ctx.stroke();

        // Text label
        ctx.fillStyle = colorWithOpacity;
        ctx.font = '600 10px Outfit';
        ctx.textAlign = 'left';
        ctx.fillText(`G_${r.idNum}.${idx + 1}`, pix.x + 12, pix.y);
        ctx.restore();
      });
    } else {
      if (r.goal_x !== null) {
        const pix = toPixel(r.goal_x, r.goal_y);
        
        ctx.strokeStyle = r.color;
        ctx.lineWidth = 2.5;
        
        // Draw goal cross marker
        ctx.beginPath();
        ctx.moveTo(pix.x - 8, pix.y - 8);
        ctx.lineTo(pix.x + 8, pix.y + 8);
        ctx.moveTo(pix.x + 8, pix.y - 8);
        ctx.lineTo(pix.x - 8, pix.y + 8);
        ctx.stroke();

        // Draw glowing halo
        ctx.strokeStyle = r.color + '33';
        ctx.lineWidth = 5;
        ctx.beginPath();
        ctx.arc(pix.x, pix.y, 12, 0, 2 * Math.PI);
        ctx.stroke();

        // Text label
        ctx.fillStyle = r.color;
        ctx.font = '600 10px Outfit';
        ctx.textAlign = 'left';
        ctx.fillText(`G_${r.idNum}`, pix.x + 12, pix.y);
      }
    }
  });

  // 6. Draw robots
  robots.forEach(r => {
    const pix = toPixel(r.current_x, r.current_y);

    // Set glowing neon effect
    ctx.shadowColor = r.color;
    ctx.shadowBlur = 10;

    // Draw pulsing active selection ring
    if (r.name === selectedRobotName) {
      ctx.strokeStyle = r.color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      const ringRadius = 13 + 2 * Math.sin(Date.now() / 150);
      ctx.arc(pix.x, pix.y, ringRadius, 0, 2 * Math.PI);
      ctx.stroke();
    }

    // Draw planning footprint circle (radius 0.49m)
    ctx.save();
    ctx.shadowBlur = 0; // Reset shadow for the footprint
    ctx.fillStyle = r.color + '33'; // Semi-transparent fill
    ctx.strokeStyle = r.color + 'bb'; // Bright outline
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    const footprintRadius = (r.radius || 0.49) * scale;
    ctx.arc(pix.x, pix.y, footprintRadius, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // Draw rounded square
    ctx.fillStyle = r.color;
    const size = 16;
    ctx.beginPath();
    ctx.roundRect(pix.x - size / 2, pix.y - size / 2, size, size, 4);
    ctx.fill();

    // Draw thin white inner dot
    ctx.shadowBlur = 0; // Reset shadow
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.arc(pix.x, pix.y, 2.5, 0, 2 * Math.PI);
    ctx.fill();

    // Text label
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 11px Outfit';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`R_${r.idNum}`, pix.x, pix.y - 12);
  });
}

let lastCanvasDisplaySize = -1;
function resizeCanvasDisplay() {
  const container = canvas.parentElement;
  const style = getComputedStyle(container);
  const padX = parseFloat(style.paddingLeft) + parseFloat(style.paddingRight);
  const padY = parseFloat(style.paddingTop) + parseFloat(style.paddingBottom);
  const availW = container.clientWidth - padX;
  const availH = container.clientHeight - padY;
  const size = Math.max(0, Math.floor(Math.min(availW, availH)));
  if (size !== lastCanvasDisplaySize) {
    lastCanvasDisplaySize = size;
    canvas.style.width = size + 'px';
    canvas.style.height = size + 'px';
  }
}

window.addEventListener('resize', resizeCanvasDisplay);

// Periodic loop for canvas redraw
function animationLoop() {
  resizeCanvasDisplay();
  drawGrid();
  requestAnimationFrame(animationLoop);
}

function fetchConfig() {
  fetch('/config')
    .then(res => res.json())
    .then(data => {
      config = { ...config, ...data };
      console.log('Loaded config:', config);
      updateRobotListUI();
    })
    .catch(err => console.error('Failed to load config:', err));
}

function fetchReservationConfig() {
  fetch('/reservation_config')
    .then(res => res.json())
    .then(data => {
      if (data && data.safe_sets) {
        reservationConfig = data;
        console.log('Loaded reservation config:', reservationConfig);
      }
    })
    .catch(err => console.error('Failed to load reservation config:', err));
}

// Boot application
resizeCanvasDisplay();
fetchConfig();
fetchReservationConfig();
initSSE();
animationLoop();
