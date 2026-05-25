// RMF Path Server Demo Application Logic - SSE/REST Edition

// State variables
let isConnected = false;
let robots = [];
let selectedRobotName = null;
let interactionMode = 'normal'; // 'normal', 'add-robot', 'set-goal'
let systemMode = 'setup'; // 'setup', 'live'
let eventSource = null;
let mouseX = 0;
let mouseY = 0;

// Canvas config
const canvas = document.getElementById('grid-canvas');
const ctx = canvas.getContext('2d');
const scale = 25; // pixels per meter
const centerX = canvas.width / 2;
const centerY = canvas.height / 2;

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
      if (msg.type === 'odom') {
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
    .then(data => console.log('Spawn response:', data))
    .catch(err => console.error('Spawn request failed:', err));
}

// Sends Goal Destination to the backend REST /destination
function sendGoalRequest(robot) {
  const url = `/destination?name=${robot.name}&x=${robot.goal_x}&y=${robot.goal_y}`;
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
  const allHaveGoals = robots.every(r => r.goal_x !== null && r.goal_y !== null);
  document.getElementById('btn-send').disabled = !allHaveGoals || systemMode === 'live' || !isConnected;

  let html = '';
  robots.forEach(r => {
    const isSelected = selectedRobotName === r.name ? 'selected' : '';
    const goalStr = r.goal_x !== null ? `(${r.goal_x}, ${r.goal_y})` : 'Not Set';
    
    let statusClass = '';
    if (r.status === 'Moving') statusClass = 'moving';
    else if (r.status === 'Reached') statusClass = 'reached';

    html += `
      <div class="robot-item ${isSelected}" onclick="selectRobot('${r.name}')">
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

function selectRobot(name) {
  selectedRobotName = name;
  interactionMode = 'set-goal';
  const verb = systemMode === 'live' ? 'NEW Goal' : 'Goal';
  showInstruction(`Click on the grid to place the ${verb} for ${name}.`);
  updateRobotListUI();
}

// Interactive canvas click/hover bindings
canvas.addEventListener('mousemove', (e) => {
  const rect = canvas.getBoundingClientRect();
  mouseX = e.clientX - rect.left;
  mouseY = e.clientY - rect.top;
  
  const gridPos = toGrid(mouseX, mouseY);
  document.getElementById('coords-display').textContent = `X: ${gridPos.x} | Y: ${gridPos.y}`;
});

canvas.addEventListener('click', (e) => {
  if (!isConnected) {
    alert('Connecting/Not connected to spawner web server! Please wait.');
    return;
  }

  const rect = canvas.getBoundingClientRect();
  const mx = e.clientX - rect.left;
  const my = e.clientY - rect.top;
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
      status: 'Pending'
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
});

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

  // 4. Draw goal routes (dotted lines)
  robots.forEach(r => {
    if (r.goal_x !== null) {
      const curPix = toPixel(r.current_x, r.current_y);
      const goalPix = toPixel(r.goal_x, r.goal_y);

      ctx.strokeStyle = r.color;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(curPix.x, curPix.y);
      ctx.lineTo(goalPix.x, goalPix.y);
      ctx.stroke();
      ctx.setLineDash([]); // Reset
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

    // Draw rounded square
    ctx.fillStyle = r.color;
    const size = 16;
    ctx.beginPath();
    ctx.roundRect(pix.x - size/2, pix.y - size/2, size, size, 4);
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

// Periodic loop for canvas redraw
function animationLoop() {
  drawGrid();
  requestAnimationFrame(animationLoop);
}

// Boot application
initSSE();
animationLoop();
