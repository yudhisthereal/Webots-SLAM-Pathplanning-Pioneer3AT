// Global variables
let ws = null;
let canvas = document.getElementById('lidarCanvas');
let ctx = canvas.getContext('2d');
let width = canvas.width;
let height = canvas.height;
let centerX = width / 2;
let centerY = height / 2;
let scale = 96; // pixels per meter (default: 2.5m radius)

// Data storage
let currentRanges = [];
let currentAngles = [];
let maxRange = 12.0;
let robotX = 0, robotY = 0, robotTheta = 0;
let currentMap = null;

// Statistics
let lastScanTime = 0;
let scanCount = 0;
let scanRate = 0;
let reconnectAttempts = 0;
const maxReconnectAttempts = 5;

console.log('[App] Script loaded and ready');

// Calculate display range from scale
function getDisplayRange() {
    let radiusPx = Math.min(width, height) / 2;
    return radiusPx / scale;
}

// Update zoom display
function updateZoomDisplay() {
    let displayRange = getDisplayRange();
    document.getElementById('zoomRange').innerHTML = displayRange.toFixed(1) + ' m';
}

// Set zoom level
function setZoom(zoomScale) {
    scale = zoomScale;
    document.getElementById('zoomSlider').value = scale;
    updateZoomDisplay();
    
    if (currentRanges.length > 0 && currentAngles.length > 0) {
        updateVisualization();
    }
}

// Initialize zoom slider
function initZoomSlider() {
    let slider = document.getElementById('zoomSlider');
    slider.addEventListener('input', function(e) {
        setZoom(parseFloat(e.target.value));
    });
}

// Transform point from Webots world coordinates to screen coordinates (robot-centric)
// Webots: X = forward/back, Y = left/right
// Screen: Y = up (forward), X = right
function worldToScreen(worldX, worldY) {
    // Webots coordinates: (x_webots, y_webots)
    // Map to screen: screen_x = y_webots (left/right becomes x on screen)
    //               screen_y = x_webots (forward/back becomes y on screen, with y increasing DOWN)
    
    // First, get robot-relative coordinates in Webots frame
    let dx_webots = worldX - robotX;      // Forward/back relative to robot
    let dy_webots = worldY - robotY;      // Left/right relative to robot
    
    // Rotate by -robotTheta so robot always faces UP on screen
    // But need to swap X and Y because Webots forward = screen up
    let cosTheta = Math.cos(-robotTheta);
    let sinTheta = Math.sin(-robotTheta);
    
    // Transform: Webots forward (dx) becomes screen UP (negative Y because screen Y increases downward)
    //            Webots right (dy) becomes screen RIGHT (positive X)
    let rotatedX = dy_webots * cosTheta - dx_webots * sinTheta;   // Left/right -> screen X
    let rotatedY = dx_webots * cosTheta + dy_webots * sinTheta;   // Forward/back -> screen Y
    
    // Convert to screen coordinates (Y increases downward, so invert Y)
    let screenX = centerX + rotatedX * scale;
    let screenY = centerY - rotatedY * scale;  // Negative because screen Y goes down
    
    return { x: screenX, y: screenY };
}

// Initialize canvas and draw static elements
function initCanvas() {
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#0f0f1a';
    ctx.fillRect(0, 0, width, height);
    
    // Draw grid circles (robot-centric)
    ctx.strokeStyle = '#2a2a3a';
    ctx.lineWidth = 1;
    
    let displayRange = getDisplayRange();
    let step = 0.5;
    let maxCircle = Math.ceil(displayRange);
    
    for (let r = step; r <= maxCircle; r += step) {
        let radius = r * scale;
        if (radius > Math.min(width, height) / 2) break;
        
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
        ctx.stroke();
        
        ctx.fillStyle = '#666';
        ctx.font = '10px Arial';
        ctx.fillText(r.toFixed(1) + 'm', centerX + radius + 3, centerY - 3);
    }
    
    // Draw axes (robot-centric)
    ctx.beginPath();
    ctx.strokeStyle = '#3a3a4a';
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, height);
    ctx.moveTo(0, centerY);
    ctx.lineTo(width, centerY);
    ctx.stroke();
    
    // Draw robot (always at center, always facing UP)
    drawRobot();
}

// Draw LiDAR points (with correct coordinate transform)
function drawLidarPoints(ranges, angles) {
    if (!ranges || !angles) return;
    
    let pointsDrawn = 0;
    let displayRange = getDisplayRange();
    
    for (let i = 0; i < ranges.length; i++) {
        let range = ranges[i];
        let angle = angles[i];
        
        // Skip invalid ranges
        if (range >= maxRange || range <= 0.1) continue;
        if (range > displayRange) continue;
        
        // LiDAR points are in robot frame from Webots:
        // angle 0 = robot's forward (X axis)
        // So point is at (range * cos(angle), range * sin(angle)) in Webots robot frame
        let localX = range * Math.cos(angle);   // Forward/back
        let localY = range * Math.sin(angle);   // Left/right
        
        // Convert to world coordinates using robot pose
        let cosTheta = Math.cos(robotTheta);
        let sinTheta = Math.sin(robotTheta);
        
        let worldX = robotX + localX * cosTheta - localY * sinTheta;
        let worldY = robotY + localX * sinTheta + localY * cosTheta;
        
        // Convert to screen
        let screen = worldToScreen(worldX, worldY);
        
        if (screen.x >= 0 && screen.x < width && screen.y >= 0 && screen.y < height) {
            // Color based on distance (red=close, yellow=medium, green=far)
            let t = Math.min(1, range / maxRange);
            let r = 255;
            let g = Math.floor(255 * t);
            let b = Math.floor(100 * (1 - t));
            
            ctx.fillStyle = `rgb(${r}, ${g}, ${b})`;
            ctx.fillRect(screen.x - 1.5, screen.y - 1.5, 3, 3);
            pointsDrawn++;
        }
    }
    
    if (pointsDrawn > 0 && scanCount % 30 === 0) {
        console.log(`[Draw] Drew ${pointsDrawn} points, robot theta: ${(robotTheta * 180 / Math.PI).toFixed(1)}°`);
    }
}

// Draw robot (always at center, facing UP on screen)
function drawRobot() {
    // Robot body (circle)
    ctx.fillStyle = '#00ff00';
    ctx.beginPath();
    ctx.arc(centerX, centerY, 8, 0, 2 * Math.PI);
    ctx.fill();
    
    // Direction indicator (always points UP on screen)
    // This represents the robot's forward direction in the visualization
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - 12);
    ctx.lineTo(centerX - 5, centerY);
    ctx.lineTo(centerX + 5, centerY);
    ctx.fillStyle = '#00ff00';
    ctx.fill();
    
    // Robot body rectangle (shows orientation)
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.strokeRect(centerX - 10, centerY - 15, 20, 30);
    
    // Label
    ctx.fillStyle = '#00ff00';
    ctx.font = 'bold 12px Arial';
    ctx.fillText('ROBOT', centerX - 20, centerY - 18);
    
    // Optional: Show robot's actual heading angle as text
    ctx.fillStyle = '#aaaaaa';
    ctx.font = '10px Arial';
    ctx.fillText(`heading: ${(robotTheta * 180 / Math.PI).toFixed(0)}°`, centerX - 30, centerY + 25);
}

// Draw occupancy map (world coordinates transformed to robot-centric)
function drawMap(mapData) {
    if (!mapData || !mapData.data) return;
    
    const mapWidth = mapData.width;
    const mapHeight = mapData.height;
    const resolution = mapData.resolution;
    const mapOriginX = -mapWidth * resolution / 2;
    const mapOriginY = -mapHeight * resolution / 2;
    
    const cellSize = Math.max(1, resolution * scale);
    const viewRadius = getDisplayRange();
    
    let occupiedCount = 0;
    let freeCount = 0;
    
    for (let gy = 0; gy < mapHeight; gy++) {
        for (let gx = 0; gx < mapWidth; gx++) {
            const value = mapData.data[gy * mapWidth + gx];
            
            // Skip unknown cells (value = -1)
            if (value < 0) continue;
            
            // Get world coordinates
            let worldX = gx * resolution + mapOriginX;
            let worldY = gy * resolution + mapOriginY;
            
            // Check if within view
            let dx = worldX - robotX;
            let dy = worldY - robotY;
            if (Math.sqrt(dx*dx + dy*dy) > viewRadius + 0.5) continue;
            
            // Transform to screen
            let screen = worldToScreen(worldX, worldY);
            
            if (screen.x + cellSize > 0 && screen.x < width && 
                screen.y + cellSize > 0 && screen.y < height) {
                
                // OCCUPIED (value >= 30 means high probability of obstacle)
                if (value >= 30) {
                    ctx.fillStyle = '#ff4444';  // Bright red for occupied
                    ctx.fillRect(screen.x, screen.y, cellSize, cellSize);
                    occupiedCount++;
                } 
                // FREE space - don't draw (let background show through)
                // This way only obstacles are visible
            }
        }
    }
    
    if (occupiedCount > 0 && scanCount % 30 === 0) {
        console.log(`[Map] Drew ${occupiedCount} occupied cells`);
    }
}

// Main visualization update
function updateVisualization() {
    initCanvas();
    
    // Draw map first (so LiDAR points appear on top)
//    if (currentMap) {
//        drawMap(currentMap);
//    }
    
    // Draw LiDAR points
    drawLidarPoints(currentRanges, currentAngles);
}

// Update scan rate calculation
function updateScanRate(currentTime) {
    if (lastScanTime > 0) {
        let delta = currentTime - lastScanTime;
        if (delta > 0) {
            let instantRate = 1.0 / delta;
            scanRate = scanRate * 0.8 + instantRate * 0.2;
            document.getElementById('scanRate').innerHTML = scanRate.toFixed(1) + ' Hz';
        }
    }
    lastScanTime = currentTime;
    scanCount++;
}

// Connect to WebSocket
function connectWebSocket() {
    let url = document.getElementById('wsUrl').value;
    console.log(`[WebSocket] Connecting to ${url}...`);
    
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.close();
    }
    
    ws = new WebSocket(url);
    
    ws.onopen = function() {
        console.log(`[WebSocket] ✅ CONNECTED to ${url}`);
        document.getElementById('connectionStatus').className = 'status connected';
        document.getElementById('connectionStatus').innerHTML = 'Connected';
        scanCount = 0;
        scanRate = 0;
        reconnectAttempts = 0;
    };
    
    ws.onmessage = function(event) {
        try {
            let data = JSON.parse(event.data);
            
            if (data.type === 'lidar_scan') {
                // Update data
                currentRanges = data.ranges;
                currentAngles = data.angles;
                maxRange = data.max_range;
                robotX = data.robot_x || 0;
                robotY = data.robot_y || 0;
                robotTheta = data.robot_theta || 0;
                
                // Update display
                updateVisualization();
                
                // Update stats panel
                document.getElementById('numPoints').innerHTML = data.num_points.toLocaleString();
                document.getElementById('rangeLimit').innerHTML = `${data.min_range.toFixed(1)}-${data.max_range.toFixed(1)} m`;
                document.getElementById('lastScan').innerHTML = data.timestamp.toFixed(2) + ' s';
                document.getElementById('posX').innerHTML = robotX.toFixed(2) + ' m';
                document.getElementById('posY').innerHTML = robotY.toFixed(2) + ' m';
                document.getElementById('theta').innerHTML = (robotTheta * 180 / Math.PI).toFixed(1) + ' °';
                
                if (data.linear_vel !== undefined) {
                    document.getElementById('linearVel').innerHTML = data.linear_vel.toFixed(2) + ' m/s';
                }
                
                updateScanRate(data.timestamp);
                
                // Update map if provided
                if (data.map) {
                    currentMap = data.map;
                }
                
            } else if (data.type === 'robot_info') {
                document.getElementById('leftSpeed').innerHTML = data.left_speed.toFixed(2) + ' m/s';
                document.getElementById('rightSpeed').innerHTML = data.right_speed.toFixed(2) + ' m/s';
                document.getElementById('robotMode').innerHTML = data.auto_navigate ? '🤖 Auto' : '🎮 Manual';
            }
        } catch (e) {
            console.error('[WebSocket] Parse error:', e);
        }
    };
    
    ws.onerror = function(error) {
        console.error('[WebSocket] Error:', error);
        document.getElementById('connectionStatus').className = 'status error';
        document.getElementById('connectionStatus').innerHTML = 'Error';
    };
    
    ws.onclose = function(event) {
        console.log(`[WebSocket] Disconnected. Code: ${event.code}`);
        document.getElementById('connectionStatus').className = 'status disconnected';
        document.getElementById('connectionStatus').innerHTML = 'Disconnected';
        
        if (reconnectAttempts < maxReconnectAttempts) {
            reconnectAttempts++;
            console.log(`Reconnect ${reconnectAttempts}/${maxReconnectAttempts} in 3s...`);
            setTimeout(() => connectWebSocket(), 3000);
        }
    };
}

// Disconnect WebSocket
function disconnectWebSocket() {
    console.log('[WebSocket] Manual disconnect');
    if (ws) {
        ws.close();
        ws = null;
    }
}

// Send command to robot
function sendCommand(command) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        console.warn('[Command] Not connected');
        return;
    }
    
    let cmd = { type: 'command', command: command };
    ws.send(JSON.stringify(cmd));
    console.log('[Command] Sent:', command);
}

// Toggle map overlay
function toggleMap() {
    if (currentMap) {
        currentMap = null;
        console.log('[Map] Hidden');
    } else if (window.lastMap) {
        currentMap = window.lastMap;
        console.log('[Map] Shown');
    }
    updateVisualization();
}

// Keyboard controls
function setupKeyboardControls() {
    document.addEventListener('keydown', function(event) {
        switch(event.key) {
            case 'ArrowUp': sendCommand('forward'); event.preventDefault(); break;
            case 'ArrowDown': sendCommand('backward'); event.preventDefault(); break;
            case 'ArrowLeft': sendCommand('left'); event.preventDefault(); break;
            case 'ArrowRight': sendCommand('right'); event.preventDefault(); break;
            case ' ': sendCommand('stop'); event.preventDefault(); break;
            case 'a': sendCommand('auto'); event.preventDefault(); break;
            case 'm': toggleMap(); event.preventDefault(); break;
        }
    });
}

// Initialize on page load
window.addEventListener('load', function() {
    console.log('[App] Initializing...');
    initCanvas();
    initZoomSlider();
    setupKeyboardControls();
    
    // Set default URL
    document.getElementById('wsUrl').value = 'ws://localhost:8766';
    
    // Initialize display values
    document.getElementById('numPoints').innerHTML = '0';
    document.getElementById('rangeLimit').innerHTML = '0-0 m';
    document.getElementById('lastScan').innerHTML = '0 s';
    document.getElementById('scanRate').innerHTML = '0 Hz';
    document.getElementById('leftSpeed').innerHTML = '0.00 m/s';
    document.getElementById('rightSpeed').innerHTML = '0.00 m/s';
    document.getElementById('robotMode').innerHTML = 'Auto';
    document.getElementById('posX').innerHTML = '0.00 m';
    document.getElementById('posY').innerHTML = '0.00 m';
    document.getElementById('theta').innerHTML = '0.00 °';
    
    // Add linear velocity display to HTML if not present
    let velRow = document.querySelector('#linearVel') ? null : 
        document.getElementById('theta').closest('tr').insertAdjacentHTML('afterend', 
        '<tr><td class="info-label">Linear Vel:</td><td class="info-value" id="linearVel">0.00 m/s</td></tr>');
    
    // Auto-connect
    setTimeout(() => {
        console.log('[App] Auto-connecting...');
        connectWebSocket();
    }, 1000);
    
    // Enter key support
    document.getElementById('wsUrl').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') connectWebSocket();
    });
});
