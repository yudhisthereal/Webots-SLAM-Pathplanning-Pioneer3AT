/// Global variables
let ws = null;
let canvas = document.getElementById('lidarCanvas');
let ctx = canvas.getContext('2d');
let width = canvas.width;
let height = canvas.height;
let centerX = width / 2;
let centerY = height / 2;
let scale = 96; // pixels per meter (default: 2.5m radius)
let maxDisplayRange = 12.0; // Maximum range to display (clamp to this)

let currentRanges = [];
let currentAngles = [];
let maxRange = 12.0;
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

// Set zoom level (pixels per meter)
function setZoom(zoomScale) {
    scale = zoomScale;
    document.getElementById('zoomSlider').value = scale;
    updateZoomDisplay();
    
    // Redraw current visualization
    if (currentRanges.length > 0 && currentAngles.length > 0) {
        updateLidarVisualization(currentRanges, currentAngles);
    }
}

// Initialize zoom slider
function initZoomSlider() {
    let slider = document.getElementById('zoomSlider');
    slider.addEventListener('input', function(e) {
        setZoom(parseFloat(e.target.value));
    });
}

// Initialize canvas
function initCanvas() {
    console.log('[Canvas] Initializing canvas...');
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#0f0f1a';
    ctx.fillRect(0, 0, width, height);
    
    // Draw grid
    ctx.strokeStyle = '#2a2a3a';
    ctx.lineWidth = 1;
    
    let displayRange = getDisplayRange();
    
    // Draw concentric circles based on zoom level
    let step = 0.5;
    let maxCircle = Math.ceil(displayRange);
    
    for (let r = step; r <= maxCircle; r += step) {
        let radius = r * scale;
        if (radius > Math.min(width, height) / 2) break;
        
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
        ctx.stroke();
        
        // Add label
        ctx.fillStyle = '#666';
        ctx.font = '10px Arial';
        ctx.fillText(r.toFixed(1) + 'm', centerX + radius + 3, centerY - 3);
    }
    
    // Draw axes
    ctx.beginPath();
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, height);
    ctx.moveTo(0, centerY);
    ctx.lineTo(width, centerY);
    ctx.stroke();
    
    // Draw robot center
    ctx.fillStyle = '#00ff00';
    ctx.beginPath();
    ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
    ctx.fill();
    
    // Draw direction indicator
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX + 15, centerY);
    ctx.lineTo(centerX + 10, centerY - 5);
    ctx.fillStyle = '#00ff00';
    ctx.fill();
    
    console.log('[Canvas] Initialization complete, display range: ' + displayRange.toFixed(1) + 'm');
}

// Draw LiDAR points (with zoom and clamping)
function drawLidarPoints(ranges, angles) {
    if (!ranges || !angles) {
        console.warn('[Draw] No ranges or angles data');
        return;
    }
    
    let pointsDrawn = 0;
    let displayRange = getDisplayRange();
    
    for (let i = 0; i < ranges.length; i++) {
        let range = ranges[i];
        let angle = angles[i];
        
        // Skip invalid ranges or points beyond display range
        if (range >= maxRange || range <= 0.1) continue;
        if (range > displayRange) continue; // Don't draw outside view
        
        // Calculate point position
        let x = range * Math.cos(angle);
        let y = range * Math.sin(angle);
        
        // Convert to canvas coordinates
        let canvasX = centerX + x * scale;
        let canvasY = centerY - y * scale;
        
        // Check if point is within canvas bounds
        if (canvasX >= 0 && canvasX < width && canvasY >= 0 && canvasY < height) {
            // Color based on distance (red for close, green for far)
            let intensity = Math.min(255, Math.floor((range / maxRange) * 255));
            let r = 255;
            let g = 255 - intensity;
            let b = 255 - intensity;
            
            ctx.fillStyle = `rgb(${r}, ${g}, ${b})`;
            ctx.fillRect(canvasX - 1, canvasY - 1, 2, 2);
            pointsDrawn++;
        }
    }
    
    if (pointsDrawn > 0 && pointsDrawn % 50 === 0) {
        console.log(`[Draw] Drew ${pointsDrawn} points (display range: ${displayRange.toFixed(1)}m)`);
    }
}

// Update LiDAR visualization
function updateLidarVisualization(ranges, angles) {
    initCanvas();
    drawLidarPoints(ranges, angles);
}

// Update scan rate calculation
function updateScanRate(currentTime) {
    if (lastScanTime > 0) {
        let delta = currentTime - lastScanTime;
        if (delta > 0) {
            let instantRate = 1.0 / delta;
            scanRate = scanRate * 0.8 + instantRate * 0.2;
            document.getElementById('scanRate').innerHTML = scanRate.toFixed(1) + ' Hz';
            console.log(`[Scan] Rate: ${scanRate.toFixed(1)} Hz (instant: ${instantRate.toFixed(1)})`);
        }
    }
    lastScanTime = currentTime;
    scanCount++;
}

// Connect to WebSocket (bridge on port 8766)
function connectWebSocket() {
    let url = document.getElementById('wsUrl').value;
    console.log(`[WebSocket] Connecting to ${url}...`);
    
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.close();
    }
    
    ws = new WebSocket(url);
    
    ws.onopen = function() {
        console.log(`[WebSocket] ✅ CONNECTED to ${url}`);
        const statusDiv = document.getElementById('connectionStatus');
        statusDiv.className = 'status connected';
        statusDiv.innerHTML = 'Connected';
        scanCount = 0;
        scanRate = 0;
        reconnectAttempts = 0;
    };
    
    ws.onmessage = function(event) {
        console.log(`[WebSocket] 📨 Raw message received, length: ${event.data.length} chars`);
        console.log(`[WebSocket] First 200 chars: ${event.data.substring(0, 200)}`);
        
        try {
            let data = JSON.parse(event.data);
            console.log(`[WebSocket] Parsed message type: ${data.type}`);
            
            if (data.type === 'lidar_scan') {
                console.log(`[LiDAR] Scan received: ${data.num_points} points, timestamp: ${data.timestamp}`);
                console.log(`[LiDAR] First 5 ranges: ${data.ranges.slice(0, 5)}`);
                console.log(`[LiDAR] First 5 angles: ${data.angles.slice(0, 5)}`);
                
                currentRanges = data.ranges;
                currentAngles = data.angles;
                maxRange = data.max_range;
                
                console.log(`[LiDAR] maxRange = ${maxRange}`);
                console.log(`[LiDAR] currentRanges length = ${currentRanges.length}`);
                console.log(`[LiDAR] currentAngles length = ${currentAngles.length}`);
                
                updateLidarVisualization(currentRanges, currentAngles);
                
                document.getElementById('numPoints').innerHTML = data.num_points.toLocaleString();
                document.getElementById('rangeLimit').innerHTML = `${data.min_range.toFixed(1)}-${data.max_range.toFixed(1)} m`;
                document.getElementById('lastScan').innerHTML = data.timestamp.toFixed(2) + ' s';
                
                updateScanRate(data.timestamp);
                
            } else if (data.type === 'robot_info') {
                console.log(`[Robot] Info: left=${data.left_speed}, right=${data.right_speed}, auto=${data.auto_navigate}`);
                document.getElementById('leftSpeed').innerHTML = data.left_speed.toFixed(2) + ' m/s';
                document.getElementById('rightSpeed').innerHTML = data.right_speed.toFixed(2) + ' m/s';
                document.getElementById('robotMode').innerHTML = data.auto_navigate ? '🤖 Auto' : '🎮 Manual';
            } else {
                console.log(`[WebSocket] Unknown message type: ${data.type}`);
            }
        } catch (e) {
            console.error('[WebSocket] ❌ JSON Parse error:', e);
            console.log('[WebSocket] Raw data that failed parsing:', event.data);
        }
    };
    
    ws.onerror = function(error) {
        console.error('[WebSocket] ❌ Error:', error);
        const statusDiv = document.getElementById('connectionStatus');
        statusDiv.className = 'status error';
        statusDiv.innerHTML = 'Error';
    };
    
    ws.onclose = function(event) {
        console.log(`[WebSocket] ❌ Disconnected. Code: ${event.code}, Reason: ${event.reason}`);
        const statusDiv = document.getElementById('connectionStatus');
        statusDiv.className = 'status disconnected';
        statusDiv.innerHTML = 'Disconnected';
        
        if (reconnectAttempts < maxReconnectAttempts) {
            reconnectAttempts++;
            console.log(`[WebSocket] Reconnect ${reconnectAttempts}/${maxReconnectAttempts} in 3s...`);
            setTimeout(() => connectWebSocket(), 3000);
        }
    };
}

// Disconnect WebSocket
function disconnectWebSocket() {
    console.log('[WebSocket] Manual disconnect requested');
    if (ws) {
        ws.close();
        ws = null;
    }
}

// Send command (via WebSocket bridge)
function sendCommand(command) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        console.warn('[Command] Not connected');
        const statusDiv = document.getElementById('connectionStatus');
        const originalText = statusDiv.innerHTML;
        statusDiv.innerHTML = 'Not Connected!';
        setTimeout(() => {
            statusDiv.innerHTML = originalText;
        }, 1000);
        return;
    }
    
    let cmd = { type: 'command', command: command };
    ws.send(JSON.stringify(cmd));
    console.log('[Command] Sent:', command);
}

// Keyboard controls
function setupKeyboardControls() {
    console.log('[Controls] Setting up keyboard listeners');
    document.addEventListener('keydown', function(event) {
        switch(event.key) {
            case 'ArrowUp': sendCommand('forward'); event.preventDefault(); break;
            case 'ArrowDown': sendCommand('backward'); event.preventDefault(); break;
            case 'ArrowLeft': sendCommand('left'); event.preventDefault(); break;
            case 'ArrowRight': sendCommand('right'); event.preventDefault(); break;
            case ' ': sendCommand('stop'); event.preventDefault(); break;
            case 'a': sendCommand('auto'); event.preventDefault(); break;
            default: console.log(`[Controls] Key pressed: ${event.key}`);
        }
    });
}

// Initialize on page load
window.addEventListener('load', function() {
    console.log('[App] Page loaded, initializing...');
    initCanvas();
    initZoomSlider(); 
    setupKeyboardControls();
    
    // Set default URL to bridge port
    document.getElementById('wsUrl').value = 'ws://localhost:8766';
    
    // Set initial display values
    document.getElementById('numPoints').innerHTML = '0';
    document.getElementById('rangeLimit').innerHTML = '0-0 m';
    document.getElementById('lastScan').innerHTML = '0 s';
    document.getElementById('scanRate').innerHTML = '0 Hz';
    document.getElementById('leftSpeed').innerHTML = '0.00 m/s';
    document.getElementById('rightSpeed').innerHTML = '0.00 m/s';
    document.getElementById('robotMode').innerHTML = 'Auto';
    document.getElementById('battery').innerHTML = '100%';
    document.getElementById('posX').innerHTML = '0.00 m';
    document.getElementById('posY').innerHTML = '0.00 m';
    document.getElementById('theta').innerHTML = '0.00 °';
    
    console.log('[App] Initial display values set');
    
    // Auto-connect after 1 second
    setTimeout(() => {
        console.log('[App] Auto-connecting...');
        connectWebSocket();
    }, 1000);
    
    // Enter key support
    document.getElementById('wsUrl').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            console.log('[App] Enter pressed, connecting...');
            connectWebSocket();
        }
    });
});
