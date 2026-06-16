// WebSocket connection
let ws = null;

// View state
let activeView = 'radar';

// Canvas references
let radarCanvas = null;
let radarCtx = null;
let mapCanvas = null;
let mapCtx = null;

// Radar view state
let radarWidth = 0;
let radarHeight = 0;
let radarCenterX = 0;
let radarCenterY = 0;
let radarScale = 96;

// Map view state
const mapView = {
    zoom: 45,
    centerX: 0,
    centerY: 0,
    dragging: false,
    dragStartX: 0,
    dragStartY: 0,
    dragOriginX: 0,
    dragOriginY: 0,
};

// Data storage
let currentRanges = [];
let currentAngles = [];
let maxRange = 12.0;
let robotX = 0;
let robotY = 0;
let robotTheta = 0;
let rawRobotX = 0;
let rawRobotY = 0;
let rawRobotTheta = 0;
let currentMap = null;
let slamMatchScore = 0;
let currentPath = [];
let manualDriving = false; // when true, keyboard controls are forwarded to robot
let showGridMap = true; // toggle for grid map visibility

// Statistics
let lastScanTime = 0;
let scanCount = 0;
let scanRate = 0;
let reconnectAttempts = 0;
const maxReconnectAttempts = 5;

// Axis range for map display (dynamic)
let mapAxisRange = 10; // meters

console.log('[App] Script loaded and ready');

function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
}

function initCanvases() {
    radarCanvas = document.getElementById('radarCanvas');
    radarCtx = radarCanvas.getContext('2d');
    mapCanvas = document.getElementById('mapCanvas');
    mapCtx = mapCanvas.getContext('2d');

    radarWidth = radarCanvas.width;
    radarHeight = radarCanvas.height;
    radarCenterX = radarWidth / 2;
    radarCenterY = radarHeight / 2;
}

function getDisplayRange() {
    const radiusPx = Math.min(radarWidth, radarHeight) / 2;
    return radiusPx / radarScale;
}

function updateZoomDisplay() {
    const displayRange = getDisplayRange();
    const zoomLabel = document.getElementById('zoomRange');
    if (zoomLabel) {
        zoomLabel.innerHTML = displayRange.toFixed(1) + ' m';
    }
}

function setRadarZoom(zoomScale) {
    radarScale = zoomScale;
    document.getElementById('zoomSlider').value = radarScale;
    updateZoomDisplay();

    if (activeView === 'radar') {
        renderRadarView();
    }
}

function initZoomSlider() {
    const slider = document.getElementById('zoomSlider');
    slider.addEventListener('input', function(event) {
        setRadarZoom(parseFloat(event.target.value));
    });

    const mapSlider = document.getElementById('mapZoomSlider');
    if (mapSlider) {
        mapSlider.addEventListener('input', function(event) {
            const zoomValue = parseFloat(event.target.value);
            mapView.zoom = zoomValue;
            document.getElementById('mapZoomValue').textContent = zoomValue.toFixed(0);
            if (activeView === 'map') {
                renderMapView();
            }
        });
    }
}

function showView(viewName) {
    activeView = viewName;

    document.getElementById('radarTab').classList.toggle('active', viewName === 'radar');
    document.getElementById('mapTab').classList.toggle('active', viewName === 'map');
    document.getElementById('radarPanel').classList.toggle('active', viewName === 'radar');
    document.getElementById('mapPanel').classList.toggle('active', viewName === 'map');

    if (viewName === 'map') {
        renderMapView();
    } else {
        renderRadarView();
    }
}

function worldToRadarScreen(range, angle) {
    // For radar view, the robot is always at center facing up
    // Range is in meters, convert to pixels
    const forward = range * Math.cos(angle);
    const left = range * Math.sin(angle);
    
    // Radar view: Y is up (forward), X is right
    return {
        x: radarCenterX + left * radarScale,
        y: radarCenterY - forward * radarScale,  // Negative because screen Y goes down
    };
}

function renderRadarBackground() {
    radarCtx.clearRect(0, 0, radarWidth, radarHeight);
    radarCtx.fillStyle = '#0f0f1a';
    radarCtx.fillRect(0, 0, radarWidth, radarHeight);

    radarCtx.strokeStyle = '#2a2a3a';
    radarCtx.lineWidth = 1;

    const displayRange = getDisplayRange();
    const step = 0.5;
    const maxCircle = Math.ceil(displayRange);

    for (let radiusMeters = step; radiusMeters <= maxCircle; radiusMeters += step) {
        const radiusPx = radiusMeters * radarScale;
        if (radiusPx > Math.min(radarWidth, radarHeight) / 2) {
            break;
        }

        radarCtx.beginPath();
        radarCtx.arc(radarCenterX, radarCenterY, radiusPx, 0, 2 * Math.PI);
        radarCtx.stroke();

        radarCtx.fillStyle = '#666';
        radarCtx.font = '10px Arial';
        radarCtx.fillText(radiusMeters.toFixed(1) + 'm', radarCenterX + radiusPx + 3, radarCenterY - 3);
    }

    radarCtx.beginPath();
    radarCtx.strokeStyle = '#3a3a4a';
    radarCtx.moveTo(radarCenterX, 0);
    radarCtx.lineTo(radarCenterX, radarHeight);
    radarCtx.moveTo(0, radarCenterY);
    radarCtx.lineTo(radarWidth, radarCenterY);
    radarCtx.stroke();
}

function drawRadarRobot() {
    radarCtx.fillStyle = '#00ff00';
    radarCtx.beginPath();
    radarCtx.arc(radarCenterX, radarCenterY, 8, 0, 2 * Math.PI);
    radarCtx.fill();

    radarCtx.beginPath();
    radarCtx.moveTo(radarCenterX, radarCenterY - 14);
    radarCtx.lineTo(radarCenterX - 6, radarCenterY + 3);
    radarCtx.lineTo(radarCenterX + 6, radarCenterY + 3);
    radarCtx.closePath();
    radarCtx.fill();

    radarCtx.strokeStyle = '#00ff00';
    radarCtx.lineWidth = 2;
    radarCtx.strokeRect(radarCenterX - 10, radarCenterY - 15, 20, 30);

    radarCtx.fillStyle = '#00ff00';
    radarCtx.font = 'bold 12px Arial';
    radarCtx.fillText('ROBOT', radarCenterX - 20, radarCenterY - 18);

    radarCtx.fillStyle = '#aaaaaa';
    radarCtx.font = '10px Arial';
    radarCtx.fillText(`heading: ${(robotTheta * 180 / Math.PI).toFixed(0)}°`, radarCenterX - 38, radarCenterY + 26);
}

function drawRadarPoints(ranges, angles) {
    if (!ranges || !angles) {
        return;
    }

    const displayRange = getDisplayRange();
    let pointsDrawn = 0;

    for (let i = 0; i < ranges.length; i++) {
        const range = ranges[i];
        const angle = angles[i];

        if (range >= maxRange || range <= 0.1 || range > displayRange) {
            continue;
        }

        const screen = worldToRadarScreen(range, angle);

        if (screen.x >= 0 && screen.x < radarWidth && screen.y >= 0 && screen.y < radarHeight) {
            const t = Math.min(1, range / maxRange);
            const red = 255;
            const green = Math.floor(255 * t);
            const blue = Math.floor(100 * (1 - t));

            radarCtx.fillStyle = `rgb(${red}, ${green}, ${blue})`;
            radarCtx.fillRect(screen.x - 1.5, screen.y - 1.5, 3, 3);
            pointsDrawn++;
        }
    }

    if (pointsDrawn > 0 && scanCount % 30 === 0) {
        console.log(`[Radar] Drew ${pointsDrawn} points`);
    }
}

function renderRadarView() {
    renderRadarBackground();
    drawRadarPoints(currentRanges, currentAngles);
    drawRadarRobot();
}

function worldToMapScreen(worldX, worldY) {
    return {
        x: mapCanvas.width / 2 + (worldX - mapView.centerX) * mapView.zoom,
        y: mapCanvas.height / 2 + (worldY - mapView.centerY) * mapView.zoom,
    };
}

function mapScreenToWorld(screenX, screenY) {
    return {
        x: mapView.centerX + (screenX - mapCanvas.width / 2) / mapView.zoom,
        y: mapView.centerY + (screenY - mapCanvas.height / 2) / mapView.zoom,
    };
}

function resetMapView() {
    mapView.zoom = 45;
    mapView.centerX = 0;
    mapView.centerY = 0;
    if (activeView === 'map') {
        renderMapView();
    }
}

function centerMapOnRobot() {
    mapView.centerX = robotX;
    mapView.centerY = robotY;
    if (activeView === 'map') {
        renderMapView();
    }
}

function zoomMap(factor) {
    mapView.zoom = clamp(mapView.zoom * factor, 10, 180);
    if (activeView === 'map') {
        renderMapView();
    }
}

function toggleGridMap() {
    showGridMap = !showGridMap;
    const btn = document.getElementById('toggleGridBtn');
    btn.textContent = showGridMap ? 'Hide Grid' : 'Show Grid';
    if (activeView === 'map') {
        renderMapView();
    }
}

function drawMapAxes() {
    const canvasWidth = mapCanvas.width;
    const canvasHeight = mapCanvas.height;
    
    // Determine the visible world range
    const visibleWorldWidth = canvasWidth / mapView.zoom;
    const visibleWorldHeight = canvasHeight / mapView.zoom;
    const leftWorld = mapView.centerX - visibleWorldWidth / 2;
    const rightWorld = mapView.centerX + visibleWorldWidth / 2;
    const bottomWorld = mapView.centerY - visibleWorldHeight / 2;
    const topWorld = mapView.centerY + visibleWorldHeight / 2;
    
    // Find a nice step size for axis labels
    let range = Math.max(visibleWorldWidth, visibleWorldHeight);
    let step = Math.pow(10, Math.floor(Math.log10(range / 5)));
    if (range / step < 4) step /= 2;
    if (range / step < 2) step /= 2;
    
    // Draw the origin axes (bold lines at x=0, y=0 if visible)
    mapCtx.strokeStyle = 'rgba(100, 100, 150, 0.3)';
    mapCtx.lineWidth = 1;
    
    // Check if origin is visible
    if (leftWorld <= 0 && rightWorld >= 0) {
        const originScreen = worldToMapScreen(0, 0);
        mapCtx.beginPath();
        mapCtx.moveTo(originScreen.x, 0);
        mapCtx.lineTo(originScreen.x, canvasHeight);
        mapCtx.stroke();
        
        // Label the Y-axis (at x=0)
        mapCtx.fillStyle = '#555';
        mapCtx.font = '10px Arial';
        mapCtx.fillText('Y', originScreen.x + 4, 14);
    }
    
    if (bottomWorld <= 0 && topWorld >= 0) {
        const originScreen = worldToMapScreen(0, 0);
        mapCtx.beginPath();
        mapCtx.moveTo(0, originScreen.y);
        mapCtx.lineTo(canvasWidth, originScreen.y);
        mapCtx.stroke();
        
        // Label the X-axis (at y=0)
        mapCtx.fillStyle = '#555';
        mapCtx.font = '10px Arial';
        mapCtx.fillText('X', canvasWidth - 20, originScreen.y - 4);
    }
    
    // Draw X-axis ticks and labels (along the bottom edge of the visible area)
    const yPosForXLabels = bottomWorld;
    const xStart = Math.ceil(leftWorld / step) * step;
    const xEnd = Math.floor(rightWorld / step) * step;
    
    mapCtx.fillStyle = '#666';
    mapCtx.font = '9px Arial';
    mapCtx.textAlign = 'center';
    mapCtx.textBaseline = 'top';
    
    for (let x = xStart; x <= xEnd; x += step) {
        if (Math.abs(x) < 0.001) continue; // Skip origin (already labeled)
        const screen = worldToMapScreen(x, yPosForXLabels);
        if (screen.x >= 0 && screen.x <= canvasWidth) {
            // Tick mark
            mapCtx.strokeStyle = 'rgba(80, 80, 120, 0.3)';
            mapCtx.lineWidth = 1;
            mapCtx.beginPath();
            mapCtx.moveTo(screen.x, screen.y + 4);
            mapCtx.lineTo(screen.x, screen.y + 10);
            mapCtx.stroke();
            
            // Label
            mapCtx.fillStyle = '#666';
            mapCtx.fillText(x.toFixed(1), screen.x, screen.y + 12);
        }
    }
    
    // Draw Y-axis ticks and labels (along the left edge of the visible area)
    const xPosForYLabels = leftWorld;
    const yStart = Math.ceil(bottomWorld / step) * step;
    const yEnd = Math.floor(topWorld / step) * step;
    
    mapCtx.textAlign = 'right';
    mapCtx.textBaseline = 'middle';
    
    for (let y = yStart; y <= yEnd; y += step) {
        if (Math.abs(y) < 0.001) continue; // Skip origin (already labeled)
        const screen = worldToMapScreen(xPosForYLabels, y);
        if (screen.y >= 0 && screen.y <= canvasHeight) {
            // Tick mark
            mapCtx.strokeStyle = 'rgba(80, 80, 120, 0.3)';
            mapCtx.lineWidth = 1;
            mapCtx.beginPath();
            mapCtx.moveTo(screen.x - 4, screen.y);
            mapCtx.lineTo(screen.x - 10, screen.y);
            mapCtx.stroke();
            
            // Label
            mapCtx.fillStyle = '#666';
            mapCtx.fillText(y.toFixed(1), screen.x - 12, screen.y);
        }
    }
    
    // Draw corner origin indicator
    mapCtx.textAlign = 'left';
    mapCtx.textBaseline = 'bottom';
    mapCtx.fillStyle = '#888';
    mapCtx.font = '11px Arial';
    mapCtx.fillText('0', 6, canvasHeight - 2);
}

function drawMapCells(mapData) {
    if (!mapData || !mapData.data || !showGridMap) {
        return;
    }

    const mapWidth = mapData.width;
    const mapHeight = mapData.height;
    const resolution = mapData.resolution;
    const mapOriginX = -mapWidth * resolution / 2;
    const mapOriginY = -mapHeight * resolution / 2;
    const cellSize = resolution * mapView.zoom;

    // Skip drawing if cells are too small (performance)
    if (cellSize < 0.5) {
        // Draw a simplified version: only every 4th cell
        const step = 4;
        const visibleWorldWidth = mapCanvas.width / mapView.zoom;
        const visibleWorldHeight = mapCanvas.height / mapView.zoom;
        const leftWorld = mapView.centerX - visibleWorldWidth / 2 - resolution * step;
        const rightWorld = mapView.centerX + visibleWorldWidth / 2 + resolution * step;
        const bottomWorld = mapView.centerY - visibleWorldHeight / 2 - resolution * step;
        const topWorld = mapView.centerY + visibleWorldHeight / 2 + resolution * step;

        const gxStart = Math.max(0, Math.floor((leftWorld - mapOriginX) / resolution));
        const gxEnd = Math.min(mapWidth - 1, Math.ceil((rightWorld - mapOriginX) / resolution));
        const gyStart = Math.max(0, Math.floor((bottomWorld - mapOriginY) / resolution));
        const gyEnd = Math.min(mapHeight - 1, Math.ceil((topWorld - mapOriginY) / resolution));

        for (let gy = gyStart; gy <= gyEnd; gy += step) {
            for (let gx = gxStart; gx <= gxEnd; gx += step) {
                const value = mapData.data[gy * mapWidth + gx];
                if (value < 0) continue;

                const worldX = mapOriginX + gx * resolution;
                const worldY = mapOriginY + gy * resolution;
                const screen = worldToMapScreen(worldX, worldY);
                const size = cellSize * step;

                if (screen.x + size < 0 || screen.x > mapCanvas.width || 
                    screen.y + size < 0 || screen.y > mapCanvas.height) {
                    continue;
                }

                if (value >= 30) {
                    mapCtx.fillStyle = 'rgba(255, 68, 68, 0.85)';
                } else {
                    mapCtx.fillStyle = 'rgba(101, 181, 255, 0.12)';
                }
                mapCtx.fillRect(screen.x, screen.y, size, size);
            }
        }
        return;
    }

    // Normal drawing when cells are large enough
    const visibleWorldWidth = mapCanvas.width / mapView.zoom;
    const visibleWorldHeight = mapCanvas.height / mapView.zoom;
    const leftWorld = mapView.centerX - visibleWorldWidth / 2 - resolution;
    const rightWorld = mapView.centerX + visibleWorldWidth / 2 + resolution;
    const bottomWorld = mapView.centerY - visibleWorldHeight / 2 - resolution;
    const topWorld = mapView.centerY + visibleWorldHeight / 2 + resolution;

    const gxStart = Math.max(0, Math.floor((leftWorld - mapOriginX) / resolution));
    const gxEnd = Math.min(mapWidth - 1, Math.ceil((rightWorld - mapOriginX) / resolution));
    const gyStart = Math.max(0, Math.floor((bottomWorld - mapOriginY) / resolution));
    const gyEnd = Math.min(mapHeight - 1, Math.ceil((topWorld - mapOriginY) / resolution));

    // Batch draw for performance
    const batchSize = 500;
    let batch = [];

    for (let gy = gyStart; gy <= gyEnd; gy++) {
        for (let gx = gxStart; gx <= gxEnd; gx++) {
            const value = mapData.data[gy * mapWidth + gx];
            if (value < 0) continue;

            const worldX = mapOriginX + gx * resolution;
            const worldY = mapOriginY + gy * resolution;
            const screen = worldToMapScreen(worldX, worldY);

            if (screen.x + cellSize < 0 || screen.x > mapCanvas.width || 
                screen.y + cellSize < 0 || screen.y > mapCanvas.height) {
                continue;
            }

            const color = value >= 30 ? 
                'rgba(255, 68, 68, 0.85)' : 
                'rgba(101, 181, 255, 0.12)';
            
            batch.push({x: screen.x, y: screen.y, w: cellSize, h: cellSize, color: color});
            
            if (batch.length >= batchSize) {
                for (const item of batch) {
                    mapCtx.fillStyle = item.color;
                    mapCtx.fillRect(item.x, item.y, item.w, item.h);
                }
                batch = [];
            }
        }
    }
    
    // Flush remaining batch
    for (const item of batch) {
        mapCtx.fillStyle = item.color;
        mapCtx.fillRect(item.x, item.y, item.w, item.h);
    }
}

function drawRobotOnMap() {
    const screen = worldToMapScreen(robotX, robotY);
    const heading = robotTheta;
    const forwardX = Math.cos(heading);
    const forwardY = Math.sin(heading);
    const leftX = -Math.sin(heading);
    const leftY = Math.cos(heading);

    const robotSize = 0.18;
    const nose = {
        x: screen.x + forwardX * robotSize * mapView.zoom,
        y: screen.y - forwardY * robotSize * mapView.zoom,
    };
    const leftPoint = {
        x: screen.x - forwardX * robotSize * 0.6 * mapView.zoom + leftX * robotSize * 0.7 * mapView.zoom,
        y: screen.y + forwardY * robotSize * 0.6 * mapView.zoom - leftY * robotSize * 0.7 * mapView.zoom,
    };
    const rightPoint = {
        x: screen.x - forwardX * robotSize * 0.6 * mapView.zoom - leftX * robotSize * 0.7 * mapView.zoom,
        y: screen.y + forwardY * robotSize * 0.6 * mapView.zoom + leftY * robotSize * 0.7 * mapView.zoom,
    };

    mapCtx.fillStyle = '#00ff88';
    mapCtx.beginPath();
    mapCtx.arc(screen.x, screen.y, 6, 0, 2 * Math.PI);
    mapCtx.fill();

    mapCtx.strokeStyle = '#00ff88';
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    mapCtx.moveTo(screen.x, screen.y);
    mapCtx.lineTo(nose.x, nose.y);
    mapCtx.stroke();

    mapCtx.fillStyle = '#00ff88';
    mapCtx.beginPath();
    mapCtx.moveTo(nose.x, nose.y);
    mapCtx.lineTo(leftPoint.x, leftPoint.y);
    mapCtx.lineTo(rightPoint.x, rightPoint.y);
    mapCtx.closePath();
    mapCtx.fill();
}

function renderMapView() {
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    mapCtx.fillStyle = '#f4f7fb';
    mapCtx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);

    // Draw axes instead of grid
    drawMapAxes();

    // Draw map cells (if enabled)
    if (currentMap) {
        drawMapCells(currentMap);
    }

    drawRobotOnMap();

    // Draw planned path if available
    drawPath(currentPath);

    mapCtx.fillStyle = '#1f2937';
    mapCtx.font = '12px Arial';
    mapCtx.fillText(`zoom: ${mapView.zoom.toFixed(0)} px/m`, 12, 18);
    mapCtx.fillText(`pose: (${robotX.toFixed(2)}, ${robotY.toFixed(2)})`, 12, 34);
    mapCtx.fillStyle = '#666';
    mapCtx.font = '10px Arial';
    mapCtx.fillText(`Grid: ${showGridMap ? 'ON' : 'OFF'}`, 12, 52);
}

function renderActiveView() {
    if (activeView === 'map') {
        renderMapView();
    } else {
        renderRadarView();
    }
}

function updateScanRate(currentTime) {
    if (lastScanTime > 0) {
        const delta = currentTime - lastScanTime;
        if (delta > 0) {
            const instantRate = 1.0 / delta;
            scanRate = scanRate * 0.8 + instantRate * 0.2;
            document.getElementById('scanRate').innerHTML = scanRate.toFixed(1) + ' Hz';
        }
    }

    lastScanTime = currentTime;
    scanCount++;
}

function connectWebSocket() {
    const url = document.getElementById('wsUrl').value;
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
            const data = JSON.parse(event.data);

            if (data.type === 'lidar_scan') {
                currentRanges = data.ranges || [];
                currentAngles = data.angles || [];
                maxRange = data.max_range !== undefined ? data.max_range : maxRange;

                robotX = data.robot_x !== undefined ? data.robot_x : 0;
                robotY = data.robot_y !== undefined ? data.robot_y : 0;
                robotTheta = data.robot_theta !== undefined ? data.robot_theta : 0;
                rawRobotX = data.raw_robot_x !== undefined ? data.raw_robot_x : robotX;
                rawRobotY = data.raw_robot_y !== undefined ? data.raw_robot_y : robotY;
                rawRobotTheta = data.raw_robot_theta !== undefined ? data.raw_robot_theta : robotTheta;
                slamMatchScore = data.slam_match_score !== undefined ? data.slam_match_score : 0;

                const leftSpeed = data.left_speed !== undefined ? data.left_speed : 0;
                const rightSpeed = data.right_speed !== undefined ? data.right_speed : 0;
                const autoMode = data.auto_navigate !== undefined ? data.auto_navigate : true;
                const linearVel = data.linear_vel !== undefined ? data.linear_vel : 0;

                updateScanRate(data.timestamp || 0);

                document.getElementById('numPoints').innerHTML = (data.num_points || 0).toLocaleString();
                document.getElementById('rangeLimit').innerHTML = `${(data.min_range || 0).toFixed(1)}-${(data.max_range || 0).toFixed(1)} m`;
                document.getElementById('lastScan').innerHTML = (data.timestamp || 0).toFixed(2) + ' s';
                document.getElementById('leftSpeed').innerHTML = leftSpeed.toFixed(2) + ' m/s';
                document.getElementById('rightSpeed').innerHTML = rightSpeed.toFixed(2) + ' m/s';
                document.getElementById('robotMode').innerHTML = autoMode ? '🤖 Auto' : '🎮 Manual';
                document.getElementById('posX').innerHTML = robotX.toFixed(2) + ' m';
                document.getElementById('posY').innerHTML = robotY.toFixed(2) + ' m';
                document.getElementById('theta').innerHTML = (robotTheta * 180 / Math.PI).toFixed(1) + ' °';
                document.getElementById('linearVel').innerHTML = linearVel.toFixed(2) + ' m/s';

                if (data.map) {
                    currentMap = data.map;
                }

                // Update path if present
                if (data.path) {
                    currentPath = data.path.map(p => ({ x: p.x, y: p.y }));
                } else {
                    currentPath = [];
                }

                renderActiveView();

                // Debug: log occasionally
                if (scanCount % 100 === 0) {
                    console.log(`[Update] pose=(${robotX.toFixed(2)}, ${robotY.toFixed(2)}), score=${slamMatchScore.toFixed(3)}`);
                }
            } else if (data.type === 'robot_info') {
                document.getElementById('leftSpeed').innerHTML = (data.left_speed || 0).toFixed(2) + ' m/s';
                document.getElementById('rightSpeed').innerHTML = (data.right_speed || 0).toFixed(2) + ' m/s';
                document.getElementById('robotMode').innerHTML = data.auto_navigate ? '🤖 Auto' : '🎮 Manual';
                document.getElementById('posX').innerHTML = (data.x || 0).toFixed(2) + ' m';
                document.getElementById('posY').innerHTML = (data.y || 0).toFixed(2) + ' m';
                document.getElementById('theta').innerHTML = ((data.theta || 0) * 180 / Math.PI).toFixed(1) + ' °';
                document.getElementById('linearVel').innerHTML = (data.linear_vel || 0).toFixed(2) + ' m/s';
            }
        } catch (error) {
            console.error('[WebSocket] Parse error:', error);
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

function disconnectWebSocket() {
    console.log('[WebSocket] Manual disconnect');
    if (ws) {
        ws.close();
        ws = null;
    }
}

function sendCommand(command) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        console.warn('[Command] Not connected');
        return;
    }

    const cmd = { type: 'command', command: command };
    ws.send(JSON.stringify(cmd));
    console.log('[Command] Sent:', command);
}

function enableManualDriving() {
    manualDriving = true;
    const btn = document.getElementById('driveManualBtn');
    btn.disabled = true;
    document.getElementById('keyboardHint').style.display = 'block';
    console.log('[Manual] Keyboard control enabled - Arrow keys to drive, Space=stop, Q=quit');
    
    // Send a message to the robot to switch to manual mode if needed
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'autonomy', auto_navigate: false }));
    }
}

function disableManualDriving() {
    manualDriving = false;
    const btn = document.getElementById('driveManualBtn');
    btn.disabled = false;
    document.getElementById('keyboardHint').style.display = 'none';
    console.log('[Manual] Keyboard control disabled');
}

function setupKeyboardControls() {
    document.addEventListener('keydown', function(event) {
        // Toggle manual driving off with 'q'
        if (event.key === 'q' || event.key === 'Q') {
            if (manualDriving) {
                disableManualDriving();
                // Send stop command when quitting manual mode
                sendCommand('stop');
            }
            return;
        }

        // Only forward movement commands while manual driving is enabled
        if (manualDriving) {
            switch (event.key) {
                case 'ArrowUp':
                    sendCommand('forward');
                    event.preventDefault();
                    break;
                case 'ArrowDown':
                    sendCommand('backward');
                    event.preventDefault();
                    break;
                case 'ArrowLeft':
                    sendCommand('left');
                    event.preventDefault();
                    break;
                case 'ArrowRight':
                    sendCommand('right');
                    event.preventDefault();
                    break;
                case ' ':
                    sendCommand('stop');
                    event.preventDefault();
                    break;
                case 'a':
                case 'A':
                    sendCommand('auto');
                    event.preventDefault();
                    break;
                case 'm':
                case 'M':
                    showView('map');
                    event.preventDefault();
                    break;
                case 'r':
                case 'R':
                    showView('radar');
                    event.preventDefault();
                    break;
            }
        }
    });
}

function setupMapInteractions() {
    mapCanvas.addEventListener('mousedown', function(event) {
        if (activeView !== 'map') {
            return;
        }

        mapView.dragging = true;
        mapView.dragStartX = event.clientX;
        mapView.dragStartY = event.clientY;
        mapView.dragOriginX = mapView.centerX;
        mapView.dragOriginY = mapView.centerY;
    });

    window.addEventListener('mousemove', function(event) {
        if (!mapView.dragging) {
            return;
        }

        const deltaX = event.clientX - mapView.dragStartX;
        const deltaY = event.clientY - mapView.dragStartY;

        mapView.centerX = mapView.dragOriginX - deltaX / mapView.zoom;
        mapView.centerY = mapView.dragOriginY - deltaY / mapView.zoom;

        if (activeView === 'map') {
            renderMapView();
        }
    });

    window.addEventListener('mouseup', function() {
        mapView.dragging = false;
    });

    mapCanvas.addEventListener('wheel', function(event) {
        if (activeView !== 'map') {
            return;
        }

        event.preventDefault();

        const rect = mapCanvas.getBoundingClientRect();
        const scaleX = mapCanvas.width / rect.width;
        const scaleY = mapCanvas.height / rect.height;
        
        const mouseX = (event.clientX - rect.left) * scaleX;
        const mouseY = (event.clientY - rect.top) * scaleY;

        const zoomFactor = event.deltaY < 0 ? 1.12 : 1 / 1.12;
        
        const mouseWorld = mapScreenToWorld(mouseX, mouseY);
        
        mapView.zoom = clamp(mapView.zoom * zoomFactor, 10, 180);
        
        mapView.centerX = mouseWorld.x - (mouseX - mapCanvas.width / 2) / mapView.zoom;
        mapView.centerY = mouseWorld.y - (mouseY - mapCanvas.height / 2) / mapView.zoom;

        renderMapView();
    }, { passive: false });
    
    // Double-click to set a goal
    mapCanvas.addEventListener('dblclick', function(event) {
        if (activeView !== 'map') return;
        const rect = mapCanvas.getBoundingClientRect();
        const scaleX = mapCanvas.width / rect.width;
        const scaleY = mapCanvas.height / rect.height;
        const canvasX = (event.clientX - rect.left) * scaleX;
        const canvasY = (event.clientY - rect.top) * scaleY;
        const world = mapScreenToWorld(canvasX, canvasY);
        console.log('[Map] Goal set at', world.x.toFixed(2), world.y.toFixed(2));
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'set_goal', x: world.x, y: world.y }));
        }
    });
}

window.addEventListener('load', function() {
    console.log('[App] Initializing...');
    initCanvases();
    initZoomSlider();
    setupKeyboardControls();
    setupMapInteractions();

    document.getElementById('wsUrl').value = 'ws://localhost:8766';

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
    document.getElementById('linearVel').innerHTML = '0.00 m/s';

    updateZoomDisplay();
    showView('map');

    // Set initial grid toggle button text
    document.getElementById('toggleGridBtn').textContent = 'Hide Grid';

    setTimeout(() => {
        console.log('[App] Auto-connecting...');
        connectWebSocket();
    }, 1000);

    document.getElementById('wsUrl').addEventListener('keypress', function(event) {
        if (event.key === 'Enter') {
            connectWebSocket();
        }
    });
});

function drawPath(path) {
    if (!path || path.length === 0) return;
    mapCtx.strokeStyle = '#ff8800';
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    for (let i = 0; i < path.length; i++) {
        const p = path[i];
        const s = worldToMapScreen(p.x, p.y);
        if (i === 0) mapCtx.moveTo(s.x, s.y);
        else mapCtx.lineTo(s.x, s.y);
    }
    mapCtx.stroke();

    // Draw points
    for (let i = 0; i < path.length; i++) {
        const p = path[i];
        const s = worldToMapScreen(p.x, p.y);
        mapCtx.fillStyle = '#ff4400';
        mapCtx.beginPath();
        mapCtx.arc(s.x, s.y, 4, 0, 2 * Math.PI);
        mapCtx.fill();
    }
}