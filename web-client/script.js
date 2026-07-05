// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  STATE
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

let ws = null;
let activeView = 'radar';
let currentMode = 'idle'; // 'idle' | 'manual' | 'auto'

// Canvas refs
let radarCanvas = null;
let radarCtx = null;
let mapCanvas = null;
let mapCtx = null;

// Radar
let radarWidth = 0;
let radarHeight = 0;
let radarCenterX = 0;
let radarCenterY = 0;
let radarScale = 96;

// Map
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

// Data
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
let currentGoal = null;
let currentCoarseGrid = null;
let showGridMap = true;

// Stats
let lastScanTime = 0;
let scanCount = 0;
let scanRate = 0;
let reconnectAttempts = 0;
const maxReconnectAttempts = 5;

// ── helpers ──
function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  INIT
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

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
    const label = document.getElementById('zoomRange');
    if (label) label.innerHTML = displayRange.toFixed(1) + ' m';
}

function setRadarZoom(zoomScale) {
    radarScale = zoomScale;
    document.getElementById('zoomSlider').value = radarScale;
    updateZoomDisplay();
    if (activeView === 'radar') renderRadarView();
}

function initZoomSlider() {
    const slider = document.getElementById('zoomSlider');
    slider.addEventListener('input', (e) => setRadarZoom(parseFloat(e.target.value)));

    const mapSlider = document.getElementById('mapZoomSlider');
    if (mapSlider) {
        mapSlider.addEventListener('input', (e) => {
            const v = parseFloat(e.target.value);
            mapView.zoom = v;
            document.getElementById('mapZoomValue').textContent = v.toFixed(0);
            if (activeView === 'map') renderMapView();
        });
    }
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  VIEWS
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

function showView(viewName) {
    activeView = viewName;
    document.getElementById('radarTab').classList.toggle('active', viewName === 'radar');
    document.getElementById('mapTab').classList.toggle('active', viewName === 'map');
    document.getElementById('radarPanel').classList.toggle('active', viewName === 'radar');
    document.getElementById('mapPanel').classList.toggle('active', viewName === 'map');
    if (viewName === 'map') renderMapView();
    else renderRadarView();
}

// ── radar ──
function worldToRadarScreen(range, angle) {
    const forward = range * Math.cos(angle);
    const left = range * Math.sin(angle);
    return {
        x: radarCenterX + left * radarScale,
        y: radarCenterY - forward * radarScale,
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
        if (radiusPx > Math.min(radarWidth, radarHeight) / 2) break;
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
    if (!ranges || !angles) return;
    const displayRange = getDisplayRange();
    for (let i = 0; i < ranges.length; i++) {
        const range = ranges[i];
        const angle = angles[i];
        if (range >= maxRange || range <= 0.1 || range > displayRange) continue;
        const screen = worldToRadarScreen(range, angle);
        if (screen.x >= 0 && screen.x < radarWidth && screen.y >= 0 && screen.y < radarHeight) {
            const t = Math.min(1, range / maxRange);
            const red = 255;
            const green = Math.floor(255 * t);
            const blue = Math.floor(100 * (1 - t));
            radarCtx.fillStyle = `rgb(${red},${green},${blue})`;
            radarCtx.fillRect(screen.x - 1.5, screen.y - 1.5, 3, 3);
        }
    }
}

function renderRadarView() {
    renderRadarBackground();
    drawRadarPoints(currentRanges, currentAngles);
    drawRadarRobot();
}

// ── map ──
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
    if (activeView === 'map') renderMapView();
}

function centerMapOnRobot() {
    mapView.centerX = robotX;
    mapView.centerY = robotY;
    if (activeView === 'map') renderMapView();
}

function toggleGridMap() {
    showGridMap = !showGridMap;
    document.getElementById('toggleGridBtn').textContent = showGridMap ? 'Hide Grid' : 'Show Grid';
    if (activeView === 'map') renderMapView();
}

function drawCoarseGrid(coarseGridData) {
    if (!coarseGridData || !coarseGridData.data || !showGridMap) return;
    const mapWidth = coarseGridData.width;
    const mapHeight = coarseGridData.height;
    const resolution = coarseGridData.resolution;
    const originX = coarseGridData.origin_x || -mapWidth * resolution / 2;
    const originY = coarseGridData.origin_y || -mapHeight * resolution / 2;
    const cellSize = resolution * mapView.zoom;
    const step = cellSize < 1.0 ? 2 : 1;

    const visibleWorldWidth = mapCanvas.width / mapView.zoom;
    const visibleWorldHeight = mapCanvas.height / mapView.zoom;
    const leftWorld = mapView.centerX - visibleWorldWidth / 2 - resolution * step;
    const rightWorld = mapView.centerX + visibleWorldWidth / 2 + resolution * step;
    const bottomWorld = mapView.centerY - visibleWorldHeight / 2 - resolution * step;
    const topWorld = mapView.centerY + visibleWorldHeight / 2 + resolution * step;

    const gxStart = Math.max(0, Math.floor((leftWorld - originX) / resolution));
    const gxEnd = Math.min(mapWidth - 1, Math.ceil((rightWorld - originX) / resolution));
    const gyStart = Math.max(0, Math.floor((bottomWorld - originY) / resolution));
    const gyEnd = Math.min(mapHeight - 1, Math.ceil((topWorld - originY) / resolution));

    for (let gy = gyStart; gy <= gyEnd; gy += step) {
        for (let gx = gxStart; gx <= gxEnd; gx += step) {
            const value = coarseGridData.data[gy * mapWidth + gx];
            if (value !== 1) continue;
            const worldX = originX + gx * resolution;
            const worldY = originY + gy * resolution;
            const screen = worldToMapScreen(worldX, worldY);
            const size = cellSize * step;
            if (screen.x + size < 0 || screen.x > mapCanvas.width ||
                screen.y + size < 0 || screen.y > mapCanvas.height) continue;
            mapCtx.fillStyle = 'rgba(255, 200, 0, 0.3)';
            mapCtx.strokeStyle = 'rgba(255, 200, 0, 0.5)';
            mapCtx.lineWidth = 0.5;
            mapCtx.fillRect(screen.x, screen.y, size, size);
            mapCtx.strokeRect(screen.x, screen.y, size, size);
        }
    }
}

function niceStep(range, targetTicks = 6) {
    const rough = range / targetTicks;
    const magnitude = Math.pow(10, Math.floor(Math.log10(rough)));
    const normalized = rough / magnitude;
    let nice;
    if (normalized < 1.5) nice = 1;
    else if (normalized < 3.5) nice = 2;
    else if (normalized < 7.5) nice = 5;
    else nice = 10;
    return nice * magnitude;
}

function drawMapAxes() {
    const canvasWidth = mapCanvas.width;
    const canvasHeight = mapCanvas.height;
    const visibleWorldWidth = canvasWidth / mapView.zoom;
    const visibleWorldHeight = canvasHeight / mapView.zoom;
    const leftWorld = mapView.centerX - visibleWorldWidth / 2;
    const rightWorld = mapView.centerX + visibleWorldWidth / 2;
    const bottomWorld = mapView.centerY - visibleWorldHeight / 2;
    const topWorld = mapView.centerY + visibleWorldHeight / 2;

    const rangeX = visibleWorldWidth;
    const rangeY = visibleWorldHeight;
    const stepx = niceStep(rangeX, 12);
    const stepy = niceStep(rangeY, 15);

    // Debug every 10th render
    const debug = (scanCount % 10 === 0);

    const originScreen = worldToMapScreen(0, 0);
    const originVisible = (leftWorld <= 0 && rightWorld >= 0 && bottomWorld <= 0 && topWorld >= 0);

    // axes lines
    mapCtx.strokeStyle = 'rgba(100,100,150,0.4)';
    mapCtx.lineWidth = 1.5;

    if (leftWorld <= 0 && rightWorld >= 0) {
        mapCtx.beginPath();
        mapCtx.moveTo(originScreen.x, 0);
        mapCtx.lineTo(originScreen.x, canvasHeight);
        mapCtx.stroke();
        mapCtx.fillStyle = '#555';
        mapCtx.font = 'bold 12px Arial';
        mapCtx.textAlign = 'left';
        mapCtx.textBaseline = 'bottom';
        mapCtx.fillText('Y', originScreen.x + 6, 18);
    }
    if (bottomWorld <= 0 && topWorld >= 0) {
        mapCtx.beginPath();
        mapCtx.moveTo(0, originScreen.y);
        mapCtx.lineTo(canvasWidth, originScreen.y);
        mapCtx.stroke();
        mapCtx.fillStyle = '#555';
        mapCtx.font = 'bold 12px Arial';
        mapCtx.textAlign = 'right';
        mapCtx.textBaseline = 'bottom';
        mapCtx.fillText('X', canvasWidth - 8, originScreen.y - 4);
    }

    // Y ticks (left) – draw at left edge
    const xPosForYLabels = leftWorld;
    const yStart = Math.ceil(bottomWorld / stepy) * stepy;
    const yEnd = Math.floor(topWorld / stepy) * stepy;

    if (debug) {
        console.log(`[Y] bottom=${bottomWorld.toFixed(6)}, top=${topWorld.toFixed(6)}, stepy=${stepy.toFixed(6)}`);
        console.log(`[Y] yStart=${yStart.toFixed(6)}, yEnd=${yEnd.toFixed(6)}, ticks=${yStart <= yEnd ? ((yEnd - yStart) / stepy + 1).toFixed(0) : 0}`);
    }

    if (yStart <= yEnd) {
        let drawn = 0;
        for (let y = yStart; y <= yEnd; y += stepy) {
            if (Math.abs(y) < 0.001) continue;
            const s = worldToMapScreen(xPosForYLabels, y);
            // Only check y is within canvas; x is always at left edge (≈0)
            if (s.y >= 0 && s.y <= canvasHeight) {
                // Draw tick from x=0 to x=6 (inward)
                mapCtx.strokeStyle = 'rgba(80,80,120,0.3)';
                mapCtx.lineWidth = 1;
                mapCtx.beginPath();
                mapCtx.moveTo(0, s.y);
                mapCtx.lineTo(6, s.y);
                mapCtx.stroke();
                // Draw label at x=10 (inside), aligned left
                mapCtx.fillStyle = '#666';
                mapCtx.textAlign = 'left';
                mapCtx.textBaseline = 'middle';
                mapCtx.fillText(y.toFixed(1), 10, s.y);
                drawn++;
            }
        }
        if (debug) console.log(`[Y] drawn ${drawn} visible ticks`);
    } else {
        if (debug) console.warn('[Y] ❌ NO TICKS – yStart > yEnd');
    }

    // X ticks (top) – draw at top edge
    const yPosForXLabelsTop = topWorld;
    const xStart = Math.ceil(leftWorld / stepx) * stepx;
    const xEnd = Math.floor(rightWorld / stepx) * stepx;

    if (debug) {
        console.log(`[X] left=${leftWorld.toFixed(6)}, right=${rightWorld.toFixed(6)}, stepx=${stepx.toFixed(6)}`);
        console.log(`[X] xStart=${xStart.toFixed(6)}, xEnd=${xEnd.toFixed(6)}, ticks=${xStart <= xEnd ? ((xEnd - xStart) / stepx + 1).toFixed(0) : 0}`);
    }

    if (xStart <= xEnd) {
        let drawn = 0;
        for (let x = xStart; x <= xEnd; x += stepx) {
            if (Math.abs(x) < 0.001) continue;
            const s = worldToMapScreen(x, yPosForXLabelsTop);
            // Only check x is within canvas; y is always at top edge (≈0)
            if (s.x >= 0 && s.x <= canvasWidth) {
                // Draw tick from y=0 to y=6 (downward)
                mapCtx.strokeStyle = 'rgba(80,80,120,0.3)';
                mapCtx.lineWidth = 1;
                mapCtx.beginPath();
                mapCtx.moveTo(s.x, 0);
                mapCtx.lineTo(s.x, 6);
                mapCtx.stroke();
                // Draw label at y=10 (inside), aligned center
                mapCtx.fillStyle = '#666';
                mapCtx.textAlign = 'center';
                mapCtx.textBaseline = 'top';
                mapCtx.fillText(x.toFixed(1), s.x, 10);
                drawn++;
            }
        }
        if (debug) console.log(`[X] drawn ${drawn} visible ticks`);
    } else {
        if (debug) console.warn('[X] ❌ NO TICKS – xStart > xEnd');
    }

    // origin label
    if (originVisible) {
        mapCtx.textAlign = 'left';
        mapCtx.textBaseline = 'bottom';
        mapCtx.fillStyle = '#888';
        mapCtx.font = '10px Arial';
        mapCtx.fillText('0', originScreen.x + 4, originScreen.y - 4);
    }

    // arrows
    mapCtx.fillStyle = 'rgba(100,100,150,0.5)';
    if (bottomWorld <= 0 && topWorld >= 0) {
        const rightEdgeX = worldToMapScreen(rightWorld - 0.1, 0);
        if (rightEdgeX.x < canvasWidth - 20) {
            mapCtx.beginPath();
            mapCtx.moveTo(rightEdgeX.x + 10, rightEdgeX.y);
            mapCtx.lineTo(rightEdgeX.x + 2, rightEdgeX.y - 5);
            mapCtx.lineTo(rightEdgeX.x + 2, rightEdgeX.y + 5);
            mapCtx.closePath();
            mapCtx.fill();
        }
    }
    if (leftWorld <= 0 && rightWorld >= 0) {
        const topEdgeY = worldToMapScreen(0, topWorld - 0.1);
        if (topEdgeY.y > 20) {
            mapCtx.beginPath();
            mapCtx.moveTo(topEdgeY.x, topEdgeY.y - 10);
            mapCtx.lineTo(topEdgeY.x - 5, topEdgeY.y - 2);
            mapCtx.lineTo(topEdgeY.x + 5, topEdgeY.y - 2);
            mapCtx.closePath();
            mapCtx.fill();
        }
    }
}

function drawMapCells(mapData) {
    if (!mapData || !mapData.data) return;
    const mapWidth = mapData.width;
    const mapHeight = mapData.height;
    const resolution = mapData.resolution;
    const mapOriginX = -mapWidth * resolution / 2;
    const mapOriginY = -mapHeight * resolution / 2;
    const cellSize = resolution * mapView.zoom;

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

    // if cells are tiny, skip every 4th
    const drawStep = cellSize < 0.5 ? 4 : 1;

    for (let gy = gyStart; gy <= gyEnd; gy += drawStep) {
        for (let gx = gxStart; gx <= gxEnd; gx += drawStep) {
            const value = mapData.data[gy * mapWidth + gx];
            if (value < 0) continue;
            const worldX = mapOriginX + gx * resolution;
            const worldY = mapOriginY + gy * resolution;
            const screen = worldToMapScreen(worldX, worldY);
            const size = cellSize * drawStep;
            if (screen.x + size < 0 || screen.x > mapCanvas.width ||
                screen.y + size < 0 || screen.y > mapCanvas.height) continue;
            mapCtx.fillStyle = value >= 30 ? 'rgba(255,68,68,0.85)' : 'rgba(101,181,255,0.12)';
            mapCtx.fillRect(screen.x, screen.y, size, size);
        }
    }
}

function drawRobotOnMap() {
    const screen = worldToMapScreen(robotX, robotY);
    const heading = robotTheta + Math.PI;
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

function drawPath(path) {
    // goal
    if (currentGoal) {
        const gs = worldToMapScreen(currentGoal.x, currentGoal.y);
        const pulseRadius = 10 + 4 * Math.sin(Date.now() / 500);
        const grad = mapCtx.createRadialGradient(gs.x, gs.y, 0, gs.x, gs.y, 20);
        grad.addColorStop(0, 'rgba(255,0,0,0.6)');
        grad.addColorStop(0.5, 'rgba(255,0,0,0.2)');
        grad.addColorStop(1, 'rgba(255,0,0,0)');
        mapCtx.fillStyle = grad;
        mapCtx.beginPath();
        mapCtx.arc(gs.x, gs.y, 20, 0, 2 * Math.PI);
        mapCtx.fill();

        mapCtx.fillStyle = '#ff0000';
        mapCtx.beginPath();
        mapCtx.arc(gs.x, gs.y, 6, 0, 2 * Math.PI);
        mapCtx.fill();

        mapCtx.strokeStyle = '#ff4444';
        mapCtx.lineWidth = 2;
        mapCtx.beginPath();
        mapCtx.arc(gs.x, gs.y, 8, 0, 2 * Math.PI);
        mapCtx.stroke();

        mapCtx.fillStyle = '#ff0000';
        mapCtx.font = 'bold 11px Arial';
        mapCtx.textAlign = 'left';
        mapCtx.textBaseline = 'bottom';
        mapCtx.fillText('🎯 GOAL', gs.x + 12, gs.y - 4);
        mapCtx.fillStyle = '#cc0000';
        mapCtx.font = '9px Arial';
        mapCtx.textAlign = 'left';
        mapCtx.textBaseline = 'top';
        mapCtx.fillText(`(${currentGoal.x.toFixed(2)}, ${currentGoal.y.toFixed(2)})`, gs.x + 12, gs.y + 4);
    }

    if (!path || path.length === 0) return;

    // dashed line from robot to first waypoint
    const robotScreen = worldToMapScreen(robotX, robotY);
    const firstPoint = path[0];
    const firstScreen = worldToMapScreen(firstPoint.x, firstPoint.y);
    const dx = firstPoint.x - robotX;
    const dy = firstPoint.y - robotY;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > 0.1) {
        mapCtx.setLineDash([6, 6]);
        mapCtx.strokeStyle = 'rgba(255,136,0,0.4)';
        mapCtx.lineWidth = 1.5;
        mapCtx.beginPath();
        mapCtx.moveTo(robotScreen.x, robotScreen.y);
        mapCtx.lineTo(firstScreen.x, firstScreen.y);
        mapCtx.stroke();
        mapCtx.setLineDash([]);

        const midX = (robotScreen.x + firstScreen.x) / 2;
        const midY = (robotScreen.y + firstScreen.y) / 2;
        const distText = dist.toFixed(2) + 'm';
        mapCtx.fillStyle = 'rgba(244,247,251,0.8)';
        const metrics = mapCtx.measureText(distText);
        const pad = 4;
        mapCtx.fillRect(midX - metrics.width / 2 - pad, midY - 16 - pad, metrics.width + pad * 2, 16 + pad * 2);
        mapCtx.fillStyle = 'rgba(255,136,0,0.8)';
        mapCtx.font = '9px Arial';
        mapCtx.textAlign = 'center';
        mapCtx.textBaseline = 'bottom';
        mapCtx.fillText(distText, midX, midY - 4);
    }

    // path line
    mapCtx.strokeStyle = '#ff8800';
    mapCtx.lineWidth = 2.5;
    mapCtx.setLineDash([]);
    mapCtx.beginPath();
    for (let i = 0; i < path.length; i++) {
        const s = worldToMapScreen(path[i].x, path[i].y);
        if (i === 0) mapCtx.moveTo(s.x, s.y);
        else mapCtx.lineTo(s.x, s.y);
    }
    mapCtx.stroke();

    // waypoints
    for (let i = 0; i < path.length; i++) {
        const s = worldToMapScreen(path[i].x, path[i].y);
        const color = (i === 0) ? '#ff8800' : (i === path.length - 1 ? '#ff4400' : '#ffaa44');
        const size = (i === 0 || i === path.length - 1) ? 5 : 3;
        mapCtx.fillStyle = 'rgba(255,136,0,0.15)';
        mapCtx.beginPath();
        mapCtx.arc(s.x, s.y, size + 6, 0, 2 * Math.PI);
        mapCtx.fill();
        mapCtx.fillStyle = color;
        mapCtx.beginPath();
        mapCtx.arc(s.x, s.y, size, 0, 2 * Math.PI);
        mapCtx.fill();
        if (path.length <= 20) {
            mapCtx.fillStyle = 'rgba(0,0,0,0.6)';
            mapCtx.font = '8px Arial';
            mapCtx.textAlign = 'center';
            mapCtx.textBaseline = 'bottom';
            mapCtx.fillText((i + 1).toString(), s.x, s.y - size - 4);
        }
    }

    // total distance
    let totalDist = 0;
    for (let i = 1; i < path.length; i++) {
        const dx = path[i].x - path[i - 1].x;
        const dy = path[i].y - path[i - 1].y;
        totalDist += Math.sqrt(dx * dx + dy * dy);
    }
    if (path.length > 1 && totalDist > 0.1) {
        const last = path[path.length - 1];
        const ls = worldToMapScreen(last.x, last.y);
        const label = '📏 ' + totalDist.toFixed(2) + 'm';
        mapCtx.fillStyle = 'rgba(244,247,251,0.85)';
        const metrics = mapCtx.measureText(label);
        const pad = 6;
        mapCtx.fillRect(ls.x - metrics.width / 2 - pad, ls.y + 12 - pad, metrics.width + pad * 2, 20 + pad * 2);
        mapCtx.fillStyle = '#cc6600';
        mapCtx.font = '10px Arial';
        mapCtx.textAlign = 'center';
        mapCtx.textBaseline = 'top';
        mapCtx.fillText(label, ls.x, ls.y + 14);
    }
}

function renderMapView() {
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    mapCtx.fillStyle = '#f4f7fb';
    mapCtx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);

    drawMapAxes();
    if (currentMap) drawMapCells(currentMap);
    drawCoarseGrid(currentCoarseGrid);
    drawRobotOnMap();
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
    if (activeView === 'map') renderMapView();
    else renderRadarView();
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  MODE SELECTOR (core)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

function setMode(mode, silent = false) {
    if (mode === currentMode) return;

    const prevMode = currentMode;
    currentMode = mode;

    // update UI: buttons
    document.querySelectorAll('.mode-btn').forEach((btn) => {
        const isActive = btn.dataset.mode === mode;
        btn.classList.toggle('active', isActive);
    });

    // keyboard hint
    const hint = document.getElementById('keyboardHint');
    if (mode === 'manual') {
        hint.classList.add('visible');
    } else {
        hint.classList.remove('visible');
    }

    // update info panel
    const modeLabels = { idle: 'Idle', manual: 'Manual', auto: 'Auto' };
    document.getElementById('robotMode').innerHTML = modeLabels[mode] || mode;

    // if silent, stop here (used when syncing from server)
    if (silent) return;

    // send command to server
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        console.warn('[Mode] Not connected – mode set locally only');
        return;
    }

    switch (mode) {
        case 'idle':
            sendCommand('stop');
            break;
        case 'manual':
            ws.send(JSON.stringify({ type: 'autonomy', auto_navigate: false }));
            break;
        case 'auto':
            sendCommand('auto');
            break;
        default:
            break;
    }

    console.log(`[Mode] → ${mode}${prevMode ? ` (was ${prevMode})` : ''}`);
}

// ── backward-compatible helpers ──
function enableManualDriving() {
    setMode('manual');
}

function disableManualDriving() {
    setMode('idle');
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  NETWORK
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

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
                document.getElementById('rangeLimit').innerHTML =
                    `${(data.min_range || 0).toFixed(1)}–${(data.max_range || 0).toFixed(1)} m`;
                document.getElementById('lastScan').innerHTML = (data.timestamp || 0).toFixed(2) + ' s';
                document.getElementById('leftSpeed').innerHTML = leftSpeed.toFixed(2) + ' m/s';
                document.getElementById('rightSpeed').innerHTML = rightSpeed.toFixed(2) + ' m/s';
                document.getElementById('posX').innerHTML = robotX.toFixed(2) + ' m';
                document.getElementById('posY').innerHTML = robotY.toFixed(2) + ' m';
                document.getElementById('theta').innerHTML = (robotTheta * 180 / Math.PI).toFixed(1) + ' °';
                document.getElementById('linearVel').innerHTML = linearVel.toFixed(2) + ' m/s';

                // if server sends auto_navigate, sync mode (silent)
                if (data.auto_navigate !== undefined) {
                    const serverMode = data.auto_navigate ? 'auto' : 'manual';
                    // only sync if different, and avoid overriding user's idle choice
                    if (serverMode !== currentMode) {
                        // If server says auto and we're in idle, switch to auto
                        // If server says manual and we're in idle, stay idle (user chose idle)
                        if (serverMode === 'auto') {
                            setMode('auto', true);
                        } else if (serverMode === 'manual' && currentMode !== 'idle') {
                            setMode('manual', true);
                        }
                        // if we're in idle and server says manual, keep idle
                    }
                }

                if (data.map) currentMap = data.map;
                if (data.coarse_grid) {
                    currentCoarseGrid = data.coarse_grid;
                }
                if (data.path) {
                    currentPath = data.path.map(p => ({ x: p.x, y: p.y }));
                } else {
                    currentPath = [];
                }
                if (data.goal) {
                    currentGoal = { x: data.goal.x, y: data.goal.y };
                }

                renderActiveView();

                if (scanCount % 100 === 0) {
                    console.log(`[Update] pose=(${robotX.toFixed(2)},${robotY.toFixed(2)})`);
                }
            } else if (data.type === 'robot_info') {
                document.getElementById('leftSpeed').innerHTML = (data.left_speed || 0).toFixed(2) + ' m/s';
                document.getElementById('rightSpeed').innerHTML = (data.right_speed || 0).toFixed(2) + ' m/s';
                document.getElementById('posX').innerHTML = (data.x || 0).toFixed(2) + ' m';
                document.getElementById('posY').innerHTML = (data.y || 0).toFixed(2) + ' m';
                document.getElementById('theta').innerHTML = ((data.theta || 0) * 180 / Math.PI).toFixed(1) + ' °';
                document.getElementById('linearVel').innerHTML = (data.linear_vel || 0).toFixed(2) + ' m/s';

                // sync mode from server
                if (data.auto_navigate !== undefined) {
                    const serverMode = data.auto_navigate ? 'auto' : 'manual';
                    if (serverMode !== currentMode) {
                        if (serverMode === 'auto') {
                            setMode('auto', true);
                        } else if (serverMode === 'manual' && currentMode !== 'idle') {
                            setMode('manual', true);
                        }
                    }
                }
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

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  KEYBOARD
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

function setupKeyboardControls() {
    document.addEventListener('keydown', function(event) {
        // Q → idle (quit manual)
        if (event.key === 'q' || event.key === 'Q') {
            if (currentMode === 'manual') {
                setMode('idle');
                sendCommand('stop');
            }
            event.preventDefault();
            return;
        }

        // only forward commands in manual mode
        if (currentMode !== 'manual') return;

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
                setMode('auto');
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
            default:
                break;
        }
    });
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  MAP INTERACTIONS
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

function setupMapInteractions() {
    mapCanvas.addEventListener('mousedown', function(event) {
        if (activeView !== 'map') return;
        mapView.dragging = true;
        mapView.dragStartX = event.clientX;
        mapView.dragStartY = event.clientY;
        mapView.dragOriginX = mapView.centerX;
        mapView.dragOriginY = mapView.centerY;
    });

    window.addEventListener('mousemove', function(event) {
        if (!mapView.dragging) return;
        const deltaX = event.clientX - mapView.dragStartX;
        const deltaY = event.clientY - mapView.dragStartY;
        mapView.centerX = mapView.dragOriginX - deltaX / mapView.zoom;
        mapView.centerY = mapView.dragOriginY - deltaY / mapView.zoom;
        if (activeView === 'map') renderMapView();
    });

    window.addEventListener('mouseup', function() {
        mapView.dragging = false;
    });

    mapCanvas.addEventListener('wheel', function(event) {
        if (activeView !== 'map') return;
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

    mapCanvas.addEventListener('dblclick', function(event) {
        if (activeView !== 'map') return;
        const rect = mapCanvas.getBoundingClientRect();
        const scaleX = mapCanvas.width / rect.width;
        const scaleY = mapCanvas.height / rect.height;
        const canvasX = (event.clientX - rect.left) * scaleX;
        const canvasY = (event.clientY - rect.top) * scaleY;
        const world = mapScreenToWorld(canvasX, canvasY);
        currentGoal = { x: world.x, y: world.y };
        console.log('[Map] Goal set at', world.x.toFixed(2), world.y.toFixed(2));
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'set_goal', x: world.x, y: world.y }));
        }
        if (activeView === 'map') renderMapView();
    });
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  BOOT
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

window.addEventListener('load', function() {
    console.log('[App] Initializing...');
    initCanvases();
    initZoomSlider();
    setupKeyboardControls();
    setupMapInteractions();

    document.getElementById('wsUrl').value = 'ws://localhost:8766';

    // set initial mode (idle)
    setMode('idle', true);

    // defaults
    document.getElementById('numPoints').innerHTML = '0';
    document.getElementById('rangeLimit').innerHTML = '0–0 m';
    document.getElementById('lastScan').innerHTML = '0 s';
    document.getElementById('scanRate').innerHTML = '0 Hz';
    document.getElementById('leftSpeed').innerHTML = '0.00 m/s';
    document.getElementById('rightSpeed').innerHTML = '0.00 m/s';
    document.getElementById('robotMode').innerHTML = 'Idle';
    document.getElementById('posX').innerHTML = '0.00 m';
    document.getElementById('posY').innerHTML = '0.00 m';
    document.getElementById('theta').innerHTML = '0.00 °';
    document.getElementById('linearVel').innerHTML = '0.00 m/s';

    updateZoomDisplay();
    showView('map');
    document.getElementById('toggleGridBtn').textContent = 'Hide Grid';

    setTimeout(() => {
        console.log('[App] Auto-connecting...');
        connectWebSocket();
    }, 1000);

    document.getElementById('wsUrl').addEventListener('keypress', function(event) {
        if (event.key === 'Enter') connectWebSocket();
    });
});