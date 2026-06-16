// File: odometry-test-dx.cpp
// Description: Pioneer 3-DX Wheel Odometry - prints position and orientation
// Uses 2 wheel encoders (differential drive with front caster)

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Lidar.hpp>
#include <webots/Keyboard.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace webots;
using namespace std;

class UDPBroadcaster {
private:
    int sock;
    struct sockaddr_in addr;
    
public:
    UDPBroadcaster(int port = 8765) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            cerr << "[UDP] Socket creation failed" << endl;
            return;
        }
        
        int broadcast = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
            cerr << "[UDP] Broadcast setting failed" << endl;
        }
        
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        
        cout << "[UDP] Broadcaster initialized on port " << port << endl;
    }
    
    void send(const string& data) {
        if (sock >= 0) {
            int result = sendto(sock, data.c_str(), data.length(), 0, 
                                (struct sockaddr*)&addr, sizeof(addr));
            if (result < 0) {
                cerr << "[UDP] Send failed" << endl;
            }
        }
    }
    
    ~UDPBroadcaster() {
        if (sock >= 0) close(sock);
    }
};

class Odometry {
private:
    // Pioneer 3-DX parameters (from WebOTS proto file)
    // From the PROTO: wheel radius = 0.0975m, wheelbase = 0.33m (distance between left and right wheels)
    double wheelRadius = 0.0975;        // meters (from Cylinder radius in BOUNDING_WHEEL)
    double wheelBase = 0.33;            // meters (track width: left wheel at +0.165, right at -0.165)
    
    // Previous values for left and right wheels
    double prevLeftPos = 0;
    double prevRightPos = 0;
    double prevTime = 0;
    
    double x = 0, y = 0, theta = 0;
    double linearVel = 0, angularVel = 0;
    bool firstUpdate = true;
    
public:
    void update(double leftPos, double rightPos, double currentTime) {
        if (firstUpdate) {
            prevLeftPos = leftPos;
            prevRightPos = rightPos;
            prevTime = currentTime;
            firstUpdate = false;
            return;
        }
        
        double dt = currentTime - prevTime;
        if (dt <= 0) return;
        
        // Distance = delta_radians * radius
        double leftDist = (leftPos - prevLeftPos) * wheelRadius;
        double rightDist = (rightPos - prevRightPos) * wheelRadius;
        
        // Calculate velocities
        linearVel = (leftDist + rightDist) / (2.0 * dt);
        angularVel = (rightDist - leftDist) / (wheelBase * dt);
        
        // Update pose using differential drive model
        double distance = (leftDist + rightDist) / 2.0;
        double deltaTheta = (rightDist - leftDist) / wheelBase;
        
        theta += deltaTheta;
        x += distance * cos(theta);
        y += distance * sin(theta);
        
        // Normalize theta to [-pi, pi]
        while (theta > M_PI) theta -= 2 * M_PI;
        while (theta < -M_PI) theta += 2 * M_PI;
        
        // Store for next iteration
        prevLeftPos = leftPos;
        prevRightPos = rightPos;
        prevTime = currentTime;
    }
    
    void printPose() {
        double thetaDeg = theta * 180.0 / M_PI;
        cout << fixed << setprecision(3);
        cout << "Position: (" << x << "m, " << y << "m) | ";
        cout << "Theta: " << thetaDeg << "°" << endl;
    }
    
    void printOdometryStats() {
        double thetaDeg = theta * 180.0 / M_PI;
        cout << fixed << setprecision(4);
        cout << "X: " << x << " m, Y: " << y << " m, Θ: " << thetaDeg << "°" << endl;
    }
    
    void printWheelOdometry(double leftPos, double rightPos) {
        cout << "Left wheel: " << fixed << setprecision(3) << leftPos << " rad | ";
        cout << "Right wheel: " << rightPos << " rad" << endl;
    }
    
    void reset() {
        x = y = theta = 0;
        linearVel = angularVel = 0;
        prevLeftPos = prevRightPos = 0;
        firstUpdate = true;
        cout << "[Odometry] Reset to origin" << endl;
    }
    
    double getX() { return x; }
    double getY() { return y; }
    double getTheta() { return theta; }
    double getLinearVel() { return linearVel; }
    double getAngularVel() { return angularVel; }
};

// Class to handle smooth velocity transitions using linear interpolation
class VelocitySmoother {
private:
    double targetLeftSpeed;
    double targetRightSpeed;
    double currentLeftSpeed;
    double currentRightSpeed;
    double smoothingFactor;  // How fast to interpolate (0.1 to 0.3 works well)
    
public:
    VelocitySmoother(double factor = 0.15) {
        targetLeftSpeed = 0.0;
        targetRightSpeed = 0.0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
        smoothingFactor = factor;
    }
    
    void setTarget(double left, double right) {
        targetLeftSpeed = left;
        targetRightSpeed = right;
    }
    
    void update(double dt) {
        // Linear interpolation (lerp) for smooth transitions
        // current = current + (target - current) * smoothingFactor
        currentLeftSpeed += (targetLeftSpeed - currentLeftSpeed) * smoothingFactor;
        currentRightSpeed += (targetRightSpeed - currentRightSpeed) * smoothingFactor;
        
        // Optional: Add deadzone to prevent micro-movements when close to zero
        if (fabs(currentLeftSpeed) < 0.01) currentLeftSpeed = 0.0;
        if (fabs(currentRightSpeed) < 0.01) currentRightSpeed = 0.0;
    }
    
    double getLeftSpeed() { return currentLeftSpeed; }
    double getRightSpeed() { return currentRightSpeed; }
    double getSmoothingFactor() { return smoothingFactor; }
    
    bool isMoving() {
        return (fabs(currentLeftSpeed) > 0.01 || fabs(currentRightSpeed) > 0.01);
    }
    
    void reset() {
        targetLeftSpeed = 0.0;
        targetRightSpeed = 0.0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
    }
    
    void setSmoothingFactor(double factor) {
        smoothingFactor = max(0.05, min(0.5, factor)); // Clamp between 0.05 and 0.5
    }
};

int main(int argc, char **argv) {
    cout << "========================================" << endl;
    cout << "Pioneer 3-DX Differential Drive Odometry" << endl;
    cout << "========================================" << endl;
    
    Robot *robot = new Robot();
    Keyboard *keyboard = new Keyboard();
    int timeStep = (int)robot->getBasicTimeStep();
    
    // Enable keyboard
    keyboard->enable(timeStep);
    
    // Initialize UDP broadcaster
    UDPBroadcaster udp(8765);
    cout << "[UDP] Broadcasting on port 8765" << endl;
    
    // Initialize LiDAR
    Lidar *lidar = robot->getLidar("lidar");
    double minRange = 0.0, maxRange = 0.0, fov = 0.0;
    int horizontalResolution = 0;
    double angleStep = 0.0;
    double startAngle = 0.0;
    double verticalFov = 0.0;
    int numberOfLayers = 0;
    bool isMultiLayer = false;
    bool *useLayer = nullptr;  // Array to track which layers to use (pointing down or level)
    
    if (lidar == NULL) {
        cout << "[WARNING] LiDAR 'lidar' not found!" << endl;
    } else {
        lidar->enable(timeStep);
        minRange = lidar->getMinRange();
        maxRange = lidar->getMaxRange();
        horizontalResolution = lidar->getHorizontalResolution();
        fov = lidar->getFov();
        verticalFov = lidar->getVerticalFov();
        numberOfLayers = lidar->getNumberOfLayers();
        angleStep = fov / horizontalResolution;
        startAngle = -fov / 2.0;
        
        cout << "[LiDAR] Model: " << lidar->getModel() << endl;
        cout << "[LiDAR] Resolution: " << horizontalResolution << " points" << endl;
        cout << "[LiDAR] Number of layers: " << numberOfLayers << endl;
        cout << "[LiDAR] Range: " << minRange << " - " << maxRange << " m" << endl;
        cout << "[LiDAR] Horizontal FOV: " << fov * 180.0 / M_PI << " degrees" << endl;
        
        if (numberOfLayers > 1) {
            isMultiLayer = true;
            cout << "[LiDAR] Vertical FOV: " << verticalFov * 180.0 / M_PI << " degrees" << endl;
            cout << "[LiDAR] This is a MULTI-LAYER (3D) LiDAR" << endl;
            
            // Print layer angles and determine which to use
            cout << "[LiDAR] Layer angles:" << endl;
            double verticalAngleStep = verticalFov / (numberOfLayers - 1);
            useLayer = new bool[numberOfLayers];
            
            for (int layer = 0; layer < numberOfLayers; layer++) {
                double layerAngle = -verticalFov/2.0 + layer * verticalAngleStep;
                useLayer[layer] = (layerAngle <= 0.0);  // Use layers pointing down or level
                
                if (useLayer[layer]) {
                    cout << "  Layer " << layer << ": " << layerAngle * 180.0 / M_PI << "° (USING)" << endl;
                } else {
                    cout << "  Layer " << layer << ": " << layerAngle * 180.0 / M_PI << "° (IGNORING)" << endl;
                }
            }
        } else {
            isMultiLayer = false;
            cout << "[LiDAR] This is a SINGLE-LAYER (2D) LiDAR" << endl;
            cout << "[LiDAR] Vertical beam divergence: " << verticalFov * 180.0 / M_PI << " degrees" << endl;
        }
    }
    
    // Initialize motors (required for movement)
    // Pioneer 3-DX has 2 drive wheels (left and right) + 1 passive caster
    Motor *leftMotor = robot->getMotor("left wheel");
    Motor *rightMotor = robot->getMotor("right wheel");
    
    if (leftMotor && rightMotor) {
        // Set motors to velocity control mode (position = INFINITY for continuous rotation)
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);
        
        cout << "[Motors] Left and right wheels ready" << endl;
    } else {
        cout << "[ERROR] Motors not found! Check motor names in the PROTO" << endl;
        cout << "Expected: 'left wheel' and 'right wheel'" << endl;
        delete robot;
        return -1;
    }
    
    // Initialize encoders for both drive wheels
    PositionSensor *leftEncoder = robot->getPositionSensor("left wheel sensor");
    PositionSensor *rightEncoder = robot->getPositionSensor("right wheel sensor");
    
    if (leftEncoder && rightEncoder) {
        leftEncoder->enable(timeStep);
        rightEncoder->enable(timeStep);
        cout << "[Encoders] Left and right wheel sensors enabled" << endl;
        cout << "[Params] Wheel radius: 0.0975m, Wheel base: 0.33m" << endl;
        cout << "[Params] Wheel circumference: " << (2.0 * M_PI * 0.0975) << "m" << endl;
        cout << "[Params] Max velocity: 12.3 rad/s (from PROTO)" << endl;
    } else {
        cout << "[ERROR] Encoders not found! Check sensor names in the PROTO" << endl;
        cout << "Expected: 'left wheel sensor' and 'right wheel sensor'" << endl;
        delete robot;
        return -1;
    }
    
    cout << "========================================" << endl;
    cout << "Controls:" << endl;
    cout << "  Arrow Up   : Move forward" << endl;
    cout << "  Arrow Down : Move backward" << endl;
    cout << "  Arrow Left : Turn left" << endl;
    cout << "  Arrow Right: Turn right" << endl;
    cout << "  Space      : Stop" << endl;
    cout << "  R          : Reset odometry" << endl;
    cout << "  P          : Print current pose" << endl;
    cout << "  L          : Print LiDAR data" << endl;
    cout << "  + / -      : Increase/Decrease smoothing (current: 0.15)" << endl;
    cout << "========================================" << endl;
    
    Odometry odom;
    VelocitySmoother smoother(0.15);  // Smoothing factor 0.15 for gradual transitions
    
    // Control variables
    double targetLeftSpeed = 0.0;
    double targetRightSpeed = 0.0;
    double maxSpeed = 6.0;      // Maximum wheel speed (rad/s) - below PROTO's 12.3 limit
    
    int iteration = 0;
    int printInterval = 500 / timeStep;  // Print every ~500ms
    int downsample = 3;  // Downsample LiDAR points for UDP
    double lastTime = 0.0;
    
    // LiDAR display variables
    int lidarScanCounter = 0;  // Counter to skip initial unstable scans
    
    while (robot->step(timeStep) != -1) {
        double currentTime = robot->getTime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        
        // Read encoders (values in radians)
        double leftPos = leftEncoder->getValue();
        double rightPos = rightEncoder->getValue();
        
        // Update odometry
        odom.update(leftPos, rightPos, currentTime);
        
        // Handle keyboard input
        int key = keyboard->getKey();
        
        while (key != -1) {
            switch (key) {
                case 'R':
                case 'r':
                    odom.reset();
                    smoother.reset();
                    break;
                    
                case 'P':
                case 'p':
                    cout << "[Pose] ";
                    odom.printPose();
                    break;
                    
                case 'L':
                case 'l':
                    if (lidar != NULL) {
                        cout << "[LiDAR] Scan at t=" << fixed << setprecision(2) << currentTime << "s" << endl;
                        
                        if (isMultiLayer) {
                            double verticalAngleStep = verticalFov / (numberOfLayers - 1);
                            for (int layer = 0; layer < numberOfLayers; layer++) {
                                const float *layerImage = lidar->getLayerRangeImage(layer);
                                if (layerImage != NULL) {
                                    double layerAngle = -verticalFov/2.0 + layer * verticalAngleStep;
                                    double layerAngleDeg = layerAngle * 180.0 / M_PI;
                                    float frontRange = layerImage[horizontalResolution/2];
                                    cout << "  Layer " << layer << " (" << layerAngleDeg << "°): Front=" << frontRange << "m" << endl;
                                }
                            }
                        } else {
                            const float *rangeImage = lidar->getRangeImage();
                            if (rangeImage != NULL) {
                                float frontRange = rangeImage[horizontalResolution/2];
                                float leftRange = rangeImage[horizontalResolution/4];
                                float rightRange = rangeImage[3*horizontalResolution/4];
                                cout << "  Front: " << frontRange << "m, Left: " << leftRange << "m, Right: " << rightRange << "m" << endl;
                            }
                        }
                    } else {
                        cout << "[LiDAR] No data available" << endl;
                    }
                    break;
                    
                case ' ':
                    targetLeftSpeed = 0.0;
                    targetRightSpeed = 0.0;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Stop] Smooth stopping initiated at t=" << fixed << setprecision(2) << currentTime << "s" << endl;
                    break;
                    
                case Keyboard::UP:
                    targetLeftSpeed = -maxSpeed;
                    targetRightSpeed = -maxSpeed;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Forward] Smooth acceleration to " << maxSpeed << " rad/s" << endl;
                    break;
                    
                case Keyboard::DOWN:
                    targetLeftSpeed = maxSpeed;
                    targetRightSpeed = maxSpeed;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Backward] Smooth acceleration to " << maxSpeed << " rad/s" << endl;
                    break;
                    
                case Keyboard::LEFT:
                    targetLeftSpeed = -maxSpeed;
                    targetRightSpeed = maxSpeed;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Turn Left] Smooth rotation start" << endl;
                    break;
                    
                case Keyboard::RIGHT:
                    targetLeftSpeed = maxSpeed;
                    targetRightSpeed = -maxSpeed;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Turn Right] Smooth rotation start" << endl;
                    break;
                    
                case '+':
                case '=':
                    {
                        double newFactor = smoother.getSmoothingFactor() + 0.02;
                        smoother.setSmoothingFactor(newFactor);
                        cout << "[Smoothing] Increased to " << fixed << setprecision(2) << smoother.getSmoothingFactor() << endl;
                    }
                    break;
                    
                case '-':
                case '_':
                    {
                        double newFactor = smoother.getSmoothingFactor() - 0.02;
                        smoother.setSmoothingFactor(newFactor);
                        cout << "[Smoothing] Decreased to " << fixed << setprecision(2) << smoother.getSmoothingFactor() << endl;
                    }
                    break;
            }
            key = keyboard->getKey();
        }
        
        // Update smooth velocity transition
        if (dt > 0 && dt < 0.1) {  // Only update with reasonable dt
            smoother.update(dt);
        } else {
            smoother.update(0.032);  // Use default timestep if dt is invalid
        }
        
        // Apply smooth motor commands
        leftMotor->setVelocity(smoother.getLeftSpeed());
        rightMotor->setVelocity(smoother.getRightSpeed());
        
        // Send LiDAR data and odometry via UDP
        if (lidar != NULL) {
            lidarScanCounter++;
            
            // Skip first 10 scans as they may be inaccurate
            if (lidarScanCounter > 10) {
                vector<float> allRanges;
                vector<double> allAngles;
                
                if (isMultiLayer) {
                    for (int layer = 0; layer < numberOfLayers; layer++) {
                        if (!useLayer[layer]) {
                            continue;  // Skip layers that point upward
                        }
                        
                        const float *layerImage = lidar->getLayerRangeImage(layer);
                        if (layerImage != NULL) {
                            for (int i = 0; i < horizontalResolution; i += downsample) {
                                float range = layerImage[i];
                                if (range < minRange || range > maxRange || isnan(range)) {
                                    range = maxRange;
                                }
                                allRanges.push_back(range);
                                allAngles.push_back(startAngle + i * angleStep);
                            }
                        }
                    }
                } else {
                    // Single-layer (2D) LiDAR: use full range image directly
                    const float *rangeImage = lidar->getRangeImage();
                    if (rangeImage != NULL) {
                        for (int i = 0; i < horizontalResolution; i += downsample) {
                            float range = rangeImage[i];
                            if (range < minRange || range > maxRange || isnan(range)) {
                                range = maxRange;
                            }
                            allRanges.push_back(range);
                            allAngles.push_back(startAngle + i * angleStep);
                        }
                    }
                }
                
                // Only send if we have data
                if (!allRanges.empty()) {
                    // Get odometry data
                    double robotX = odom.getX();
                    double robotY = odom.getY();
                    double robotTheta = odom.getTheta();
                    double linearVel = odom.getLinearVel();
                    double angularVel = odom.getAngularVel();
                    
                    // Build JSON message with filtered LiDAR data and odometry
                    stringstream json;
                    json << fixed << setprecision(3);
                    json << "{\"type\":\"lidar_scan\",";
                    json << "\"timestamp\":" << currentTime << ",";
                    json << "\"num_points\":" << allRanges.size() << ",";
                    json << "\"min_range\":" << minRange << ",";
                    json << "\"max_range\":" << maxRange << ",";
                    json << "\"fov\":" << fov << ",";
                    json << "\"robot_x\":" << robotX << ",";
                    json << "\"robot_y\":" << robotY << ",";
                    json << "\"robot_theta\":" << -robotTheta << ",";
                    json << "\"left_speed\":" << smoother.getLeftSpeed() << ",";
                    json << "\"right_speed\":" << smoother.getRightSpeed() << ",";
                    json << "\"linear_vel\":" << linearVel << ",";
                    json << "\"angular_vel\":" << angularVel << ",";
                    json << "\"auto_navigate\":false,";
                    
                    // Add ranges
                    json << "\"ranges\":[";
                    for (size_t i = 0; i < allRanges.size(); i++) {
                        json << allRanges[i];
                        if (i + 1 < allRanges.size()) json << ",";
                    }
                    json << "],";
                    
                    // Add angles
                    json << "\"angles\":[";
                    for (size_t i = 0; i < allAngles.size(); i++) {
                        json << allAngles[i];
                        if (i + 1 < allAngles.size()) json << ",";
                    }
                    json << "]}";
                    
                    // Send UDP packet
                    udp.send(json.str());
                }
            }
        }
        
        // Send robot info every 2 seconds (exactly like P3-AT)
        if (iteration % (int)(2000 / timeStep) == 0 && iteration > 0) {
            double robotX = odom.getX();
            double robotY = odom.getY();
            double robotTheta = odom.getTheta();
            double linearVel = odom.getLinearVel();
            double angularVel = odom.getAngularVel();
            
            stringstream info;
            info << fixed << setprecision(3);
            info << "{\"type\":\"robot_info\",";
            info << "\"timestamp\":" << currentTime << ",";
            info << "\"left_speed\":" << smoother.getLeftSpeed() << ",";
            info << "\"right_speed\":" << smoother.getRightSpeed() << ",";
            info << "\"x\":" << robotX << ",";
            info << "\"y\":" << robotY << ",";
            info << "\"theta\":" << robotTheta << ",";
            info << "\"linear_vel\":" << linearVel << ",";
            info << "\"angular_vel\":" << angularVel << "}";
            udp.send(info.str());
            
            cout << "[Robot] t=" << currentTime << "s, pos=(" << robotX << "," << robotY << "," << robotTheta << "), vel=" << linearVel << "m/s" << endl;
        }
        
        // Print pose at regular intervals
        if (iteration % printInterval == 0) {
            if (smoother.isMoving()) {
                cout << "t=" << fixed << setprecision(2) << currentTime << "s | ";
                odom.printPose();
                // Show current speeds when moving (debug info)
                cout << "    [vL: " << setw(6) << smoother.getLeftSpeed() 
                     << ", vR: " << setw(6) << smoother.getRightSpeed() << " rad/s]" << endl;
            } else {
                // Print occasional pose even when stopped
                if (iteration % (printInterval * 5) == 0) {
                    cout << "t=" << fixed << setprecision(2) << currentTime << "s [idle] | ";
                    odom.printPose();
                }
            }
        }
        
        iteration++;
    }
    
    delete keyboard;
    delete robot;
    if (useLayer != nullptr) {
        delete[] useLayer;
    }
    return 0;
}