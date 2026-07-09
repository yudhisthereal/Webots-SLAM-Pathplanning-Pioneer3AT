// File: odometry-test-p3dx.cpp
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
#include <fcntl.h>
#include <vector>
#include <sstream>

using namespace webots;
using namespace std;

const int STARTUP_STEPS = 100;  // number of initial steps to wait
const double ROBOT_WIDTH = 0.2; // meters, width of P3DX

class UDPBroadcaster
{
private:
    int sock;
    struct sockaddr_in addr;

public:
    UDPBroadcaster(int port = 8765)
    {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            cerr << "[UDP] Socket creation failed" << endl;
            return;
        }

        int broadcast = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
        {
            cerr << "[UDP] Broadcast setting failed" << endl;
        }

        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");

        cout << "[UDP] Broadcaster initialized on port " << port << endl;
    }

    void send(const string &data)
    {
        if (sock >= 0)
        {
            int result = sendto(sock, data.c_str(), data.length(), 0,
                                (struct sockaddr *)&addr, sizeof(addr));
            if (result < 0)
            {
                cerr << "[UDP] Send failed" << endl;
            }
        }
    }

    ~UDPBroadcaster()
    {
        if (sock >= 0)
            close(sock);
    }
};

class Odometry
{
private:
    // Pioneer 3-DX parameters (from WebOTS proto file)
    double wheelRadius = 0.0975; // meters
    double wheelBase = 0.33;     // meters (track width)

    // Previous values for left and right wheels
    double prevLeftPos = 0;
    double prevRightPos = 0;
    double prevTime = 0;

    double x = 0, y = 0, theta = 0;
    double linearVel = 0, angularVel = 0;
    bool firstUpdate = true;

public:
    void update(double leftPos, double rightPos, double currentTime)
    {
        if (firstUpdate)
        {
            prevLeftPos = leftPos;
            prevRightPos = rightPos;
            prevTime = currentTime;
            firstUpdate = false;
            return;
        }

        double dt = currentTime - prevTime;
        if (dt <= 0)
            return;

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
        while (theta > M_PI)
            theta -= 2 * M_PI;
        while (theta < -M_PI)
            theta += 2 * M_PI;

        // Store for next iteration
        prevLeftPos = leftPos;
        prevRightPos = rightPos;
        prevTime = currentTime;
    }

    void printPose()
    {
        double thetaDeg = theta * 180.0 / M_PI;
        cout << fixed << setprecision(3);
        cout << "Position: (" << x << "m, " << y << "m) | ";
        cout << "Theta: " << thetaDeg << "°" << endl;
    }

    void printOdometryStats()
    {
        double thetaDeg = theta * 180.0 / M_PI;
        cout << fixed << setprecision(4);
        cout << "X: " << x << " m, Y: " << y << " m, Θ: " << thetaDeg << "°" << endl;
    }

    void reset()
    {
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

class VelocitySmoother
{
private:
    double targetLeftSpeed;
    double targetRightSpeed;
    double currentLeftSpeed;
    double currentRightSpeed;
    double smoothingFactor;

public:
    VelocitySmoother(double factor = 0.15)
    {
        targetLeftSpeed = 0.0;
        targetRightSpeed = 0.0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
        smoothingFactor = factor;
    }

    void setTarget(double left, double right)
    {
        targetLeftSpeed = left;
        targetRightSpeed = right;
    }

    void update(double dt)
    {
        if (dt > 0.1)
            dt = 0.032; // Cap dt for stability
        currentLeftSpeed += (targetLeftSpeed - currentLeftSpeed) * smoothingFactor;
        currentRightSpeed += (targetRightSpeed - currentRightSpeed) * smoothingFactor;

        if (fabs(currentLeftSpeed) < 0.005)
            currentLeftSpeed = 0.0;
        if (fabs(currentRightSpeed) < 0.005)
            currentRightSpeed = 0.0;
    }

    double getLeftSpeed() { return currentLeftSpeed; }
    double getRightSpeed() { return currentRightSpeed; }
    double getSmoothingFactor() { return smoothingFactor; }

    bool isMoving()
    {
        return (fabs(currentLeftSpeed) > 0.01 || fabs(currentRightSpeed) > 0.01);
    }

    void reset()
    {
        targetLeftSpeed = 0.0;
        targetRightSpeed = 0.0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
    }

    void setSmoothingFactor(double factor)
    {
        smoothingFactor = max(0.05, min(0.5, factor));
    }
};

int main(int argc, char **argv)
{
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

    // UDP receiver for commands from bridge
    int cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (cmd_sock < 0)
    {
        cerr << "[UDP] Command socket creation failed" << endl;
    }
    else
    {
        struct sockaddr_in cmd_addr;
        memset(&cmd_addr, 0, sizeof(cmd_addr));
        cmd_addr.sin_family = AF_INET;
        cmd_addr.sin_port = htons(8767);
        cmd_addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(cmd_sock, (struct sockaddr *)&cmd_addr, sizeof(cmd_addr)) < 0)
        {
            cerr << "[UDP] Command socket bind failed" << endl;
            close(cmd_sock);
            cmd_sock = -1;
        }
        else
        {
            int flags = fcntl(cmd_sock, F_GETFL, 0);
            fcntl(cmd_sock, F_SETFL, flags | O_NONBLOCK);
            cout << "[UDP] Command listener on port 8767" << endl;
        }
    }

    int path_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (path_sock < 0)
    {
        cerr << "[UDP] Path socket creation failed" << endl;
    }
    else
    {
        struct sockaddr_in path_addr;
        memset(&path_addr, 0, sizeof(path_addr));
        path_addr.sin_family = AF_INET;
        path_addr.sin_port = htons(8768);
        path_addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(path_sock, (struct sockaddr *)&path_addr, sizeof(path_addr)) < 0)
        {
            cerr << "[UDP] Path socket bind failed" << endl;
            close(path_sock);
            path_sock = -1;
        }
        else
        {
            int flags = fcntl(path_sock, F_GETFL, 0);
            fcntl(path_sock, F_SETFL, flags | O_NONBLOCK);
            cout << "[UDP] Path listener on port 8768" << endl;
        }
    }

    // Initialize LiDAR
    Lidar *lidar = robot->getLidar("lidar");
    double minRange = 0.0, maxRange = 0.0, fov = 0.0;
    int horizontalResolution = 0;
    double angleStep = 0.0;
    double startAngle = 0.0;
    double verticalFov = 0.0;
    int numberOfLayers = 0;
    bool isMultiLayer = false;
    bool *useLayer = nullptr;

    if (lidar == NULL)
    {
        cout << "[WARNING] LiDAR 'lidar' not found!" << endl;
    }
    else
    {
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

        if (numberOfLayers > 1)
        {
            isMultiLayer = true;
            cout << "[LiDAR] Vertical FOV: " << verticalFov * 180.0 / M_PI << " degrees" << endl;
            cout << "[LiDAR] This is a MULTI-LAYER (3D) LiDAR" << endl;

            double verticalAngleStep = verticalFov / (numberOfLayers - 1);
            useLayer = new bool[numberOfLayers];

            for (int layer = 0; layer < numberOfLayers; layer++)
            {
                double layerAngle = -verticalFov / 2.0 + layer * verticalAngleStep;
                useLayer[layer] = (layerAngle <= 0.0);

                if (useLayer[layer])
                {
                    cout << "  Layer " << layer << ": " << layerAngle * 180.0 / M_PI << "° (USING)" << endl;
                }
                else
                {
                    cout << "  Layer " << layer << ": " << layerAngle * 180.0 / M_PI << "° (IGNORING)" << endl;
                }
            }
        }
        else
        {
            isMultiLayer = false;
            cout << "[LiDAR] This is a SINGLE-LAYER (2D) LiDAR" << endl;
        }
    }

    // Initialize motors
    Motor *leftMotor = robot->getMotor("left wheel");
    Motor *rightMotor = robot->getMotor("right wheel");

    if (leftMotor && rightMotor)
    {
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);
        cout << "[Motors] Left and right wheels ready" << endl;
    }
    else
    {
        cout << "[ERROR] Motors not found!" << endl;
        delete robot;
        return -1;
    }

    // Initialize encoders
    PositionSensor *leftEncoder = robot->getPositionSensor("left wheel sensor");
    PositionSensor *rightEncoder = robot->getPositionSensor("right wheel sensor");

    if (leftEncoder && rightEncoder)
    {
        leftEncoder->enable(timeStep);
        rightEncoder->enable(timeStep);
        cout << "[Encoders] Left and right wheel sensors enabled" << endl;
        cout << "[Params] Wheel radius: 0.0975m, Wheel base: 0.33m" << endl;
    }
    else
    {
        cout << "[ERROR] Encoders not found!" << endl;
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
    cout << "  + / -      : Increase/Decrease smoothing" << endl;
    cout << "========================================" << endl;

    Odometry odom;
    VelocitySmoother smoother(0.15);

    bool autoMode = false;
    std::vector<std::pair<double, double>> pathPoints;
    size_t pathIndex = 0;
    bool pathActive = false;

    double targetLeftSpeed = 0.0;
    double targetRightSpeed = 0.0;
    double maxSpeed = 4.0;

    int iteration = 0;
    int printInterval = 500 / timeStep;
    int downsample = 3;
    double lastTime = 0.0;
    int lidarScanCounter = 0;

    // Command buffer
    char cmdBuffer[65536];

    int stepCounter = 0;
    bool processingStarted = false;

    while (robot->step(timeStep) != -1)
    {
        double obstacleDistThres = autoMode ? 0.3 : 0.15;
        stepCounter++; // increment each simulation step

        // --- If we haven't reached startup steps, do nothing---
        if (stepCounter <= STARTUP_STEPS)
        {
            continue; // jump to next iteration
        }
        else if (!processingStarted)
        {
            processingStarted = true;
            cout << "PROCESSING STARTED" << endl;
        }

        double currentTime = robot->getTime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // Read encoders
        double leftPos = leftEncoder->getValue();
        double rightPos = rightEncoder->getValue();

        // Update odometry
        odom.update(leftPos, rightPos, currentTime);

        // ========== OBSTACLE DETECTION ==========
        bool obstacleAhead = false;
        if (lidar != NULL)
        {
            if (isMultiLayer)
            {
                for (int layer = 0; layer < numberOfLayers; layer++)
                {
                    if (!useLayer[layer])
                        continue;
                    const float *layerImage = lidar->getLayerRangeImage(layer);
                    if (layerImage != NULL)
                    {
                        for (int i = 0; i < horizontalResolution; i += 1) // full resolution
                        {
                            float range = layerImage[i];
                            if (range < minRange || range > maxRange || isnan(range))
                                continue;
                            double angle = startAngle + i * angleStep;
                            double x = range * cos(angle);
                            double y = range * sin(angle);
                            if (x > 0 && x < obstacleDistThres && fabs(y) < ROBOT_WIDTH / 2.0)
                            {
                                obstacleAhead = true;
                                break;
                            }
                        }
                    }
                    if (obstacleAhead)
                        break;
                }
            }
            else
            {
                const float *rangeImage = lidar->getRangeImage();
                if (rangeImage != NULL)
                {
                    for (int i = 0; i < horizontalResolution; i += 1)
                    {
                        float range = rangeImage[i];
                        if (range < minRange || range > maxRange || isnan(range))
                            continue;
                        double angle = startAngle + i * angleStep;
                        double x = range * cos(angle);
                        double y = range * sin(angle);
                        if (x > 0 && x < obstacleDistThres && fabs(y) < ROBOT_WIDTH / 2.0)
                        {
                            obstacleAhead = true;
                            break;
                        }
                    }
                }
            }
        }

        // Handle keyboard input
        int key = keyboard->getKey();

        while (key != -1)
        {
            switch (key)
            {
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
                if (lidar != NULL)
                {
                    cout << "[LiDAR] Scan at t=" << fixed << setprecision(2) << currentTime << "s" << endl;
                    if (isMultiLayer)
                    {
                        for (int layer = 0; layer < numberOfLayers; layer++)
                        {
                            const float *layerImage = lidar->getLayerRangeImage(layer);
                            if (layerImage != NULL)
                            {
                                float frontRange = layerImage[horizontalResolution / 2];
                                cout << "  Layer " << layer << ": Front=" << frontRange << "m" << endl;
                            }
                        }
                    }
                    else
                    {
                        const float *rangeImage = lidar->getRangeImage();
                        if (rangeImage != NULL)
                        {
                            float frontRange = rangeImage[horizontalResolution / 2];
                            cout << "  Front: " << frontRange << "m" << endl;
                        }
                    }
                }
                break;

            case ' ':
                if (!autoMode)
                {
                    targetLeftSpeed = 0.0;
                    targetRightSpeed = 0.0;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Stop] Smooth stopping" << endl;
                }
                break;

            case Keyboard::UP:
                if (!autoMode)
                {
                    targetLeftSpeed = maxSpeed;
                    targetRightSpeed = maxSpeed;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Forward] Speed: " << maxSpeed << " rad/s" << endl;
                }
                break;

            case Keyboard::DOWN:
                if (!autoMode)
                {
                    targetLeftSpeed = -maxSpeed;
                    targetRightSpeed = -maxSpeed;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Backward] Speed: " << maxSpeed << " rad/s" << endl;
                }
                break;

            case Keyboard::LEFT:
                if (!autoMode)
                {
                    targetLeftSpeed = -maxSpeed;
                    targetRightSpeed = maxSpeed;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Turn Left]" << endl;
                }
                break;

            case Keyboard::RIGHT:
                if (!autoMode)
                {
                    targetLeftSpeed = maxSpeed;
                    targetRightSpeed = -maxSpeed;
                    smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                    cout << "[Turn Right]" << endl;
                }
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
        if (dt > 0 && dt < 0.1)
        {
            smoother.update(dt);
        }
        else
        {
            smoother.update(0.032);
        }

        // Poll for incoming command messages from bridge
        if (cmd_sock >= 0)
        {
            struct sockaddr_in src;
            socklen_t srclen = sizeof(src);
            ssize_t r = recvfrom(cmd_sock, cmdBuffer, sizeof(cmdBuffer) - 1, 0, (struct sockaddr *)&src, &srclen);
            if (r > 0)
            {
                cmdBuffer[r] = '\0';
                std::string msg(cmdBuffer);

                while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r'))
                {
                    msg.pop_back();
                }

                cout << "[DEBUG] Raw command received: '" << msg << "'" << endl;
                cout << "[DEBUG] autoMode: " << (autoMode ? "true" : "false") << endl;

                if (msg.rfind("CMD:", 0) == 0)
                {
                    std::string cmd = msg.substr(4);
                    cout << "[DEBUG] Parsed CMD: '" << cmd << "'" << endl;

                    if (!autoMode)
                    {
                        cout << "[Command] Executing: " << cmd << endl;
                        if (cmd == "forward")
                        {
                            targetLeftSpeed = maxSpeed;
                            targetRightSpeed = maxSpeed;
                            smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                            cout << "[DEBUG] Set speeds to forward: " << targetLeftSpeed << ", " << targetRightSpeed << endl;
                        }
                        else if (cmd == "backward")
                        {
                            targetLeftSpeed = -maxSpeed;
                            targetRightSpeed = -maxSpeed;
                            smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                            cout << "[DEBUG] Set speeds to backward: " << targetLeftSpeed << ", " << targetRightSpeed << endl;
                        }
                        else if (cmd == "left")
                        {
                            targetLeftSpeed = -maxSpeed;
                            targetRightSpeed = maxSpeed;
                            smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                            cout << "[DEBUG] Set speeds to left: " << targetLeftSpeed << ", " << targetRightSpeed << endl;
                        }
                        else if (cmd == "right")
                        {
                            targetLeftSpeed = maxSpeed;
                            targetRightSpeed = -maxSpeed;
                            smoother.setTarget(targetLeftSpeed, targetRightSpeed);
                            cout << "[DEBUG] Set speeds to right: " << targetLeftSpeed << ", " << targetRightSpeed << endl;
                        }
                        else if (cmd == "stop")
                        {
                            targetLeftSpeed = 0.0;
                            targetRightSpeed = 0.0;
                            smoother.setTarget(0.0, 0.0);
                            cout << "[DEBUG] Set speeds to stop" << endl;
                        }
                        else if (cmd == "auto")
                        {
                            autoMode = !autoMode;
                            cout << "[DEBUG] Toggled auto mode to: " << (autoMode ? "true" : "false") << endl;
                            if (autoMode)
                            {
                                cout << "[Auto] Autonomous mode ENABLED" << endl;
                                pathIndex = 0;
                                pathActive = !pathPoints.empty();
                            }
                            else
                            {
                                cout << "[Auto] Autonomous mode DISABLED" << endl;
                                pathPoints.clear();
                                pathActive = false;
                                targetLeftSpeed = 0.0;
                                targetRightSpeed = 0.0;
                                smoother.setTarget(0.0, 0.0);
                            }
                        }
                        else
                        {
                            cout << "[DEBUG] Unknown command: " << cmd << endl;
                        }
                    }
                    else
                    {
                        cout << "[DEBUG] Command ignored - autoMode is enabled" << endl;
                    }
                }
                else if (msg.rfind("AUTO:", 0) == 0)
                {
                    std::string v = msg.substr(5);
                    bool newAuto = (v == "1");
                    cout << "[DEBUG] AUTO: setting autoMode to " << (newAuto ? "true" : "false") << endl;
                    if (newAuto != autoMode)
                    {
                        autoMode = newAuto;
                        if (autoMode)
                        {
                            cout << "[Auto] Autonomous mode ENABLED by bridge" << endl;
                            pathIndex = 0;
                            pathActive = !pathPoints.empty();
                        }
                        else
                        {
                            cout << "[Auto] Autonomous mode DISABLED by bridge" << endl;
                            pathPoints.clear();
                            pathActive = false;
                            targetLeftSpeed = 0.0;
                            targetRightSpeed = 0.0;
                            smoother.setTarget(0.0, 0.0);
                        }
                    }
                }
                else if (msg.rfind("SPEED:", 0) == 0)
                {
                    std::string speedStr = msg.substr(6);
                    try
                    {
                        double newSpeed = std::stod(speedStr);
                        newSpeed = std::max(0.1, std::min(10.0, newSpeed));
                        maxSpeed = newSpeed;
                        cout << "[Speed] Max speed set to " << fixed << setprecision(2) << maxSpeed << " rad/s" << endl;
                    }
                    catch (...)
                    {
                        cerr << "[Speed] Invalid value: " << speedStr << endl;
                    }
                }
                else
                {
                    cout << "[DEBUG] Unknown message type: " << msg.substr(0, msg.find(':')) << endl;
                }
            }
        }

        if (path_sock >= 0)
        {
            struct sockaddr_in src;
            socklen_t srclen = sizeof(src);
            ssize_t r = recvfrom(path_sock, cmdBuffer, sizeof(cmdBuffer) - 1, 0, (struct sockaddr *)&src, &srclen);
            if (r > 0)
            {
                cmdBuffer[r] = '\0';
                std::string msg(cmdBuffer);

                while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r'))
                {
                    msg.pop_back();
                }

                cout << "[Path] Received: " << msg.substr(0, min((size_t)100, msg.length())) << endl;

                if (msg.rfind("PATH:", 0) == 0)
                {
                    std::string body = msg.substr(5);
                    pathPoints.clear();
                    pathIndex = 0;
                    pathActive = false;

                    std::stringstream ss(body);
                    std::string pair;
                    while (std::getline(ss, pair, ';'))
                    {
                        size_t comma = pair.find(',');
                        if (comma != std::string::npos)
                        {
                            try
                            {
                                double px = std::stod(pair.substr(0, comma));
                                double py = std::stod(pair.substr(comma + 1));
                                pathPoints.emplace_back(px, py);
                            }
                            catch (...)
                            {
                                // ignore
                            }
                        }
                    }

                    if (!pathPoints.empty())
                    {
                        pathActive = true;
                        cout << "[Path] Received " << pathPoints.size() << " points on dedicated port" << endl;
                    }
                }
            }
        }

        // ========== OBSTACLE STOP (active in ALL modes) ==========
        if (obstacleAhead)
        {
            static int obstacleMsgCounter = 0;
            if (++obstacleMsgCounter % 10 == 0)
            {
                cout << "[Obstacle] Obstacle detected ahead! Stopping." << endl;
            }
            targetLeftSpeed = 0.0;
            targetRightSpeed = 0.0;
            smoother.setTarget(0.0, 0.0);
        }
        else if (autoMode && pathActive && !pathPoints.empty() && pathIndex < pathPoints.size())
        {
            double tx = pathPoints[pathIndex].first;
            double ty = pathPoints[pathIndex].second;
            double dxp = tx - odom.getX();
            double dyp = ty - odom.getY();
            double dist = sqrt(dxp * dxp + dyp * dyp);
            double desired_heading = atan2(dyp, dxp);
            double diff = desired_heading - odom.getTheta();
            while (diff > M_PI)
                diff -= 2 * M_PI;
            while (diff < -M_PI)
                diff += 2 * M_PI;

            double turnThresh = 0.15;
            double turnSpeed = maxSpeed * 0.35;

            bool isLastWaypoint = (pathIndex == pathPoints.size() - 1);
            double stopThreshold = 0.12;

            if (dist < stopThreshold)
            {
                if (isLastWaypoint)
                {
                    cout << "[Path] Final goal reached! Stopping." << endl;
                    pathActive = false;
                    targetLeftSpeed = 0.0;
                    targetRightSpeed = 0.0;
                    smoother.setTarget(0.0, 0.0);
                    pathPoints.clear();
                }
                else
                {
                    pathIndex++;
                    cout << "[Path] Reached waypoint " << pathIndex << "/" << pathPoints.size() << endl;
                }
            }
            else if (fabs(diff) > turnThresh)
            {
                if (diff > 0)
                {
                    targetLeftSpeed = -turnSpeed;
                    targetRightSpeed = turnSpeed;
                }
                else
                {
                    targetLeftSpeed = turnSpeed;
                    targetRightSpeed = -turnSpeed;
                }
                smoother.setTarget(targetLeftSpeed, targetRightSpeed);
            }
            else
            {
                double speed = maxSpeed * 0.9;
                if (isLastWaypoint)
                {
                    double brakingZone = 1.2;
                    if (dist < brakingZone)
                    {
                        speed = speed * (dist / brakingZone);
                        if (speed < 0.08)
                            speed = 0.08;
                    }
                }
                targetLeftSpeed = speed;
                targetRightSpeed = speed;
                smoother.setTarget(targetLeftSpeed, targetRightSpeed);
            }
        }

        // Apply motor commands
        leftMotor->setVelocity(smoother.getLeftSpeed());
        rightMotor->setVelocity(smoother.getRightSpeed());

        // Send LiDAR data and odometry via UDP
        if (lidar != NULL)
        {
            lidarScanCounter++;

            if (lidarScanCounter > 10)
            {
                vector<float> allRanges;
                vector<double> allAngles;

                if (isMultiLayer)
                {
                    for (int layer = 0; layer < numberOfLayers; layer++)
                    {
                        if (!useLayer[layer])
                            continue;

                        const float *layerImage = lidar->getLayerRangeImage(layer);
                        if (layerImage != NULL)
                        {
                            for (int i = 0; i < horizontalResolution; i += downsample)
                            {
                                float range = layerImage[i];
                                if (range < minRange || range > maxRange || isnan(range))
                                {
                                    range = maxRange;
                                }
                                allRanges.push_back(range);
                                allAngles.push_back(startAngle + i * angleStep);
                            }
                        }
                    }
                }
                else
                {
                    const float *rangeImage = lidar->getRangeImage();
                    if (rangeImage != NULL)
                    {
                        for (int i = 0; i < horizontalResolution; i += downsample)
                        {
                            float range = rangeImage[i];
                            if (range < minRange || range > maxRange || isnan(range))
                            {
                                range = maxRange;
                            }
                            allRanges.push_back(range);
                            allAngles.push_back(startAngle + i * angleStep);
                        }
                    }
                }

                if (!allRanges.empty())
                {
                    double robotX = odom.getX();
                    double robotY = odom.getY();
                    double robotTheta = odom.getTheta();
                    double linearVel = odom.getLinearVel();
                    double angularVel = odom.getAngularVel();

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
                    json << "\"auto_navigate\":" << (autoMode ? "true" : "false") << ",";

                    json << "\"ranges\":[";
                    for (size_t i = 0; i < allRanges.size(); i++)
                    {
                        json << allRanges[i];
                        if (i + 1 < allRanges.size())
                            json << ",";
                    }
                    json << "],";

                    json << "\"angles\":[";
                    for (size_t i = 0; i < allAngles.size(); i++)
                    {
                        json << allAngles[i];
                        if (i + 1 < allAngles.size())
                            json << ",";
                    }
                    json << "]}";

                    udp.send(json.str());
                }
            }
        }

        // Print pose at regular intervals
        if (iteration % printInterval == 0)
        {
            if (smoother.isMoving())
            {
                odom.printPose();
            }
        }

        iteration++;
    }

    delete keyboard;
    delete robot;
    if (useLayer != nullptr)
    {
        delete[] useLayer;
    }
    if (cmd_sock >= 0)
        close(cmd_sock);

    if (path_sock >= 0)
        close(path_sock);
    return 0;
}