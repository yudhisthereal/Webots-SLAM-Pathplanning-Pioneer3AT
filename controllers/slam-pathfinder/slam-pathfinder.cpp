// File:          slam-pathfinder.cpp
// Description:   Pioneer AT3 LiDAR controller with UDP broadcast
// For:           Webots R2025a with Pioneer 3-AT

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
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

int main(int argc, char **argv) {
    cout << "========================================" << endl;
    cout << "Pioneer 3-AT LiDAR Controller (UDP)" << endl;
    cout << "========================================" << endl;
    
    // Create Robot instance
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();
    cout << "[Robot] Time step: " << timeStep << " ms" << endl;
    
    // Initialize LiDAR
    Lidar *lidar = robot->getLidar("lidar");
    if (lidar == NULL) {
        cout << "[ERROR] LiDAR 'lidar' not found!" << endl;
        delete robot;
        return -1;
    }
    
    lidar->enable(timeStep);
    
    double minRange = lidar->getMinRange();
    double maxRange = lidar->getMaxRange();
    int horizontalResolution = lidar->getHorizontalResolution();
    double fov = lidar->getFov();
    
    cout << "[LiDAR] Model: " << lidar->getModel() << endl;
    cout << "[LiDAR] Resolution: " << horizontalResolution << " points" << endl;
    cout << "[LiDAR] Range: " << minRange << " - " << maxRange << " m" << endl;
    cout << "[LiDAR] FOV: " << fov * 180.0 / M_PI << " degrees" << endl;
    
    // Initialize motors
    Motor *backLeftMotor = robot->getMotor("back left wheel");
    Motor *backRightMotor = robot->getMotor("back right wheel");
    Motor *frontLeftMotor = robot->getMotor("front left wheel");
    Motor *frontRightMotor = robot->getMotor("front right wheel");
    
    if (backLeftMotor && backRightMotor && frontLeftMotor && frontRightMotor) {
        backLeftMotor->setPosition(INFINITY);
        backRightMotor->setPosition(INFINITY);
        frontLeftMotor->setPosition(INFINITY);
        frontRightMotor->setPosition(INFINITY);
        
        backLeftMotor->setVelocity(0.0);
        backRightMotor->setVelocity(0.0);
        frontLeftMotor->setVelocity(0.0);
        frontRightMotor->setVelocity(0.0);
        
        cout << "[Motors] All 4 wheels initialized" << endl;
    } else {
        cout << "[WARNING] Motors not found" << endl;
    }
    
    // Initialize UDP broadcaster
    UDPBroadcaster udp(8765);
    cout << "[UDP] Broadcasting on port 8765" << endl;
    cout << "========================================" << endl;
    
    // Control variables
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    bool autoNavigate = true;
    int iteration = 0;
    int downsample = 3;  // Send every 3rd point (120 points total)
    
    while (robot->step(timeStep) != -1) {
        const float *rangeImage = lidar->getRangeImage();
        
        if (rangeImage != NULL) {
            double angleStep = fov / horizontalResolution;
            double startAngle = -fov / 2.0;
            int numPoints = horizontalResolution / downsample;
            
            // Build JSON message
            stringstream json;
            json << fixed << setprecision(3);
            json << "{\"type\":\"lidar_scan\",";
            json << "\"timestamp\":" << robot->getTime() << ",";
            json << "\"num_points\":" << numPoints << ",";
            json << "\"min_range\":" << minRange << ",";
            json << "\"max_range\":" << maxRange << ",";
            json << "\"fov\":" << fov << ",";
            
            // Add ranges (downsampled)
            json << "\"ranges\":[";
            for (int i = 0; i < horizontalResolution; i += downsample) {
                float range = rangeImage[i];
                if (range < minRange || range > maxRange || isnan(range)) {
                    range = maxRange;
                }
                json << range;
                if (i + downsample < horizontalResolution) json << ",";
            }
            json << "],";
            
            // Add angles (downsampled)
            json << "\"angles\":[";
            for (int i = 0; i < horizontalResolution; i += downsample) {
                double angle = startAngle + i * angleStep;
                json << angle;
                if (i + downsample < horizontalResolution) json << ",";
            }
            json << "]}";
            
            // cout << "[LIDAR] readings: " << endl << json.str() << endl;
            
            // Send UDP packet every frame
            udp.send(json.str());
            
            // Simple obstacle avoidance
            if (autoNavigate && backLeftMotor && backRightMotor) {
                int centerIdx = (horizontalResolution / downsample) / 2;
                float minFrontRange = maxRange;
                
                for (int i = centerIdx - 10; i < centerIdx + 10 && i < numPoints; i++) {
                    int origIdx = i * downsample;
                    if (origIdx < horizontalResolution) {
                        float range = rangeImage[origIdx];
                        if (range > minRange && range < minFrontRange) {
                            minFrontRange = range;
                        }
                    }
                }
                
                // Obstacle avoidance logic
                if (minFrontRange < 0.5) {
                    leftSpeed = -0.3;
                    rightSpeed = 0.3;
                } else if (minFrontRange < 1.0) {
                    leftSpeed = 0.15;
                    rightSpeed = 0.1;
                } else {
                    leftSpeed = 0.4;
                    rightSpeed = 0.4;
                }
                
                backLeftMotor->setVelocity(leftSpeed);
                backRightMotor->setVelocity(rightSpeed);
                frontLeftMotor->setVelocity(leftSpeed);
                frontRightMotor->setVelocity(rightSpeed);
            }
        }
        
        // Send robot info every 2 seconds
        if (iteration % (int)(2000 / timeStep) == 0 && iteration > 0) {
            stringstream info;
            info << fixed << setprecision(3);
            info << "{\"type\":\"robot_info\",";
            info << "\"timestamp\":" << robot->getTime() << ",";
            info << "\"left_speed\":" << leftSpeed << ",";
            info << "\"right_speed\":" << rightSpeed << ",";
            info << "\"auto_navigate\":" << (autoNavigate ? "true" : "false") << "}";
            udp.send(info.str());
            
            cout << "[Robot] t=" << robot->getTime() << "s, L=" << leftSpeed << " R=" << rightSpeed << endl;
        }
        
        iteration++;
    }
    
    delete robot;
    return 0;
}