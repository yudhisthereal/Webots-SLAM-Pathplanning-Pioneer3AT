// File: odometry-test-at.cpp
// Description: Pioneer 3-AT Wheel Odometry - prints position and orientation
// Rear wheels powered, front wheels passive (free-rolling)

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace webots;
using namespace std;

class Odometry {
private:
    // Pioneer 3-AT parameters (from WebOTS proto file)
    double wheelRadius = 0.11;           // meters
    double wheelBase = 0.394;            // meters (track width from wheel positions)
    
    // Previous values for rear wheels (powered) and front wheels (passive but tracked for odometry)
    double prevRearLeftPos = 0;
    double prevRearRightPos = 0;
    double prevFrontLeftPos = 0;
    double prevFrontRightPos = 0;
    double prevTime = 0;
    
    double x = 0, y = 0, theta = 0;
    double linearVel = 0, angularVel = 0;
    bool firstUpdate = true;
    
public:
    void update(double rearLeftPos, double rearRightPos, 
                double frontLeftPos, double frontRightPos, 
                double currentTime) {
        if (firstUpdate) {
            prevRearLeftPos = rearLeftPos;
            prevRearRightPos = rearRightPos;
            prevFrontLeftPos = frontLeftPos;
            prevFrontRightPos = frontRightPos;
            prevTime = currentTime;
            firstUpdate = false;
            return;
        }
        
        double dt = currentTime - prevTime;
        if (dt <= 0) return;
        
        // Calculate distances for ALL wheels (including passive front ones)
        double rearLeftDist = (rearLeftPos - prevRearLeftPos) * wheelRadius;
        double rearRightDist = (rearRightPos - prevRearRightPos) * wheelRadius;
        double frontLeftDist = (frontLeftPos - prevFrontLeftPos) * wheelRadius;
        double frontRightDist = (frontRightPos - prevFrontRightPos) * wheelRadius;
        
        // Average left wheels and right wheels for better odometry accuracy
        // This accounts for any minor slipping or differences between front/rear
        double leftDist = (rearLeftDist + frontLeftDist) / 2.0;
        double rightDist = (rearRightDist + frontRightDist) / 2.0;
        
        // Calculate velocities (using rear wheels as the primary source)
        linearVel = (rearLeftDist + rearRightDist) / (2.0 * dt);
        angularVel = (rearRightDist - rearLeftDist) / (wheelBase * dt);
        
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
        prevRearLeftPos = rearLeftPos;
        prevRearRightPos = rearRightPos;
        prevFrontLeftPos = frontLeftPos;
        prevFrontRightPos = frontRightPos;
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
    
    void printWheelOdometry(double rearLeft, double rearRight, double frontLeft, double frontRight) {
        cout << "Rear Left: " << fixed << setprecision(3) << rearLeft << " rad | ";
        cout << "Rear Right: " << rearRight << " rad | ";
        cout << "Front Left: " << frontLeft << " rad | ";
        cout << "Front Right: " << frontRight << " rad" << endl;
    }
    
    void reset() {
        x = y = theta = 0;
        linearVel = angularVel = 0;
        prevRearLeftPos = prevRearRightPos = 0;
        prevFrontLeftPos = prevFrontRightPos = 0;
        firstUpdate = true;
        cout << "[Odometry] Reset to origin" << endl;
    }
};

int main(int argc, char **argv) {
    cout << "========================================" << endl;
    cout << "Pioneer 3-AT Odometry (Rear-Wheel Drive)" << endl;
    cout << "Front wheels are passive (free-rolling)" << endl;
    cout << "========================================" << endl;
    
    Robot *robot = new Robot();
    Keyboard *keyboard = new Keyboard();
    int timeStep = (int)robot->getBasicTimeStep();
    
    // Enable keyboard
    keyboard->enable(timeStep);
    
    // Initialize REAR motors (these are the only powered wheels)
    Motor *rearLeftMotor = robot->getMotor("back left wheel");
    Motor *rearRightMotor = robot->getMotor("back right wheel");
    
    if (rearLeftMotor && rearRightMotor) {
        // Set motors to velocity control mode
        rearLeftMotor->setPosition(INFINITY);
        rearRightMotor->setPosition(INFINITY);
        
        rearLeftMotor->setVelocity(0.0);
        rearRightMotor->setVelocity(0.0);
        
        cout << "[Motors] Rear wheels powered and ready" << endl;
    } else {
        cout << "[ERROR] Rear motors not found!" << endl;
        cout << "Expected: 'back left wheel' and 'back right wheel'" << endl;
        delete robot;
        return -1;
    }
    
    // Check if front motors exist (but we won't control them - they remain free)
    Motor *frontLeftMotor = robot->getMotor("front left wheel");
    Motor *frontRightMotor = robot->getMotor("front right wheel");
    
    if (frontLeftMotor && frontRightMotor) {
        // IMPORTANT: Set front wheels to free mode (no position control, no velocity control)
        // This makes them passive/loose wheels that just follow the robot's motion
        frontLeftMotor->setPosition(INFINITY);
        frontRightMotor->setPosition(INFINITY);
        
        // Set velocity to 0 and let physics handle the rolling
        frontLeftMotor->setVelocity(0.0);
        frontRightMotor->setVelocity(0.0);
        
        // Disable motor torque to make them truly passive (free-rolling)
        frontLeftMotor->setTorque(0.0);
        frontRightMotor->setTorque(0.0);
        
        cout << "[Motors] Front wheels set to PASSIVE (free-rolling mode)" << endl;
    } else {
        cout << "[WARNING] Front motors not found - robot may have only 2 wheels" << endl;
    }
    
    // Initialize ALL FOUR encoders for accurate odometry
    PositionSensor *rearLeftEncoder = robot->getPositionSensor("back left wheel sensor");
    PositionSensor *rearRightEncoder = robot->getPositionSensor("back right wheel sensor");
    PositionSensor *frontLeftEncoder = robot->getPositionSensor("front left wheel sensor");
    PositionSensor *frontRightEncoder = robot->getPositionSensor("front right wheel sensor");
    
    int encoderCount = 0;
    
    if (rearLeftEncoder) {
        rearLeftEncoder->enable(timeStep);
        encoderCount++;
        cout << "[Encoder] Rear left sensor enabled" << endl;
    }
    if (rearRightEncoder) {
        rearRightEncoder->enable(timeStep);
        encoderCount++;
        cout << "[Encoder] Rear right sensor enabled" << endl;
    }
    if (frontLeftEncoder) {
        frontLeftEncoder->enable(timeStep);
        encoderCount++;
        cout << "[Encoder] Front left sensor enabled (passive tracking)" << endl;
    }
    if (frontRightEncoder) {
        frontRightEncoder->enable(timeStep);
        encoderCount++;
        cout << "[Encoder] Front right sensor enabled (passive tracking)" << endl;
    }
    
    if (encoderCount >= 2) {
        cout << "[Encoders] " << encoderCount << " wheel sensors enabled" << endl;
        cout << "[Params] Wheel radius: 0.11m, Wheel base: 0.394m" << endl;
        cout << "[Params] Wheel circumference: " << (2.0 * M_PI * 0.11) << "m" << endl;
        cout << "[Params] Drive configuration: REAR-WHEEL DRIVE only" << endl;
    } else {
        cout << "[ERROR] Insufficient encoders found! Need at least 2." << endl;
        delete robot;
        return -1;
    }
    
    cout << "========================================" << endl;
    cout << "Controls:" << endl;
    cout << "  Arrow Up   : Move forward (rear wheels only)" << endl;
    cout << "  Arrow Down : Move backward (rear wheels only)" << endl;
    cout << "  Arrow Left : Turn left (rear wheels only)" << endl;
    cout << "  Arrow Right: Turn right (rear wheels only)" << endl;
    cout << "  Space      : Stop" << endl;
    cout << "  R          : Reset odometry" << endl;
    cout << "  P          : Print current pose" << endl;
    cout << "  W          : Print raw wheel readings" << endl;
    cout << "========================================" << endl;
    cout << "NOTE: Front wheels are PASSIVE and free-rolling" << endl;
    cout << "========================================" << endl;
    
    Odometry odom;
    
    // Control variables (only for rear wheels)
    double rearLeftSpeed = 0.0;
    double rearRightSpeed = 0.0;
    double maxSpeed = 6.0;      // Maximum wheel speed (rad/s)
    
    int iteration = 0;
    int printInterval = 500 / timeStep;  // Print every ~500ms
    
    while (robot->step(timeStep) != -1) {
        double currentTime = robot->getTime();
        
        // Read ALL FOUR encoders (front wheels are passive but we still read them)
        double rearLeftPos = rearLeftEncoder ? rearLeftEncoder->getValue() : 0;
        double rearRightPos = rearRightEncoder ? rearRightEncoder->getValue() : 0;
        double frontLeftPos = frontLeftEncoder ? frontLeftEncoder->getValue() : 0;
        double frontRightPos = frontRightEncoder ? frontRightEncoder->getValue() : 0;
        
        // Update odometry with all four wheel readings
        odom.update(rearLeftPos, rearRightPos, frontLeftPos, frontRightPos, currentTime);
        
        // Handle keyboard input
        int key = keyboard->getKey();
        
        while (key != -1) {
            switch (key) {
                case 'R':
                case 'r':
                    odom.reset();
                    break;
                    
                case 'P':
                case 'p':
                    cout << "[Pose] ";
                    odom.printPose();
                    break;
                    
                case 'W':
                case 'w':
                    cout << "[Wheel Readings] ";
                    odom.printWheelOdometry(rearLeftPos, rearRightPos, frontLeftPos, frontRightPos);
                    break;
                    
                case ' ':
                    rearLeftSpeed = 0.0;
                    rearRightSpeed = 0.0;
                    cout << "[Stop] Robot stopped at t=" << fixed << setprecision(2) << currentTime << "s" << endl;
                    break;
                    
                case Keyboard::UP:
                    rearLeftSpeed = -maxSpeed;
                    rearRightSpeed = -maxSpeed;
                    cout << "[Forward] Rear wheels moving at " << maxSpeed << " rad/s" << endl;
                    break;
                    
                case Keyboard::DOWN:
                    rearLeftSpeed = maxSpeed;
                    rearRightSpeed = maxSpeed;
                    cout << "[Backward] Rear wheels moving at " << maxSpeed << " rad/s" << endl;
                    break;
                    
                case Keyboard::LEFT:
                    rearLeftSpeed = -maxSpeed;
                    rearRightSpeed = maxSpeed;
                    cout << "[Turn Left] Rear wheels rotating" << endl;
                    break;
                    
                case Keyboard::RIGHT:
                    rearLeftSpeed = maxSpeed;
                    rearRightSpeed = -maxSpeed;
                    cout << "[Turn Right] Rear wheels rotating" << endl;
                    break;
            }
            key = keyboard->getKey();
        }
        
        // Apply motor commands ONLY to rear wheels (front wheels are passive)
        rearLeftMotor->setVelocity(rearLeftSpeed);
        rearRightMotor->setVelocity(rearRightSpeed);
        
        // Print pose at regular intervals
        if (iteration % printInterval == 0) {
            if (rearLeftSpeed != 0 || rearRightSpeed != 0) {
                cout << "t=" << fixed << setprecision(2) << currentTime << "s | ";
                odom.printPose();
            } else if (iteration % (printInterval * 5) == 0) {
                cout << "t=" << fixed << setprecision(2) << currentTime << "s [idle] | ";
                odom.printPose();
            }
        }
        
        iteration++;
    }
    
    delete keyboard;
    delete robot;
    return 0;
}