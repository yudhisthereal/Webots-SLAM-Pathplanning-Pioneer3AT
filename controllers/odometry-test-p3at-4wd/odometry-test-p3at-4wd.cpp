// File: odometry-test.cpp
// Description: Pioneer AT3 Wheel Odometry - prints position and orientation
// Now uses all 4 wheel encoders for accurate odometry

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
    
    // Previous values for all four wheels
    double prevBackLeftPos = 0;
    double prevBackRightPos = 0;
    double prevFrontLeftPos = 0;
    double prevFrontRightPos = 0;
    double prevTime = 0;
    
    double x = 0, y = 0, theta = 0;
    double linearVel = 0, angularVel = 0;
    bool firstUpdate = true;
    
public:
    void update(double backLeftPos, double backRightPos, 
                double frontLeftPos, double frontRightPos, 
                double currentTime) {
        if (firstUpdate) {
            prevBackLeftPos = backLeftPos;
            prevBackRightPos = backRightPos;
            prevFrontLeftPos = frontLeftPos;
            prevFrontRightPos = frontRightPos;
            prevTime = currentTime;
            firstUpdate = false;
            return;
        }
        
        double dt = currentTime - prevTime;
        if (dt <= 0) return;
        
        // CORRECTED: distance = delta_radians * radius (NOT circumference!)
        double backLeftDist = (backLeftPos - prevBackLeftPos) * wheelRadius;
        double backRightDist = (backRightPos - prevBackRightPos) * wheelRadius;
        double frontLeftDist = (frontLeftPos - prevFrontLeftPos) * wheelRadius;
        double frontRightDist = (frontRightPos - prevFrontRightPos) * wheelRadius;
        
        // Average left wheels and right wheels
        double leftDist = (backLeftDist + frontLeftDist) / 2.0;
        double rightDist = (backRightDist + frontRightDist) / 2.0;
        
        // Calculate velocities
        linearVel = (leftDist + rightDist) / (2.0 * dt);
        angularVel = (rightDist - leftDist) / (wheelBase * dt);
        
        // Update pose
        double distance = (leftDist + rightDist) / 2.0;
        double deltaTheta = (rightDist - leftDist) / wheelBase;
        
        theta += deltaTheta;
        x += distance * cos(theta);
        y += distance * sin(theta);
        
        // Normalize theta to [-pi, pi]
        while (theta > M_PI) theta -= 2 * M_PI;
        while (theta < -M_PI) theta += 2 * M_PI;
        
        // Store for next iteration
        prevBackLeftPos = backLeftPos;
        prevBackRightPos = backRightPos;
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
    
    void reset() {
        x = y = theta = 0;
        linearVel = angularVel = 0;
        prevBackLeftPos = prevBackRightPos = 0;
        prevFrontLeftPos = prevFrontRightPos = 0;
        firstUpdate = true;
        cout << "[Odometry] Reset to origin" << endl;
    }
};

int main(int argc, char **argv) {
    cout << "========================================" << endl;
    cout << "Pioneer 3-AT Wheel Odometry (4-Wheel)" << endl;
    cout << "========================================" << endl;
    
    Robot *robot = new Robot();
    Keyboard *keyboard = new Keyboard();
    int timeStep = (int)robot->getBasicTimeStep();
    
    // Enable keyboard
    keyboard->enable(timeStep);
    
    // Initialize motors (required for movement)
    Motor *backLeftMotor = robot->getMotor("back left wheel");
    Motor *backRightMotor = robot->getMotor("back right wheel");
    Motor *frontLeftMotor = robot->getMotor("front left wheel");
    Motor *frontRightMotor = robot->getMotor("front right wheel");
    
    if (backLeftMotor && backRightMotor && frontLeftMotor && frontRightMotor) {
        // Set motors to velocity control mode
        backLeftMotor->setPosition(INFINITY);
        backRightMotor->setPosition(INFINITY);
        frontLeftMotor->setPosition(INFINITY);
        frontRightMotor->setPosition(INFINITY);
        
        backLeftMotor->setVelocity(0.0);
        backRightMotor->setVelocity(0.0);
        frontLeftMotor->setVelocity(0.0);
        frontRightMotor->setVelocity(0.0);
        
        cout << "[Motors] All 4 wheels ready" << endl;
    } else {
        cout << "[ERROR] Motors not found!" << endl;
        delete robot;
        return -1;
    }
    
    // Initialize ALL FOUR encoders
    PositionSensor *backLeftEncoder = robot->getPositionSensor("back left wheel sensor");
    PositionSensor *backRightEncoder = robot->getPositionSensor("back right wheel sensor");
    PositionSensor *frontLeftEncoder = robot->getPositionSensor("front left wheel sensor");
    PositionSensor *frontRightEncoder = robot->getPositionSensor("front right wheel sensor");
    
    if (backLeftEncoder && backRightEncoder && frontLeftEncoder && frontRightEncoder) {
        backLeftEncoder->enable(timeStep);
        backRightEncoder->enable(timeStep);
        frontLeftEncoder->enable(timeStep);
        frontRightEncoder->enable(timeStep);
        cout << "[Encoders] All 4 wheel sensors enabled" << endl;
        cout << "[Params] Wheel radius: 0.11m, Wheel base: 0.394m" << endl;
        cout << "[Params] Wheel circumference: " << (2.0 * M_PI * 0.11) << "m" << endl;
    } else {
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
    cout << "========================================" << endl;
    
    Odometry odom;
    
    // Control variables
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    double maxSpeed = 0.5;  // Maximum wheel speed (rad/s)
    
    int iteration = 0;
    int printInterval = 500 / timeStep;  // Print every ~500ms
    
    while (robot->step(timeStep) != -1) {
        double currentTime = robot->getTime();
        
        // Read ALL FOUR encoders (values in radians as per WebOTS documentation)
        double backLeftPos = backLeftEncoder->getValue();
        double backRightPos = backRightEncoder->getValue();
        double frontLeftPos = frontLeftEncoder->getValue();
        double frontRightPos = frontRightEncoder->getValue();
        
        // Update odometry with all four wheel readings
        odom.update(backLeftPos, backRightPos, frontLeftPos, frontRightPos, currentTime);
        
        // Optional: Print raw encoder values for debugging
        // odom.printRawEncoders(backLeftPos, backRightPos, frontLeftPos, frontRightPos);
        
        // Handle keyboard input
        int key = keyboard->getKey();
        
        while (key != -1) {
            switch (key) {
                case 'R':
                case 'r':
                    odom.reset();
                    break;
                case ' ':  // Space bar
                    leftSpeed = 0.0;
                    rightSpeed = 0.0;
                    cout << "[Stop] Robot stopped" << endl;
                    break;
                case Keyboard::UP:
                    leftSpeed = maxSpeed;
                    rightSpeed = maxSpeed;
                    break;
                case Keyboard::DOWN:
                    leftSpeed = -maxSpeed;
                    rightSpeed = -maxSpeed;
                    break;
                case Keyboard::LEFT:
                    leftSpeed = -maxSpeed;
                    rightSpeed = maxSpeed;
                    break;
                case Keyboard::RIGHT:
                    leftSpeed = maxSpeed;
                    rightSpeed = -maxSpeed;
                    break;
            }
            key = keyboard->getKey();  // Get next key
        }
        
        // Apply motor commands to ALL FOUR wheels
        backLeftMotor->setVelocity(leftSpeed);
        backRightMotor->setVelocity(rightSpeed);
        frontLeftMotor->setVelocity(leftSpeed);
        frontRightMotor->setVelocity(rightSpeed);
        
        // Print pose at regular intervals
        if (iteration % printInterval == 0 && leftSpeed != 0 && rightSpeed != 0) {
            cout << "t=" << fixed << setprecision(2) << currentTime << "s | ";
            odom.printPose();
        }
        
        iteration++;
    }
    
    delete keyboard;
    delete robot;
    return 0;
}