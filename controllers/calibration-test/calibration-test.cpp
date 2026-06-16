// File: calibrate-encoders.cpp (temporary, delete after calibration)
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();
    
    // First, list all available position sensors (debugging)
    cout << "=== AVAILABLE SENSORS ===" << endl;
    cout << "Checking common sensor names..." << endl;
    
    // Try different possible sensor names for Pioneer 3-AT
    const char* sensorNames[] = {
        "back left wheel sensor",
        "back right wheel sensor", 
        "front left wheel sensor",
        "front right wheel sensor",
        "left wheel sensor",
        "right wheel sensor",
        "back left motor sensor",
        "back right motor sensor"
    };
    
    for (const char* name : sensorNames) {
        PositionSensor* test = robot->getPositionSensor(name);
        if (test) {
            cout << "✓ Found: " << name << endl;
        } else {
            cout << "✗ Not found: " << name << endl;
        }
    }
    
    // Get motor and encoder with proper error checking
    Motor *backLeftMotor = robot->getMotor("back left wheel");
    PositionSensor *backLeftEncoder = robot->getPositionSensor("back left wheel sensor");
    
    if (!backLeftMotor) {
        cout << "ERROR: Could not find motor 'back left wheel'" << endl;
        delete robot;
        return -1;
    }
    
    if (!backLeftEncoder) {
        cout << "ERROR: Could not find encoder 'back left wheel sensor'" << endl;
        delete robot;
        return -1;
    }
    
    // Set up motor
    backLeftMotor->setPosition(INFINITY);
    backLeftMotor->setVelocity(0.0);
    
    // Enable encoder and wait for valid reading
    backLeftEncoder->enable(timeStep);
    
    // Run a few steps to get first valid encoder reading
    cout << "Waiting for encoder to stabilize..." << endl;
    for (int i = 0; i < 10; i++) {
        robot->step(timeStep);
    }
    
    double initialValue = backLeftEncoder->getValue();
    if (isnan(initialValue)) {
        cout << "ERROR: Encoder still returning NaN after stabilization" << endl;
        delete robot;
        return -1;
    }
    
    cout << "Initial encoder value: " << fixed << setprecision(6) << initialValue << endl;
    cout << "\n=== ENCODER CALIBRATION ===" << endl;
    cout << "Spinning wheel at 1 rad/s for 10 seconds..." << endl;
    
    backLeftMotor->setVelocity(1.0);
    
    double startPos = backLeftEncoder->getValue();
    double startTime = robot->getTime();
    
    cout << "Start position: " << startPos << " units" << endl;
    
    // Run for 10 seconds (10000ms / timeStep iterations)
    int totalSteps = (10 * 1000) / timeStep;
    for (int i = 0; i < totalSteps; i++) {
        robot->step(timeStep);
        
        // Optional: print progress every second
        if (i % (1000 / timeStep) == 0 && i > 0) {
            double currentPos = backLeftEncoder->getValue();
            cout << "  Progress: " << i * timeStep / 1000 << "s, encoder: " << currentPos << endl;
        }
    }
    
    double endPos = backLeftEncoder->getValue();
    double endTime = robot->getTime();
    
    backLeftMotor->setVelocity(0.0);
    
    // Calculate results
    double commandedRadians = 1.0 * (endTime - startTime);
    double encoderDelta = endPos - startPos;
    double ratio = encoderDelta / commandedRadians;
    
    cout << "\n=== RESULTS ===" << endl;
    cout << fixed << setprecision(4);
    cout << "Start time: " << startTime << " s" << endl;
    cout << "End time:   " << endTime << " s" << endl;
    cout << "Commanded rotation: " << commandedRadians << " rad" << endl;
    cout << "Start encoder: " << startPos << " units" << endl;
    cout << "End encoder:   " << endPos << " units" << endl;
    cout << "Encoder delta: " << encoderDelta << " units" << endl;
    cout << "Ratio (encoder/command): " << ratio << endl;
    
    cout << "\n=== INTERPRETATION ===" << endl;
    if (fabs(ratio - 1.0) < 0.01) {
        cout << "✓ Encoder returns RADIANS (ratio ≈ 1.0)" << endl;
        cout << "  Use: distance = delta * wheelRadius" << endl;
    } else if (fabs(ratio - (1.0/(2*M_PI))) < 0.01) {
        cout << "✓ Encoder returns REVOLUTIONS (ratio ≈ 0.159)" << endl;
        cout << "  Use: distance = delta * wheelCircumference" << endl;
    } else {
        cout << "⚠ Encoder has unexpected scaling factor: " << ratio << endl;
        cout << "  Your effective wheel radius should be: " << (0.11 * ratio) << " m" << endl;
        cout << "  Or keep radius 0.11m and multiply distance by: " << ratio << endl;
    }
    
    delete robot;
    return 0;
}