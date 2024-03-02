#include <Servo.h>

// Define a structure for each part of the arm
struct ArmPart {
    String name;
    int pin;
    int minAngle;
    int maxAngle;
    int defaultAngle;
    Servo servo;
};

// Define a structure for the robot arm
struct RobotArm {
    ArmPart base;     // Base to Shoulder is 81mm Turns 180° on its axis
    ArmPart shoulder; // Shoulder joint to Elbow joint is 104mm
    ArmPart elbow;    // Elbow joint to the Wrist joint is 96 mm
    ArmPart wrist;    // Wrist joint to the Gripper point is 125mm
    ArmPart hand;     // Hand 130mm - Rotates the wrist 180° on its axis
    ArmPart gripper;  // Gripper max open 58mm
};

// Initialize the arm
RobotArm arm = {
// part name, pin on the board, min angle, max angle, default angle
    {"base",     3, 35, 150, 90},
    {"shoulder", 5, 0, 180, 140},
    {"elbow",    6, 0, 140, 100},
    {"wrist",    9, 89, 180, 135},
    {"hand",     10, 0, 180, 90},
    {"gripper",  11, 20, 90, 20}
};
