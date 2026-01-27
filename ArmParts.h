#include <Servo.h>

// Servo type: PWM (standard) or BUS (Hiwonder LX serial protocol)
enum ServoType { SERVO_PWM, SERVO_BUS };

// Define a structure for each part of the arm
struct ArmPart {
    String name;
    int pin;          // PWM: Arduino pin number, BUS: servo ID
    int minAngle;
    int maxAngle;
    int defaultAngle;
    ServoType type;
    Servo servo;      // Only used for SERVO_PWM
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
// PWM servos: base (pin 3), shoulder (pin 5), gripper (pin 11)
// Bus servos: elbow (ID 1), wrist (ID 2), hand (ID 3)
RobotArm arm = {
// name,       pin/ID, min, max, default, type
    {"base",     3,  35, 150,  90, SERVO_PWM},
    {"shoulder", 5,  0,  180, 140, SERVO_PWM},
    {"elbow",    1,  0,  140, 100, SERVO_BUS},
    {"wrist",    2,  89, 180, 135, SERVO_BUS},
    {"hand",     3,  0,  180,  90, SERVO_BUS},
    {"gripper",  11, 20,  90,  20, SERVO_PWM}
};
