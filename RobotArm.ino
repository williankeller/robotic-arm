#include <Wire.h>
#include <Servo.h>

// Define a structure for each part of the arm
struct ArmPart {
    String name;
    int pin;
    int minAngle;
    int maxAngle;
    Servo servo;  // Servo object for each part
};

// Define a structure for the robot arm
struct RobotArm {
    ArmPart base;
    ArmPart shoulder;
    ArmPart elbow;
    ArmPart wrist;
    ArmPart gripper;
};

// Initialize the robot arm with specific values
RobotArm arm = {
    {"base",     3, 0, 180},   // base, pin, min, max
    {"shoulder", 5, 0, 180},   // shoulder, pin, min, max
    {"elbow",    6, 0, 180},   // elbow, pin, min, max
    {"wrist",    9, 0, 180},   // wrist, pin, min, max
    {"gripper",  10, 25, 155}  // gripper, pin, min, max
};


void setup() {
    Serial.begin(9600);

    // Attach each servo to its corresponding pin
    arm.base.servo.attach(arm.base.pin);
    arm.shoulder.servo.attach(arm.shoulder.pin);
    arm.elbow.servo.attach(arm.elbow.pin);
    arm.wrist.servo.attach(arm.wrist.pin);
    arm.gripper.servo.attach(arm.gripper.pin);
}

void loop() {

    // Check if data is available to read from the serial buffer
    if (Serial.available() > 0) {
        // Read the incoming string until a newline is received
        String data = Serial.readStringUntil('\n');

        if (data.startsWith("base")) {
            int angle = data.substring(5).toInt();
            moveBase(angle);
        }

        if (data.startsWith("shoulder")) {
            int angle = data.substring(9).toInt();
            moveShoulder(angle);
        }

        if (data.startsWith("elbow")) {
            int angle = data.substring(6).toInt();
            moveElbow(angle);
        }

        if (data.startsWith("wrist")) {
            int angle = data.substring(6).toInt();
            moveWrist(angle);
        }

        if (data.startsWith("gripper")) {
            int angle = data.substring(8).toInt();
            moveGripper(angle);
        }

        if (data.startsWith("arm")) {
            int baseAngle = data.substring(4, 7).toInt();
            int shoulderAngle = data.substring(8, 11).toInt();
            int elbowAngle = data.substring(12, 15).toInt();
            int wristAngle = data.substring(16, 19).toInt();
            moveArm(baseAngle, shoulderAngle, elbowAngle, wristAngle);
        }
    }
}
// Shoulder min 40

// Function to set the servo angle
void setServoPosition(ArmPart part, int angle) {
    Serial.print("Pin: ");
    Serial.print(part.pin);
    Serial.print(": ");
    Serial.print(part.name);
    Serial.print(", Angle: ");
    Serial.print(angle);
    Serial.print("\n");

    angle = constrain(angle, part.minAngle, part.maxAngle);
    part.servo.write(angle);  // Use the write method to set the servo position
}

void moveBase(int angle) {
    setServoPosition(arm.base, angle);
}

void moveShoulder(int angle) {
    setServoPosition(arm.shoulder, angle);
}

void moveElbow(int angle) {
    setServoPosition(arm.elbow, angle);
}

void moveWrist(int angle) {
    setServoPosition(arm.wrist, angle);
}

void moveGripper(int angle) {
    setServoPosition(arm.gripper, angle);
}

void moveArm(int baseAngle, int shoulderAngle, int elbowAngle, int wristAngle) {
    moveBase(baseAngle);
    moveShoulder(shoulderAngle);
    moveElbow(elbowAngle);
    moveWrist(wristAngle);
}
