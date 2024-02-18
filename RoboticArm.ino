#include <Wire.h>
#include <Servo.h>
#include <math.h>

// Define PI for trigonometric calculations
#define PI 3.14159265358979323846

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
    ArmPart base;     // Base to Shoulder is 81mm Turns 180° on its axis
    ArmPart shoulder; // Shoulder joint to Elbow joint is 104mm
    ArmPart elbow;    // Elbow joint to the Wrist joint is 96 mm
    ArmPart wrist;    // Wrist joint to the Gripper point is 125mm
    ArmPart hand;     // Hand 130mm - Rotates the wrist 180° on its axis
    ArmPart gripper;  // Gripper max open 58mm
};

// Initialize the arm (part name, pin on the board, min angle, max angle)
RobotArm arm = {
    {"base",     3, 30, 150}, // base, pin, min, max
    {"shoulder", 5, 0, 180},  // shoulder, pin, min, max
    {"elbow",    6, 0, 180},  // elbow, pin, min, max
    {"wrist",    9, 0, 180},  // wrist, pin, min, max
    {"hand",     10, 0, 180}, // hand, pin, min, max
    {"gripper",  11, 0, 98}   // gripper, pin, min, max
};


void setup() {
    Serial.begin(9600);

    // Attach each servo to its corresponding pin
    arm.base.servo.attach(arm.base.pin);
    arm.shoulder.servo.attach(arm.shoulder.pin);
    arm.elbow.servo.attach(arm.elbow.pin);
    arm.wrist.servo.attach(arm.wrist.pin);
    arm.hand.servo.attach(arm.hand.pin);
    arm.gripper.servo.attach(arm.gripper.pin);

    // Move the arm to its initial position
    moveInitialPosition();
    openGripper();
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

        if (data.startsWith("hand")) {
            int angle = data.substring(5).toInt();
            moveHand(angle);
        }

        if (data.startsWith("gripper")) {
            int value = data.substring(8).toInt();
            if (value == 1) {
                closeGripper();
            } else {
                openGripper();
            }
        }

        if (data.startsWith("grab")) {
          grab();
        }

        if (data.startsWith("position")) {
            // Extract the x, y, z coordinates from the string
            int firstComma = data.indexOf(',');
            int secondComma = data.indexOf(',', firstComma + 1);
            float x = data.substring(9, firstComma).toFloat();
            float y = data.substring(firstComma + 1, secondComma).toFloat();
            float z = data.substring(secondComma + 1).toFloat();

            // Move the arm to the specified position
            moveToPosition(x, y, z);
        }
    }
}

// Function to set the servo angle
void setServoPosition(ArmPart &part, int targetAngle) {
    targetAngle = constrain(targetAngle, part.minAngle, part.maxAngle); // Constrain the target angle within limits

    // Read the current position of the servo
    int currentAngle = part.servo.read();

    // Define the step size for smoother movement
    int stepSize = 1;

    if (currentAngle < targetAngle) {
        for (int pos = currentAngle; pos < targetAngle; pos += stepSize) {
            part.servo.write(pos); // Move to the next position
            delay(15); // Wait for 15ms to slow down the movement
        }
    } else {
        for (int pos = currentAngle; pos > targetAngle; pos -= stepSize) {
            part.servo.write(pos); // Move to the next position
            delay(15); // Wait for 15ms to slow down the movement
        }
    }
    // For debugging
    Serial.print("Pin: ");
    Serial.print(part.pin);
    Serial.print(": ");
    Serial.print(part.name);
    Serial.print(", Current: ");
    Serial.print(currentAngle);
    Serial.print(", Angle: ");
    Serial.println(targetAngle);
}

void moveInitialPosition() {
    moveBase(90);
    moveShoulder(130);
    moveElbow(160);
    moveWrist(90);
    moveHand(90);
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

void moveHand(int angle) {
    setServoPosition(arm.hand, angle);
}

void closeGripper() {
    setServoPosition(arm.gripper, 180);
}

void openGripper() {
    setServoPosition(arm.gripper, 0);
}

void grab() {
    moveShoulder(90);//90
    moveElbow(130);
    moveWrist(60);
    delay(200);
    moveHand(180);
    openGripper();
    moveElbow(150);
    moveShoulder(50);
    moveWrist(80);
    moveElbow(160);
    delay(500);
    closeGripper();
    delay(500);
    moveInitialPosition();
    delay(300);
    moveBase(60);
    moveShoulder(100);
    moveWrist(30);
    moveHand(90);
    delay(1000);
    openGripper();
    delay(1000);
    moveInitialPosition();
}

// Pseudocode for an inverse kinematics function
void moveToPosition(float x, float y, float z) {
    // Calculate each servo angle required to reach (x, y, z)
    int baseAngle = calculateBaseAngle(x, y);
    int shoulderAngle = calculateShoulderAngle(x, y, z);
    int elbowAngle = calculateElbowAngle(x, y, z, shoulderAngle);
    int wristAngle = calculateWristAngle(shoulderAngle, elbowAngle);

    // Use existing functions to move each part
    //moveBase(baseAngle);
    moveShoulder(shoulderAngle);
    moveElbow(elbowAngle);
    moveWrist(wristAngle);
    // Hand and gripper movements can be adjusted based on the task
}

int calculateBaseAngle(float x, float y) {
    // Calculate the direction angle in radians
    float angleRadians = atan2(y, x);

    float piAngle = (180.0 / PI);

    // Convert radians to degrees
    float angleDegrees = angleRadians * piAngle;

    Serial.print("Base: X, Y: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print("; Radians: ");
    Serial.print(angleRadians);
    Serial.print("; PI Angle: ");
    Serial.print(piAngle);
    Serial.print("; Degrees: ");
    Serial.println(angleDegrees);

    return angleDegrees;
}

int calculateShoulderAngle(float x, float y, float z) {
    // Calculate the planar distance from the base to the target point
    float distance = sqrt(x * x + y * y);

    // Height from the base to the target point
    float height = z - 81; // Assuming 81mm is the height of the base to the shoulder joint

    // Calculate the distance from the shoulder joint to the target point
    float hypotenuse = sqrt(distance * distance + height * height);

    float shoulderLength = 104.00; // Shoulder to elbow
    float elbowLength = 97.00; // Elbow to wrist (excluding the wrist to gripper length)

    // Ensure the argument for acos is within the valid range [-1, 1]
    float acosArgument = (shoulderLength * shoulderLength + hypotenuse * hypotenuse - elbowLength * elbowLength) / (2 * shoulderLength * hypotenuse);
    acosArgument = max(min(acosArgument, 1.0f), -1.0f);

    // Calculate the angle between the shoulder and the hypotenuse
    float angle = acos(acosArgument);

    // Convert radians to degrees
    int angleDegrees = angle * (180.0 / PI);

    int shoulderServoAngle;
    // Adjust these conditions based on servo offset and geometry.
    if (z > 200) {
        shoulderServoAngle = 90 + (z - 200) / 2;
    } else if (z < 200) {
        shoulderServoAngle = 90 - (200 - z) / 2;
    } else {
        shoulderServoAngle = 90;
    }

    // Debugging Output
    Serial.print("Shoulder: X, Y, Z: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.print("; Distance: ");
    Serial.print(distance);
    Serial.print("; Height: ");
    Serial.print(height);
    Serial.print("; Hypotenuse: ");
    Serial.print(hypotenuse);
    Serial.print("; Acos Argument: ");
    Serial.print(acosArgument);
    Serial.print("; Angle (rad): ");
    Serial.print(angle);
    Serial.print("; Degrees: ");
    Serial.print(angleDegrees);
    Serial.print("; Final Angle: ");
    Serial.println(shoulderServoAngle);

    return shoulderServoAngle;
}

int calculateElbowAngle(float x, float y, float z, int shoulderAngle) {
    // Calculate the planar distance from the base to the target point
    float distance = sqrt(x * x + y * y);

    // Height from the base to the target point
    float height = z - 81; // Assuming 81mm is the height of the base to the shoulder joint

    // Calculate the distance from the shoulder joint to the target point
    float hypotenuse = sqrt(distance * distance + height * height);

    float shoulderLength = 104.00; // Shoulder to elbow
    float elbowLength = 97.00; // Elbow to wrist (excluding the wrist to gripper length)

    // Use the law of cosines to calculate the elbow joint angle
    float acosArgument = (shoulderLength * shoulderLength + elbowLength * elbowLength - hypotenuse * hypotenuse) / (2 * shoulderLength * elbowLength);
    acosArgument = max(min(acosArgument, 1.0f), -1.0f); // Ensure the argument is within the valid range [-1, 1]

    // Calculate the elbow joint angle
    float elbowJointAngle = acos(acosArgument);

    // Convert radians to degrees
    int angleDegrees = elbowJointAngle * (180.0 / PI);

    // Debugging Output
    Serial.print("Elbow: X, Y, Z: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.print("; Distance: ");
    Serial.print(distance);
    Serial.print("; Height: ");
    Serial.print(height);
    Serial.print("; Hypotenuse: ");
    Serial.print(hypotenuse);
    Serial.print("; Shoulder-Elbow Angle (rad): ");
    Serial.print(elbowJointAngle);
    Serial.print("; Degrees: ");
    Serial.println(angleDegrees);

    return angleDegrees;
}

int calculateWristAngle(int shoulderAngle, int elbowAngle) {
    // Calculate the wrist angle based on the shoulder and elbow angles
    int wristAngle = 180 - shoulderAngle - elbowAngle;

    // Debugging Output
    Serial.print("Wrist: Shoulder Angle: ");
    Serial.print(shoulderAngle);
    Serial.print("; Elbow Angle: ");
    Serial.print(elbowAngle);
    Serial.print("; Wrist Angle: ");
    Serial.println(wristAngle);

    return wristAngle;
}
