#include <Wire.h>
#include <Servo.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "InverseKinematics.h"
#include "RobotArm.h"

RobotArm arm;

HUSKYLENS huskylens;

// Define PI for trigonometric calculations
#define PI 3.14159265358979323846

// Define a structure for each part of the arm
struct ArmPart {
    String name;
    int pin;
    int minAngle;
    int maxAngle;
    int defaultAngle;
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

// Initialize the arm (part name, pin on the board, min angle, max angle, default angle)
RobotArm arm = {
    {"base",     3, 35, 150, 90},
    {"shoulder", 5, 0, 180, 140},
    {"elbow",    6, 0, 140, 100},
    {"wrist",    9, 89, 180, 135},
    {"hand",     10, 0, 180, 90},
    {"gripper",  11, 20, 98, 98}
};

void setup() {
    Serial.begin(9600);

    Wire.begin();
    while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }

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

// Initial servo position for the base
int currentHorizontalPos = arm.base.defaultAngle;
int shoulderCurrentPos = arm.shoulder.defaultAngle;
int elbowCurrentPos = arm.elbow.defaultAngle;
int wristCurrentPos = arm.wrist.defaultAngle;
int handCurrentPos = arm.hand.defaultAngle;

void loop() {
    if (!huskylens.request()) {
        Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    } else if (!huskylens.isLearned()) {
        Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    } else if (Serial.available() > 0) {
        // Read the incoming string until a newline is received
        String data = Serial.readStringUntil('\n');

        if (data.startsWith("reset")) {
            moveInitialPosition();
            currentHorizontalPos = arm.base.defaultAngle;
            shoulderCurrentPos = arm.shoulder.defaultAngle;
            elbowCurrentPos = arm.elbow.defaultAngle;
            wristCurrentPos = arm.wrist.defaultAngle;
        }

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
            } else if (value == 1) {
                openGripper();
            } else {
                moveGripper(value);
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
            float gripAngle = data.substring(secondComma + 1).toFloat();

            // Move the arm to the specified position
            moveToPosition(x, y, z, gripAngle);
        }
    }
    else {
        while (huskylens.available()) {
            HUSKYLENSResult result = huskylens.read();

            // Horizontal movement
            if (result.xCenter > 190) { // Object is on the right
                currentHorizontalPos -= 2; // Move servo left
                handCurrentPos += 2;
                
            } else if (result.xCenter < 130) { // Object is on the left
                currentHorizontalPos += 2; // Move servo right
                handCurrentPos -= 2;
            }
            
            // Vertical movement
            if (result.yCenter > 160) {
                // Object is at the bottom, move servo down
                shoulderCurrentPos += 2;
                elbowCurrentPos += 3;
            } else if (result.yCenter < 60) {
                shoulderCurrentPos -= 1.5;
                elbowCurrentPos -= 3;
            }
            
            moveBase(currentHorizontalPos);

            moveShoulder(shoulderCurrentPos);

            moveElbow(elbowCurrentPos); 
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
    moveShoulder(140);
    moveElbow(100);
    moveWrist(135);
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

void moveGripper(int angle) {
    setServoPosition(arm.gripper, angle);
}

void closeGripper() {
    setServoPosition(arm.gripper, 180);
}

void openGripper() {
    setServoPosition(arm.gripper, 0);
}


void moveToPosition(float x, float y, float z, float gripAngle) {
    float baseAngle, r, shoulderAngle, elbowAngle, wristAngle;
    
    solveXYZ(x, y, baseAngle, r);
    solveRZ(r, z, gripAngle, shoulderAngle, elbowAngle, wristAngle);
    
    // Convert radians to degrees for servo positioning
    int currentBaseAngle = arm.base.servo.read();
    int currentShoulderAngle = arm.shoulder.servo.read();
    int currentElbowAngle = arm.elbow.servo.read();
    int currentWristAngle = arm.wrist.servo.read();

    baseAngle += currentBaseAngle + 2;
    shoulderAngle += currentShoulderAngle + 2;
    elbowAngle += currentElbowAngle + 2;
    wristAngle += currentWristAngle + 2;


    // Move the arm to the specified position
    moveBase(radiansToDegrees(baseAngle));
    moveShoulder(radiansToDegrees(shoulderAngle));
    moveElbow(radiansToDegrees(elbowAngle));
    moveWrist(radiansToDegrees(wristAngle));
}

// Convert radians to degrees
float radiansToDegrees(float radians) {
    return radians * 180 / PI;
}
