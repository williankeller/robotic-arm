#include <Wire.h>
#include <Servo.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "InverseKinematics.h"
#include "Movements.h"

HUSKYLENS huskylens;

// Define PI for trigonometric calculations
#define PI 3.14159265358979323846

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
    attachServos();
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
                handCurrentPos += 1;
                
            } else if (result.xCenter < 130) { // Object is on the left
                currentHorizontalPos += 2; // Move servo right
                handCurrentPos -= 1;
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

            // Get the distance from the object
            float width = result.width;
            float height = result.height;
            float distance = 0.0;

            // Calculate distance using the width and height of the object
            if (width > height) {
                distance = 0.5 * 3.3 * 480 / width;
            } else {
                distance = 0.5 * 3.3 * 480 / height;
            }
            Serial.print("----- Distance: ");
            Serial.print("W: ");
            Serial.print(width);
            Serial.print(", H: ");
            Serial.print(height);
            Serial.print(", D: ");
            Serial.println(distance);

            // Store the original position of the arm
            int originalShoulderPos = shoulderCurrentPos;
            int originalElbowPos = elbowCurrentPos;
            int originalWristPos = wristCurrentPos;

            if (distance < 6) {
                // Object is too close, move the arm back
                shoulderCurrentPos += 2;
                elbowCurrentPos -= 2;
                wristCurrentPos += 2;
            } else if (distance > 7) {
                // Move the arm back to the original position
                shoulderCurrentPos = originalShoulderPos;
                elbowCurrentPos = originalElbowPos;
                wristCurrentPos = originalWristPos;
            }
            moveBase(currentHorizontalPos);
            moveShoulder(shoulderCurrentPos);
            moveElbow(elbowCurrentPos);
            moveWrist(wristCurrentPos);
        }
    }
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
