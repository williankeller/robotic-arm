#include <Wire.h>
#include <Servo.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "InverseKinematics.h"
#include "Movements.h"

HUSKYLENS huskylens;

// Number of waypoints for Cartesian trajectory interpolation
#define TRAJECTORY_STEPS 10

// Vision tracking smoothing factor (0.0 = no update, 1.0 = no smoothing)
#define VISION_SMOOTHING 0.15f

// Vision frame center and dead band thresholds
#define VISION_CENTER_X 160
#define VISION_CENTER_Y 120
#define VISION_DEADBAND_X 30
#define VISION_DEADBAND_Y 40

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

// Tracking positions for vision (float for sub-degree smoothing precision)
float currentHorizontalPos = arm.base.defaultAngle;
float shoulderCurrentPos = arm.shoulder.defaultAngle;
float elbowCurrentPos = arm.elbow.defaultAngle;
float wristCurrentPos = arm.wrist.defaultAngle;
float handCurrentPos = arm.hand.defaultAngle;

// Track last IK position for Cartesian trajectory interpolation
float lastX = 0, lastY = 0, lastZ = 0, lastGripAngle = 0;
bool hasLastPosition = false;

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
            hasLastPosition = false;
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
            } else if (value == 0) {
                openGripper();
            } else {
                moveGripper(value);
            }
        }

        if (data.startsWith("grab")) {
            grab();
        }

        if (data.startsWith("position")) {
            // Expected format: "position x,y,z,gripAngle"
            int firstComma = data.indexOf(',');
            int secondComma = data.indexOf(',', firstComma + 1);
            int thirdComma = data.indexOf(',', secondComma + 1);
            float x = data.substring(9, firstComma).toFloat();
            float y = data.substring(firstComma + 1, secondComma).toFloat();
            float z = data.substring(secondComma + 1, thirdComma).toFloat();
            float gripAngle = data.substring(thirdComma + 1).toFloat();

            moveToPosition(x, y, z, gripAngle);
        }
    }
    else {
        while (huskylens.available()) {
            HUSKYLENSResult result = huskylens.read();

            // Compute target positions using proportional control
            float targetHorizontal = currentHorizontalPos;
            float targetShoulder = shoulderCurrentPos;
            float targetElbow = elbowCurrentPos;
            float targetHand = handCurrentPos;

            // Horizontal tracking with dead band
            int errorX = VISION_CENTER_X - result.xCenter;
            if (abs(errorX) > VISION_DEADBAND_X) {
                targetHorizontal += errorX * 0.05f;
                targetHand -= errorX * 0.02f;
            }

            // Vertical tracking with dead band
            int errorY = VISION_CENTER_Y - result.yCenter;
            if (abs(errorY) > VISION_DEADBAND_Y) {
                targetShoulder += errorY * 0.04f;
                targetElbow += errorY * 0.06f;
            }

            // Apply exponential smoothing for fluid motion
            currentHorizontalPos += (targetHorizontal - currentHorizontalPos) * VISION_SMOOTHING;
            shoulderCurrentPos += (targetShoulder - shoulderCurrentPos) * VISION_SMOOTHING;
            elbowCurrentPos += (targetElbow - elbowCurrentPos) * VISION_SMOOTHING;
            handCurrentPos += (targetHand - handCurrentPos) * VISION_SMOOTHING;

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
            Serial.print(F("----- Distance: "));
            Serial.print(F("W: "));
            Serial.print(width);
            Serial.print(F(", H: "));
            Serial.print(height);
            Serial.print(F(", D: "));
            Serial.println(distance);

            // Distance-based depth adjustment
            float targetWrist = wristCurrentPos;
            if (distance < 6) {
                targetShoulder += 2;
                targetElbow -= 2;
                targetWrist += 2;
            }
            wristCurrentPos += (targetWrist - wristCurrentPos) * VISION_SMOOTHING;

            // Write directly to servos for responsive vision tracking
            // (no easing needed since adjustments are small and continuous)
            arm.base.servo.write(constrain((int)currentHorizontalPos, arm.base.minAngle, arm.base.maxAngle));
            arm.shoulder.servo.write(constrain((int)shoulderCurrentPos, arm.shoulder.minAngle, arm.shoulder.maxAngle));
            arm.elbow.servo.write(constrain((int)elbowCurrentPos, arm.elbow.minAngle, arm.elbow.maxAngle));
            arm.wrist.servo.write(constrain((int)wristCurrentPos, arm.wrist.minAngle, arm.wrist.maxAngle));
            arm.hand.servo.write(constrain((int)handCurrentPos, arm.hand.minAngle, arm.hand.maxAngle));
        }
    }
}


// Move to a Cartesian position using inverse kinematics
// Uses Cartesian linear interpolation for straight-line end-effector paths
void moveToPosition(float x, float y, float z, float gripAngle) {
    float baseAngle, r, shoulderAngle, elbowAngle, wristAngle;

    if (hasLastPosition) {
        // Interpolate in Cartesian space for a straight-line tool path
        for (int step = 1; step <= TRAJECTORY_STEPS; step++) {
            float t = (float)step / (float)TRAJECTORY_STEPS;

            float interpX = lastX + (x - lastX) * t;
            float interpY = lastY + (y - lastY) * t;
            float interpZ = lastZ + (z - lastZ) * t;
            float interpGrip = lastGripAngle + (gripAngle - lastGripAngle) * t;

            solveXYZ(interpX, interpY, baseAngle, r);
            if (!solveRZ(r, interpZ, interpGrip, shoulderAngle, elbowAngle, wristAngle)) {
                Serial.println(F("Position unreachable during trajectory"));
                return;
            }

            // Convert radians to degrees and move all joints simultaneously
            ArmPart *parts[] = {&arm.base, &arm.shoulder, &arm.elbow, &arm.wrist};
            int targets[] = {
                (int)radiansToDegrees(baseAngle),
                (int)radiansToDegrees(shoulderAngle),
                (int)radiansToDegrees(elbowAngle),
                (int)radiansToDegrees(wristAngle)
            };
            moveJointsSynchronized(parts, targets, 4, 10);
        }
    } else {
        // First move: go directly to target position
        solveXYZ(x, y, baseAngle, r);
        if (!solveRZ(r, z, gripAngle, shoulderAngle, elbowAngle, wristAngle)) {
            Serial.println(F("Position unreachable"));
            return;
        }

        ArmPart *parts[] = {&arm.base, &arm.shoulder, &arm.elbow, &arm.wrist};
        int targets[] = {
            (int)radiansToDegrees(baseAngle),
            (int)radiansToDegrees(shoulderAngle),
            (int)radiansToDegrees(elbowAngle),
            (int)radiansToDegrees(wristAngle)
        };
        moveJointsSynchronized(parts, targets, 4);
    }

    // Store position for next trajectory interpolation
    lastX = x;
    lastY = y;
    lastZ = z;
    lastGripAngle = gripAngle;
    hasLastPosition = true;
}

// Convert radians to degrees
float radiansToDegrees(float radians) {
    return radians * 180.0f / PI;
}
