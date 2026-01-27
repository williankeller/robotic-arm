#include <Wire.h>
#include <Servo.h>
#include "HUSKYLENS.h"
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

// Number of attempts to initialize HuskyLens before giving up
#define HUSKYLENS_INIT_RETRIES 3

// HuskyLens state
bool huskyLensConnected = false;
bool trackingEnabled = false;

void setup() {
    Serial.begin(9600);

    // Attach servos first so the arm is functional immediately
    attachServos();

    // Try to initialize HuskyLens (non-blocking)
    Wire.begin();
    for (int i = 0; i < HUSKYLENS_INIT_RETRIES; i++) {
        if (huskylens.begin(Wire)) {
            huskyLensConnected = true;
            Serial.println(F("HuskyLens connected."));
            break;
        }
        delay(100);
    }

    if (!huskyLensConnected) {
        Serial.println(F("HuskyLens not found. Arm running in manual mode."));
    }

    Serial.println(F("Ready. Send 'help' for available commands."));
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

// Reset vision tracking positions to defaults
void resetTrackingPositions() {
    currentHorizontalPos = arm.base.defaultAngle;
    shoulderCurrentPos = arm.shoulder.defaultAngle;
    elbowCurrentPos = arm.elbow.defaultAngle;
    wristCurrentPos = arm.wrist.defaultAngle;
    handCurrentPos = arm.hand.defaultAngle;
}

// Write angle to a servo (handles both PWM and bus types for vision tracking)
void writeServoAngle(ArmPart &part, int angle) {
    angle = constrain(angle, part.minAngle, part.maxAngle);
    if (part.type == SERVO_BUS) {
        busServoMove(part.pin, degreesToBusPos(angle), 50);
    } else {
        part.servo.write(angle);
    }
}

void loop() {
    // Always process serial commands, regardless of HuskyLens state
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        handleCommand(data);
        return;
    }

    // Run vision tracking only when enabled and HuskyLens is available
    if (trackingEnabled && huskyLensConnected) {
        runVisionTracking();
    }
}

// Process a serial command
void handleCommand(String &data) {
    if (data.startsWith("reset")) {
        moveInitialPosition();
        resetTrackingPositions();
        hasLastPosition = false;
    }

    else if (data.startsWith("teach")) {
        String param = data.substring(6);
        param.trim();

        if (param == "on") {
            teachStart();
        } else if (param == "capture") {
            teachCapture();
        } else if (param == "play") {
            teachPlay();
        } else if (param == "off") {
            teachStop();
        } else {
            Serial.print(F("Teach mode is "));
            Serial.println(teachMode ? F("on") : F("off"));
            Serial.print(F("Waypoints: "));
            Serial.println(teachCount);
        }
    }

    else if (data.startsWith("tracking")) {
        String param = data.substring(9);
        param.trim();

        if (param == "on") {
            if (!huskyLensConnected) {
                Wire.begin();
                if (huskylens.begin(Wire)) {
                    huskyLensConnected = true;
                    Serial.println(F("HuskyLens connected."));
                } else {
                    Serial.println(F("HuskyLens not found. Connect it and try again."));
                    return;
                }
            }
            trackingEnabled = true;
            resetTrackingPositions();
            Serial.println(F("Auto tracking enabled."));
        } else if (param == "off") {
            trackingEnabled = false;
            Serial.println(F("Auto tracking disabled. Returning to home position."));
            moveInitialPosition();
            resetTrackingPositions();
        } else {
            Serial.print(F("Tracking is "));
            Serial.println(trackingEnabled ? F("on") : F("off"));
        }
    }

    else if (data.startsWith("base")) {
        int angle = data.substring(5).toInt();
        moveBase(angle);
    }

    else if (data.startsWith("shoulder")) {
        int angle = data.substring(9).toInt();
        moveShoulder(angle);
    }

    else if (data.startsWith("elbow")) {
        int angle = data.substring(6).toInt();
        moveElbow(angle);
    }

    else if (data.startsWith("wrist")) {
        int angle = data.substring(6).toInt();
        moveWrist(angle);
    }

    else if (data.startsWith("hand")) {
        int angle = data.substring(5).toInt();
        moveHand(angle);
    }

    else if (data.startsWith("gripper")) {
        int value = data.substring(8).toInt();
        if (value == 1) {
            closeGripper();
        } else if (value == 0) {
            openGripper();
        } else {
            moveGripper(value);
        }
    }

    else if (data.startsWith("grab")) {
        grab();
    }

    else if (data.startsWith("position")) {
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

    else if (data.startsWith("help")) {
        Serial.println(F("Commands:"));
        Serial.println(F("  reset              - Return to home position"));
        Serial.println(F("  base <angle>       - Move base (35-150)"));
        Serial.println(F("  shoulder <angle>   - Move shoulder (0-180)"));
        Serial.println(F("  elbow <angle>      - Move elbow (0-140)"));
        Serial.println(F("  wrist <angle>      - Move wrist (89-180)"));
        Serial.println(F("  hand <angle>       - Move hand (0-180)"));
        Serial.println(F("  gripper <0|1|angle> - Open/close/move gripper"));
        Serial.println(F("  grab               - Pick-and-place sequence"));
        Serial.println(F("  position x,y,z,grip - IK move (mm, radians)"));
        Serial.println(F("  tracking on|off    - Enable/disable auto tracking"));
        Serial.println(F("  teach on           - Enter teach mode (bus servos free)"));
        Serial.println(F("  teach capture      - Save current position as waypoint"));
        Serial.println(F("  teach play         - Replay recorded waypoints"));
        Serial.println(F("  teach off          - Exit teach mode"));
        Serial.println(F("  help               - Show this message"));
    }
}

// Run HuskyLens vision tracking loop
void runVisionTracking() {
    if (!huskylens.request()) {
        Serial.println(F("HuskyLens request failed."));
        return;
    }

    if (!huskylens.isLearned()) {
        return;
    }

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

        // Write to servos (handles both PWM and bus types)
        writeServoAngle(arm.base, (int)currentHorizontalPos);
        writeServoAngle(arm.shoulder, (int)shoulderCurrentPos);
        writeServoAngle(arm.elbow, (int)elbowCurrentPos);
        writeServoAngle(arm.wrist, (int)wristCurrentPos);
        writeServoAngle(arm.hand, (int)handCurrentPos);
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
