#include "ArmParts.h"

// Smooth step easing function: S-curve for natural acceleration/deceleration
// Input t in [0.0, 1.0], output in [0.0, 1.0]
float smoothStep(float t) {
    return t * t * (3.0f - 2.0f * t);
}

void attachServos() {
    // Attach each servo to its corresponding pin
    arm.base.servo.attach(arm.base.pin);
    arm.base.servo.write(arm.base.defaultAngle);

    arm.shoulder.servo.attach(arm.shoulder.pin);
    arm.shoulder.servo.write(arm.shoulder.defaultAngle);

    arm.elbow.servo.attach(arm.elbow.pin);
    arm.elbow.servo.write(arm.elbow.defaultAngle);

    arm.wrist.servo.attach(arm.wrist.pin);
    arm.wrist.servo.write(arm.wrist.defaultAngle);

    arm.hand.servo.attach(arm.hand.pin);
    arm.hand.servo.write(arm.hand.defaultAngle);

    arm.gripper.servo.attach(arm.gripper.pin);
    arm.gripper.servo.write(arm.gripper.defaultAngle);
}

// Move a single servo to a target angle with ease-in/ease-out
// stepDelayMs controls overall speed (lower = faster)
void setServoPosition(ArmPart &part, int targetAngle, int stepDelayMs = 15) {
    targetAngle = constrain(targetAngle, part.minAngle, part.maxAngle);
    int currentAngle = part.servo.read();

    int totalSteps = abs(targetAngle - currentAngle);
    if (totalSteps == 0) return;

    for (int step = 1; step <= totalSteps; step++) {
        float t = (float)step / (float)totalSteps;
        float easedT = smoothStep(t);
        int pos = currentAngle + (int)((targetAngle - currentAngle) * easedT);
        part.servo.write(pos);
        delay(stepDelayMs);
    }
    // Ensure exact target is reached
    part.servo.write(targetAngle);

    Serial.print(part.name);
    Serial.print(": ");
    Serial.print(currentAngle);
    Serial.print(" -> ");
    Serial.println(targetAngle);
}

// Maximum number of joints that can be moved simultaneously
#define MAX_SYNC_JOINTS 6

// Move multiple joints simultaneously with easing
// All joints start and finish at the same time, producing coordinated motion
void moveJointsSynchronized(ArmPart *parts[], int targets[], int numJoints, int stepDelayMs = 15) {
    if (numJoints > MAX_SYNC_JOINTS) numJoints = MAX_SYNC_JOINTS;

    int currentAngles[MAX_SYNC_JOINTS];
    int maxDistance = 0;

    // Read current positions and find the longest travel distance
    for (int i = 0; i < numJoints; i++) {
        targets[i] = constrain(targets[i], parts[i]->minAngle, parts[i]->maxAngle);
        currentAngles[i] = parts[i]->servo.read();
        int distance = abs(targets[i] - currentAngles[i]);
        if (distance > maxDistance) {
            maxDistance = distance;
        }
    }

    if (maxDistance == 0) return;

    // Use the longest travel as the step count so we get 1° resolution on the largest move
    int totalSteps = maxDistance;

    for (int step = 1; step <= totalSteps; step++) {
        float t = (float)step / (float)totalSteps;
        float easedT = smoothStep(t);

        for (int i = 0; i < numJoints; i++) {
            int pos = currentAngles[i] + (int)((targets[i] - currentAngles[i]) * easedT);
            parts[i]->servo.write(pos);
        }
        delay(stepDelayMs);
    }

    // Ensure all joints reach their exact targets
    for (int i = 0; i < numJoints; i++) {
        parts[i]->servo.write(targets[i]);
        Serial.print(parts[i]->name);
        Serial.print(": ");
        Serial.print(currentAngles[i]);
        Serial.print(" -> ");
        Serial.println(targets[i]);
    }
}
