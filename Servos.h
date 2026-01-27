#include "ArmParts.h"
#include "BusServo.h"

// Smooth step easing function: S-curve for natural acceleration/deceleration
// Input t in [0.0, 1.0], output in [0.0, 1.0]
float smoothStep(float t) {
    return t * t * (3.0f - 2.0f * t);
}

// Get current angle of a servo (works for both PWM and bus types)
int getServoAngle(ArmPart &part) {
    if (part.type == SERVO_BUS) {
        int pos = busServoReadPosition(part.pin);
        if (pos >= 0) {
            return busPosToDegrees(pos);
        }
        return part.defaultAngle; // fallback if read fails
    }
    return part.servo.read();
}

void attachServos() {
    // Initialize bus servo serial line
    initBusServos();
    delay(100);

    // Attach PWM servos
    if (arm.base.type == SERVO_PWM) {
        arm.base.servo.attach(arm.base.pin);
        arm.base.servo.write(arm.base.defaultAngle);
    }
    if (arm.shoulder.type == SERVO_PWM) {
        arm.shoulder.servo.attach(arm.shoulder.pin);
        arm.shoulder.servo.write(arm.shoulder.defaultAngle);
    }
    if (arm.gripper.type == SERVO_PWM) {
        arm.gripper.servo.attach(arm.gripper.pin);
        arm.gripper.servo.write(arm.gripper.defaultAngle);
    }

    // Move bus servos to default positions (1 second move time)
    if (arm.elbow.type == SERVO_BUS) {
        busServoMove(arm.elbow.pin, degreesToBusPos(arm.elbow.defaultAngle), 1000);
    }
    if (arm.wrist.type == SERVO_BUS) {
        busServoMove(arm.wrist.pin, degreesToBusPos(arm.wrist.defaultAngle), 1000);
    }
    if (arm.hand.type == SERVO_BUS) {
        busServoMove(arm.hand.pin, degreesToBusPos(arm.hand.defaultAngle), 1000);
    }
    delay(1000);
}

// Move a single servo to a target angle with ease-in/ease-out
// stepDelayMs controls overall speed (lower = faster)
void setServoPosition(ArmPart &part, int targetAngle, int stepDelayMs = 15) {
    targetAngle = constrain(targetAngle, part.minAngle, part.maxAngle);
    int currentAngle = getServoAngle(part);

    int totalSteps = abs(targetAngle - currentAngle);
    if (totalSteps == 0) return;

    if (part.type == SERVO_BUS) {
        // Bus servo: use built-in timed move (servo handles interpolation)
        int moveTime = totalSteps * stepDelayMs;
        busServoMove(part.pin, degreesToBusPos(targetAngle), moveTime);
        delay(moveTime);
    } else {
        // PWM servo: step loop with easing
        for (int step = 1; step <= totalSteps; step++) {
            float t = (float)step / (float)totalSteps;
            float easedT = smoothStep(t);
            int pos = currentAngle + (int)((targetAngle - currentAngle) * easedT);
            part.servo.write(pos);
            delay(stepDelayMs);
        }
        part.servo.write(targetAngle);
    }

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
// Bus servos use their built-in timed move; PWM servos use step loop with easing
void moveJointsSynchronized(ArmPart *parts[], int targets[], int numJoints, int stepDelayMs = 15) {
    if (numJoints > MAX_SYNC_JOINTS) numJoints = MAX_SYNC_JOINTS;

    int currentAngles[MAX_SYNC_JOINTS];
    int maxDistance = 0;

    // Read current positions and find the longest travel distance
    for (int i = 0; i < numJoints; i++) {
        targets[i] = constrain(targets[i], parts[i]->minAngle, parts[i]->maxAngle);
        currentAngles[i] = getServoAngle(*parts[i]);
        int distance = abs(targets[i] - currentAngles[i]);
        if (distance > maxDistance) {
            maxDistance = distance;
        }
    }

    if (maxDistance == 0) return;

    int totalSteps = maxDistance;
    int totalTime = totalSteps * stepDelayMs;

    // Send bus servo move commands first (they start immediately and run in parallel)
    for (int i = 0; i < numJoints; i++) {
        if (parts[i]->type == SERVO_BUS) {
            busServoMove(parts[i]->pin, degreesToBusPos(targets[i]), totalTime);
        }
    }

    // Check if there are any PWM servos that need step-by-step control
    bool hasPwm = false;
    for (int i = 0; i < numJoints; i++) {
        if (parts[i]->type == SERVO_PWM) {
            hasPwm = true;
            break;
        }
    }

    if (hasPwm) {
        // Step through PWM servos with easing (bus servos move in parallel)
        for (int step = 1; step <= totalSteps; step++) {
            float t = (float)step / (float)totalSteps;
            float easedT = smoothStep(t);

            for (int i = 0; i < numJoints; i++) {
                if (parts[i]->type == SERVO_PWM) {
                    int pos = currentAngles[i] + (int)((targets[i] - currentAngles[i]) * easedT);
                    parts[i]->servo.write(pos);
                }
            }
            delay(stepDelayMs);
        }
    } else {
        // All bus servos - just wait for the timed move to complete
        delay(totalTime);
    }

    // Ensure PWM servos reach exact targets and log all moves
    for (int i = 0; i < numJoints; i++) {
        if (parts[i]->type == SERVO_PWM) {
            parts[i]->servo.write(targets[i]);
        }
        Serial.print(parts[i]->name);
        Serial.print(": ");
        Serial.print(currentAngles[i]);
        Serial.print(" -> ");
        Serial.println(targets[i]);
    }
}
