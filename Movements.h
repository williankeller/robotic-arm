#include "Servos.h"

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

// Move to initial position with all joints synchronized
void moveInitialPosition() {
    ArmPart *parts[] = {&arm.base, &arm.shoulder, &arm.elbow, &arm.wrist, &arm.hand};
    int targets[] = {90, 140, 100, 135, 90};
    moveJointsSynchronized(parts, targets, 5);
}

// Coordinated grab sequence using synchronized multi-joint movements
void grab() {
    // Phase 1: Move arm to pre-grab posture (shoulder + elbow + wrist together)
    {
        ArmPart *parts[] = {&arm.shoulder, &arm.elbow, &arm.wrist};
        int targets[] = {90, 130, 150};
        moveJointsSynchronized(parts, targets, 3);
    }
    delay(200);

    // Phase 2: Rotate hand and open gripper
    moveHand(180);
    openGripper();

    // Phase 3: Extend arm to reach object (shoulder + elbow + wrist together)
    {
        ArmPart *parts[] = {&arm.shoulder, &arm.elbow, &arm.wrist};
        int targets[] = {80, 130, 160};
        moveJointsSynchronized(parts, targets, 3);
    }
    delay(500);

    // Phase 4: Grip the object
    closeGripper();
    delay(500);

    // Phase 5: Lift to safe position
    moveInitialPosition();
    delay(300);

    // Phase 6: Move to drop position (base + shoulder + wrist + hand together)
    {
        ArmPart *parts[] = {&arm.base, &arm.shoulder, &arm.wrist, &arm.hand};
        int targets[] = {60, 100, 160, 90};
        moveJointsSynchronized(parts, targets, 4);
    }
    delay(1000);

    // Phase 7: Release object
    openGripper();
    delay(1000);

    // Phase 8: Return to initial position
    moveInitialPosition();
}

// --- Teach mode: record and replay arm positions ---

#define MAX_WAYPOINTS 20

struct Waypoint {
    int angles[6]; // base, shoulder, elbow, wrist, hand, gripper
};

Waypoint teachWaypoints[MAX_WAYPOINTS];
int teachCount = 0;
bool teachMode = false;

// Enter teach mode: disable torque on bus servos so they can be moved by hand
void teachStart() {
    teachMode = true;
    teachCount = 0;

    // Disable torque on bus servos (they become free to move)
    if (arm.elbow.type == SERVO_BUS) busServoSetTorque(arm.elbow.pin, false);
    if (arm.wrist.type == SERVO_BUS) busServoSetTorque(arm.wrist.pin, false);
    if (arm.hand.type == SERVO_BUS) busServoSetTorque(arm.hand.pin, false);

    Serial.println(F("Teach mode on. Move bus servos (elbow, wrist, hand) by hand."));
    Serial.println(F("Use commands for PWM servos (base, shoulder, gripper)."));
    Serial.println(F("  teach capture - save current position as waypoint"));
    Serial.println(F("  teach play    - replay recorded waypoints"));
    Serial.println(F("  teach off     - exit teach mode"));
}

// Capture current arm position as a waypoint
void teachCapture() {
    if (!teachMode) {
        Serial.println(F("Not in teach mode. Send 'teach on' first."));
        return;
    }
    if (teachCount >= MAX_WAYPOINTS) {
        Serial.println(F("Waypoint storage full (max 20)."));
        return;
    }

    Waypoint &wp = teachWaypoints[teachCount];
    wp.angles[0] = getServoAngle(arm.base);
    wp.angles[1] = getServoAngle(arm.shoulder);
    wp.angles[2] = getServoAngle(arm.elbow);
    wp.angles[3] = getServoAngle(arm.wrist);
    wp.angles[4] = getServoAngle(arm.hand);
    wp.angles[5] = getServoAngle(arm.gripper);

    teachCount++;

    Serial.print(F("Waypoint "));
    Serial.print(teachCount);
    Serial.print(F(" saved: "));
    for (int i = 0; i < 6; i++) {
        Serial.print(wp.angles[i]);
        if (i < 5) Serial.print(F(", "));
    }
    Serial.println();
}

// Replay all recorded waypoints
void teachPlay() {
    if (teachCount == 0) {
        Serial.println(F("No waypoints recorded."));
        return;
    }

    // Re-enable torque on bus servos for playback
    if (arm.elbow.type == SERVO_BUS) busServoSetTorque(arm.elbow.pin, true);
    if (arm.wrist.type == SERVO_BUS) busServoSetTorque(arm.wrist.pin, true);
    if (arm.hand.type == SERVO_BUS) busServoSetTorque(arm.hand.pin, true);
    delay(100);

    Serial.print(F("Playing "));
    Serial.print(teachCount);
    Serial.println(F(" waypoints..."));

    for (int i = 0; i < teachCount; i++) {
        Serial.print(F("Waypoint "));
        Serial.println(i + 1);

        ArmPart *parts[] = {&arm.base, &arm.shoulder, &arm.elbow,
                            &arm.wrist, &arm.hand, &arm.gripper};
        int targets[] = {
            teachWaypoints[i].angles[0],
            teachWaypoints[i].angles[1],
            teachWaypoints[i].angles[2],
            teachWaypoints[i].angles[3],
            teachWaypoints[i].angles[4],
            teachWaypoints[i].angles[5]
        };
        moveJointsSynchronized(parts, targets, 6);
        delay(500);
    }

    Serial.println(F("Playback complete."));
}

// Exit teach mode: re-enable torque on bus servos
void teachStop() {
    if (arm.elbow.type == SERVO_BUS) busServoSetTorque(arm.elbow.pin, true);
    if (arm.wrist.type == SERVO_BUS) busServoSetTorque(arm.wrist.pin, true);
    if (arm.hand.type == SERVO_BUS) busServoSetTorque(arm.hand.pin, true);

    teachMode = false;

    Serial.print(F("Teach mode off. "));
    Serial.print(teachCount);
    Serial.println(F(" waypoints stored. Send 'teach play' to replay."));
}
