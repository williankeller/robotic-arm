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
