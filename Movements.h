#include "Servos.h"

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