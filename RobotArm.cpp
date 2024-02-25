#include "RobotArm.h"

RobotArm::RobotArm():
    base(9, 0, 180, 90),
    shoulder(10, 0, 180, 140),
    elbow(11, 0, 180, 100),
    wrist(6, 0, 180, 135),
    hand(5, 0, 180, 90),
    gripper(3, 0, 180, 0) {

        base.attach();
        shoulder.attach();
        elbow.attach();
        wrist.attach();
        hand.attach();
        gripper.attach();
}