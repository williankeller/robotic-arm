#ifndef RobotArm_h
#define RobotArm_h

#include "ArmPart.h"

class RobotArm {
public:
    RobotArm();
    void moveInitialPosition();
    void moveToPosition(float x, float y, float z, float gripAngle);
    // Other methods...
private:
    ArmPart base;
    ArmPart shoulder;
    ArmPart elbow;
    ArmPart wrist;
    ArmPart hand;
    ArmPart gripper;
    // Helper methods...
};

#endif
