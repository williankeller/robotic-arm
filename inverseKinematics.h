
const int baseHeight = 81;
const int armLength1 = 104; // Shoulder to elbow
const int armLength2 = 96; // Elbow to wrist
const int gripLengthOpen = 58;

// Function to calculate the base angle and radial distance
void SolveXYZ(float x, float y, float &baseAngle, float &r) {
    baseAngle = atan2(y, x); // atan2 handles x=0
    r = sqrt(x * x + y * y);
}

// Function to calculate the shoulder, elbow, and wrist angles
void SolveRZ(float r, float z, float gripAngle, float &shoulderAngle, float &elbowAngle, float &wristAngle) {
    float r_prime = r - (sin(gripAngle) * gripLengthOpen);
    float z_prime = z - baseHeight + (cos(gripAngle) * gripLengthOpen);
    float h = sqrt(r_prime * r_prime + z_prime * z_prime) / 2;
    elbowAngle = asin(h / armLength1) * 2;
    shoulderAngle = atan2(z_prime, r_prime) + ((PI - elbowAngle) / 2);
    wristAngle = PI + gripAngle - shoulderAngle - elbowAngle;
}

// Function to move the arm to a desired position
void moveToPosition(float x, float y, float z, float gripAngle) {
    float baseAngle, r, shoulderAngle, elbowAngle, wristAngle;
    SolveXYZ(x, y, baseAngle, r);
    SolveRZ(r, z, gripAngle, shoulderAngle, elbowAngle, wristAngle);
    
    // Convert radians to degrees for servo positioning
    moveBase(degrees(baseAngle));
    moveShoulder(degrees(shoulderAngle));
    moveElbow(degrees(elbowAngle));
    moveWrist(degrees(wristAngle));
}
