
#include <Servo.h>

// Create servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;

// Servo pin connections
const int baseServoPin = 3;
const int shoulderServoPin = 5;
const int elbowServoPin = 6;
const int wristServoPin = 9;

// Function to convert degrees to microseconds for the servo pulse width
int angleToMicroseconds(int angle) {
  return map(angle, 0, 180, 544, 2400); // Values for standard servos
}

// Function to write angles to the servos
void writeServoAngles(float base, float shoulder, float elbow, float wrist) {
  baseServo.writeMicroseconds(angleToMicroseconds((int)base));
  shoulderServo.writeMicroseconds(angleToMicroseconds((int)shoulder));
  elbowServo.writeMicroseconds(angleToMicroseconds((int)elbow));
  wristServo.writeMicroseconds(angleToMicroseconds((int)wrist));
}

// Inverse kinematics calculations (from the previous Python function)
// Note: The following code will need to be adapted into proper C++ syntax
void inverseKinematics(float x, float y, float z, float* angles) {
  // Arm dimensions in mm
  const float baseHeight = 80;
  const float upperArmLength = 104;
  const float lowerArmLength = 89;
  const float gripLength = 175;

  // Calculate the base angle and wrist center
  float baseAngle = atan2(y, x);
  float wristCenterX = x - gripLength * cos(baseAngle);
  float wristCenterY = y - gripLength * sin(baseAngle);
  float wristCenterZ = z - baseHeight;

  // Calculate the arm extension (distance from shoulder to wrist center)
  float armExtension = sqrt(wristCenterX*wristCenterX + wristCenterY*wristCenterY + wristCenterZ*wristCenterZ);

  // Calculate the shoulder angle
  float angleA = acos((upperArmLength*upperArmLength + armExtension*armExtension - lowerArmLength*lowerArmLength) / (2 * upperArmLength * armExtension));
  float angleB = atan2(wristCenterZ, sqrt(wristCenterX*wristCenterX + wristCenterY*wristCenterY));
  float shoulderAngle = angleA + angleB;

  // Calculate the elbow angle
  float elbowAngle = PI - acos((upperArmLength*upperArmLength + lowerArmLength*lowerArmLength - armExtension*armExtension) / (2 * upperArmLength * lowerArmLength));

  // Calculate the wrist angle
  float wristAngle = PI/2 - (shoulderAngle + elbowAngle);

  // Convert angles from radians to degrees
  angles[0] = degrees(baseAngle);
  angles[1] = degrees(shoulderAngle);
  angles[2] = degrees(elbowAngle);
  angles[3] = degrees(wristAngle);
}

void setup() {
  // Attach the servos to their respective pins
  baseServo.attach(baseServoPin);
  shoulderServo.attach(shoulderServoPin);
  elbowServo.attach(elbowServoPin);
  wristServo.attach(wristServoPin);
}

void loop() {
  // Example desired position (x, y, z) in mm
  float x = 100;
  float y = 100;
  float z = 100;
  
  // Array to hold calculated servo angles
  float angles[4];
  
  // Perform inverse kinematics calculation
  inverseKinematics(x, y, z, angles);
  
  // Write the calculated angles to the servos
  writeServoAngles(angles[0], angles[1], angles[2], angles[3]);
  
  // Small delay before the next calculation
  delay(2000);
}
