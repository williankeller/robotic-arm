#include <Wire.h>
#include <Servo.h>

// Define a structure for each part of the arm
struct ArmPart {
    String name;
    int pin;
    int minAngle;
    int maxAngle;
    Servo servo;  // Servo object for each part
};

// Define a structure for the robot arm
struct RobotArm {
    ArmPart base;       // Base - Turns arm left to right
    ArmPart shoulder;   // Shoulder - First part of the arm
    ArmPart elbow;      // Elbow - Second junction of the arm
    ArmPart wrist;      // Wrist - Third junction of the arm
    ArmPart hand;       // Hand - Rotates the wrist 180 degrees
    ArmPart gripper;    // Gripper - Grasps objects
};

// Initialize the robot arm with specific values
RobotArm arm = {
    {"base",     3, 30, 150}, // base, pin, min, max
    {"shoulder", 5, 0, 180},  // shoulder, pin, min, max
    {"elbow",    6, 0, 180},  // elbow, pin, min, max
    {"wrist",    9, 0, 180},  // wrist, pin, min, max
    {"hand",     10, 0, 180}, // hand, pin, min, max
    {"gripper",  11, 0, 98}   // gripper, pin, min, max
};


void setup() {
    Serial.begin(9600);

    // Attach each servo to its corresponding pin
    arm.base.servo.attach(arm.base.pin);
    arm.shoulder.servo.attach(arm.shoulder.pin);
    arm.elbow.servo.attach(arm.elbow.pin);
    arm.wrist.servo.attach(arm.wrist.pin);
    arm.hand.servo.attach(arm.hand.pin);
    arm.gripper.servo.attach(arm.gripper.pin);

    // Move the arm to its initial position
    moveInitialPosition();
    openGripper();
}

void loop() {

    // Check if data is available to read from the serial buffer
    if (Serial.available() > 0) {
        // Read the incoming string until a newline is received
        String data = Serial.readStringUntil('\n');

        if (data.startsWith("base")) {
            int angle = data.substring(5).toInt();
            moveBase(angle);
        }

        if (data.startsWith("shoulder")) {
            int angle = data.substring(9).toInt();
            moveShoulder(angle);
        }

        if (data.startsWith("elbow")) {
            int angle = data.substring(6).toInt();
            moveElbow(angle);
        }

        if (data.startsWith("wrist")) {
            int angle = data.substring(6).toInt();
            moveWrist(angle);
        }

        if (data.startsWith("hand")) {
            int angle = data.substring(5).toInt();
            moveHand(angle);
        }

        if (data.startsWith("gripper")) {
            int value = data.substring(8).toInt();
            if (value == 1) {
                closeGripper();
            } else {
                openGripper();
            }
        }

        if (data.startsWith("grab")) {
            moveShoulder(90);//90
            moveElbow(130);
            moveWrist(60);
            delay(200);
            openGripper();
            moveShoulder(40);
            moveWrist(70);
            moveElbow(150);
            delay(500);
            closeGripper();
            delay(500);
            moveInitialPosition();
            delay(300);
            moveBase(60);
            moveShoulder(100);
            moveWrist(30);
            openGripper();// open gripper
            delay(1000);
            moveInitialPosition();
        }

        if (data.startsWith("invert")) {
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
            openGripper();// open gripper
            delay(1000);
            moveInitialPosition();
        }

        if (data.startsWith("circle")) {
            drawCircle();
        }

        if (data.startsWith("square")) {
            drawSquare();
        }
    }
}

// Function to set the servo angle
void setServoPosition(ArmPart &part, int targetAngle) {
    targetAngle = constrain(targetAngle, part.minAngle, part.maxAngle); // Constrain the target angle within limits

    int currentAngle = part.servo.read(); // Read the current position of the servo
    int stepSize = 1;  // Define the step size for smoother movement, can be adjusted for different servos

    if (currentAngle < targetAngle) {
        for (int pos = currentAngle; pos < targetAngle; pos += stepSize) {
            part.servo.write(pos); // Move to the next position
            delay(15); // Wait for 15ms to slow down the movement
        }
    } else {
        for (int pos = currentAngle; pos > targetAngle; pos -= stepSize) {
            part.servo.write(pos); // Move to the next position
            delay(15); // Wait for 15ms to slow down the movement
        }
    }

    Serial.print("Pin: ");
    Serial.print(part.pin);
    Serial.print(": ");
    Serial.print(part.name);
    Serial.print(", Current: ");
    Serial.print(currentAngle);
    Serial.print(", Angle: ");
    Serial.println(targetAngle);
}

void moveInitialPosition() {
    moveBase(90);
    moveShoulder(130);
    moveElbow(160);
    moveWrist(90);
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

void closeGripper() {
    setServoPosition(arm.gripper, 180);
}

void openGripper() {
    setServoPosition(arm.gripper, 0);
}

void moveArm(int baseAngle, int shoulderAngle, int elbowAngle, int wristAngle, int handAngle) {
    moveBase(baseAngle);
    moveShoulder(shoulderAngle);
    moveElbow(elbowAngle);
    moveWrist(wristAngle);
    moveHand(handAngle);
}

void drawCircle() {

    moveBase(90); 
    moveElbow(130);
    moveShoulder(40);
    closeGripper(); 
    
    for (int i = 0; i < 360; i += 10) {
        moveBase(100 + sin(i * PI / 180) * 20);
        moveShoulder(50 + cos(i * PI / 180) * 5);
    }
}

void drawSquare() {
  
  int baseCenter = 90;  // Center position for the base
  int shoulderCenter = 70;  // Center position for the shoulder
  int sideLength = 30;  // Length of each side of the square

  // Move to the starting corner of the square
  moveBase(baseCenter - sideLength / 2);
  moveShoulder(shoulderCenter - sideLength / 2);
  closeGripper();

  // Draw all four sides of the square, alternating horizontal and vertical lines
  for (int i = 0; i < 4; i++) {
    
    if (i % 2 == 0) {
      // Horizontal side
      for (int j = 0; j < sideLength; j++) {
        moveBase(baseCenter - sideLength / 2 + j);
        delay(50); // Adjust delay for drawing speed
      }
    } else {
      // Vertical side
      for (int j = 0; j < sideLength; j++) {
        moveShoulder(shoulderCenter - sideLength / 2 + j);
        delay(50); // Adjust delay for drawing speed
      }
    }

    // Turn 90 degrees to start the next side
    moveShoulder(shoulderCenter + sideLength / 2);  // This ensures both horizontal and vertical turns
    delay(500);
  }
  moveInitialPosition();
}
