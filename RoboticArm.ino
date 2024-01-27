#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the Adafruit_PWMServoDriver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo settings for DS3225MG
const int minPulseWidth = 500; // Minimum pulse width for DS3225MG in microseconds
const int maxPulseWidth = 2500; // Maximum pulse width for DS3225MG in microseconds
const int servoFreq = 50; // Servo frequency (50Hz for most servos)

// define arm part names and pins, example: const int base =[pin number];
// Define a structure for each part of the arm
struct ArmPart {
    String name;
    int pin;
    int minAngle;
    int maxAngle;
};

// Define a structure for the robot arm
struct RobotArm {
    ArmPart base;
    ArmPart shoulder;
    ArmPart elbow;
    ArmPart wrist;
    ArmPart gripper;
};

// Initialize the robot arm with specific values
RobotArm arm = {
    {"base",     0, 0, 180},   // base, pin, min, max
    {"shoulder", 1, 0, 180},   // shoulder, pin, min, max
    {"elbow",    2, 0, 180},   // elbow, pin, min, max
    {"wrist",    3, 0, 180},   // wrist, pin, min, max
    {"gripper",  5, 25, 155}   // gripper, pin, min, max
};


void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(servoFreq);  // Analog servos run at ~50 Hz updates
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

        if (data.startsWith("gripper")) {
            int angle = data.substring(8).toInt();
            moveGripper(angle);
        }

        if (data.startsWith("arm")) {
            int baseAngle = data.substring(4, 7).toInt();
            int shoulderAngle = data.substring(8, 11).toInt();
            int elbowAngle = data.substring(12, 15).toInt();
            int wristAngle = data.substring(16, 19).toInt();
            moveArm(baseAngle, shoulderAngle, elbowAngle, wristAngle);
        }
    }
}
// Shoulder min 40

// Function to set the servo angle
void setServoPosition(ArmPart part, int angle) {
    Serial.print("Pin: ");
    Serial.print(part.pin);
    Serial.print(": ");
    Serial.print(part.name);
    Serial.print(", Angle: ");
    Serial.print(angle);
    Serial.print("\n");

    angle = constrain(angle, part.minAngle, part.maxAngle);
    int pwmValue = angleToPulse(angle, part.minAngle, part.maxAngle);
    pwm.setPWM(part.pin, 0, pwmValue);
}

int angleToPulse (int angle, int minAngle, int maxAngle) {
    int pulseWidth = map(angle, minAngle, maxAngle, minPulseWidth, maxPulseWidth);
    int pwmValue = int(pulseWidth / 1000000.0 * servoFreq * 4096.0);

    // Check that the calculated PWM value is within the valid range for the PCA9685 board
    if (pwmValue < 0 || pwmValue > 4095) {
        Serial.println("Error: Invalid PWM value for PCA9685 board");
        return 0;
    }
    return pwmValue;
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

void moveGripper(int angle) {
    setServoPosition(arm.gripper, angle);
}

void moveArm(int baseAngle, int shoulderAngle, int elbowAngle, int wristAngle) {
    moveBase(baseAngle);
    moveShoulder(shoulderAngle);
    moveElbow(elbowAngle);
    moveWrist(wristAngle);
}
