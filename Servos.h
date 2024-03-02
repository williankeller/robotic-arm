#include "ArmParts.h"

void attachServos() {
    // Attach each servo to its corresponding pin
    arm.base.servo.attach(arm.base.pin);
    arm.base.servo.write(arm.base.defaultAngle);

    arm.shoulder.servo.attach(arm.shoulder.pin);
    arm.shoulder.servo.write(arm.shoulder.defaultAngle);

    arm.elbow.servo.attach(arm.elbow.pin);
    arm.elbow.servo.write(arm.elbow.defaultAngle);

    arm.wrist.servo.attach(arm.wrist.pin);
    arm.wrist.servo.write(arm.wrist.defaultAngle);

    arm.hand.servo.attach(arm.hand.pin);
    arm.hand.servo.write(arm.hand.defaultAngle);

    arm.gripper.servo.attach(arm.gripper.pin);
    arm.gripper.servo.write(arm.gripper.defaultAngle);
}

// Function to set the servo angle
void setServoPosition(ArmPart &part, int targetAngle) {
    targetAngle = constrain(targetAngle, part.minAngle, part.maxAngle); // Constrain the target angle within limits

    // Read the current position of the servo
    int currentAngle = part.servo.read();

    // Define the step size for smoother movement
    int stepSize = 1;

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
    // For debugging
    Serial.print("Pin: ");
    Serial.print(part.pin);
    Serial.print(": ");
    Serial.print(part.name);
    Serial.print(", Current: ");
    Serial.print(currentAngle);
    Serial.print(", Angle: ");
    Serial.println(targetAngle);
}
