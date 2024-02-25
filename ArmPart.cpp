#include "ArmPart.h"

ArmPart::ArmPart(String name, int pin, int minAngle, int maxAngle, int defaultAngle):
    _name(name),
    _pin(pin),
    _minAngle(minAngle),
    _maxAngle(maxAngle),
    _defaultAngle(defaultAngle) {
    // Constructor body, potentially empty if initialization list is used
}

void ArmPart::attach() {
    _servo.attach(_pin);
    moveTo(_servo, _defaultAngle);
}

void moveTo(part, int targetAngle) {
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