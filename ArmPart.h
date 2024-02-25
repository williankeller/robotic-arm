#ifndef ArmPart_h
#define ArmPart_h

#include <Servo.h>

class ArmPart {
public:
    ArmPart(int pin, int minAngle, int maxAngle, int defaultAngle);
    void attach();
    void moveTo(int angle);

private:
    String _name;
     int _pin;
    int _minAngle, _maxAngle, _defaultAngle;
    Servo _servo;
};

#endif
