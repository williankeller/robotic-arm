# Robotic Arm (6 DOF)

- The height of the shoulder joint is **81mm** _(The base turns 180° on its axis left to right - 90° is the center)_;
- The length of the shoulder joint _(shoulder joint to elbow joint)_ is **104mm**;
- The length of the elbow joint _(elbow joint to wrist joint)_ is **96mm**;
- The hand is held at **90°** _(i.e. the hand is horizontal when the lower arm is parallel to the `x`, `y` plane)_;
- The grip length _(wrist joint to grip point)_ open **58mm**.

The base servo has a range of motion from 15° to 165°. The robotic arm is aligned with the positive y-axis when the angle of the base servo is at 90°.

![Arm position](https://github.com/williankeller/robotic-arm/assets/2963928/8007479c-2229-4966-b62f-b4b53a711857")


The range of motion for the shoulder servo is from 15° to 165°. The upper arm _(from shoulder to elbow)_ is aligned with the positive z-axis when the angle of the shoulder servo is at 90°.

![Arm position](https://github.com/williankeller/robotic-arm/assets/2963928/1e3c4670-bb6c-415b-9343-9653edde1ead)

The range of motion for the elbow servo is from 0° to 160°.
The lower arm _(elbow to the wrist)_ is at right angles to the upper arm when the angle of the elbow servo is at 90°.
If the shoulder and elbow servos are both at 90°, the upper arm will be parallel with the horizontal `x`, `y` plane.

Here are the positions for the "point to the top" pose:
- Base servo angle: 90° _(when pointing forward "center")_;
- Shoulder servo angle: 90°;
- Elbow servo angle: 0°;
- Wrist servo angle: 0°;
- Hand servo angle: 90°.

The x, y Plane
Calculations in the `x, y` plane use coordinates `x`, `y`.
The `z` plane is positioned by the rotation of the base servo and passes through the points `0, 0, 0` and `x, y, 0`.
The distance from the point `0, 0, 0` to `x, y, 0` is the radial distance, `r`.

The base angle and the radial distance `r` is calculated in the `SolveXYZ` function as follows:

```
baseAngle = atan(y/x)
r = sqrt(x2 + y2)
```

### The z Plane
Calculations in the `z` plane use coordinates `r`, `z`.

### Grip Length
The grip mechanism uses a scissor action to open and close the gripper, so the grip length changes depending on the grip width _(the distance between the jaws of the grip mechanism)_.
To determine the relationship of length to width, the grip length was measured at various grip widths and the results were plotted.

### Grip Angle
There are many different solutions for the joint angles that can position the grip point at a desired `x, y, z` coordinate.
For this application, the grip angle _(the angle of the gripper from horizontal)_ is specified, to constrain the solution to a single result. 
This has the desired effect of knowing from what angle an object will be gripped. 
The calculations are performed in the `SolveRZ` function. 
Using the grip angle and grip length, the location of the wrist joint is determined, and the `r’`, `z’` values are calculated.

```
r’ = r - (sin(gripAngle) * gripLength)
z’ = z - baseHeight + (cos(gripAngle) * gripLength)
```

The elbow angle can now be determined as follows:

```
h = sqrt(z’2 * r’2) / 2
elbowAngle = asin(h / armLength) * 2
```

Knowing the elbow angle, the shoulder angle can be calculated.

```
shoulderAngle = atan2(z’ / r’) + ((PI - elbowAngle) / 2)
```

The wrist angle is then determined by the summing the other joint angles.

```
wristAngle = PI + gripAngle - shoulderAngle - elbowAngle
```

Arm geometry constants (in millimeters):

```
const float baseHeight = 81.0;
const float shoulderLength = 104.0;
const float elbowLength = 96.0;
const float gripperLengthOpen = 58.0;
```
