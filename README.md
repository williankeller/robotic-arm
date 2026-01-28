# Robotic Arm (6 DOF)

An Arduino-based 6 degrees-of-freedom robotic arm with serial command control, inverse kinematics positioning, and optional HuskyLens vision tracking.

## Serial Commands

Connect via Serial Monitor at **9600 baud** with **Newline** line ending. Send `help` to list all commands.

### Joint Control

| Command | Description | Range |
|---------|-------------|-------|
| `base <angle>` | Rotate base | 35–150° |
| `shoulder <angle>` | Move shoulder | 0–180° |
| `elbow <angle>` | Move elbow | 0–140° |
| `wrist <angle>` | Move wrist | 89–180° |
| `hand <angle>` | Rotate hand | 0–180° |
| `gripper <0\|1\|angle>` | 0 = open, 1 = close, or specific angle | 20–90° |

### Sequences & Positioning

| Command | Description |
|---------|-------------|
| `reset` | Return all joints to home position |
| `grab` | Run full pick-and-place sequence |
| `position x,y,z,gripAngle` | Move to Cartesian position using inverse kinematics (mm, radians) |

### Teach Mode

Record arm positions by physically moving the bus servos (elbow, wrist, hand) and replay them. PWM servos (base, shoulder, gripper) are controlled via serial commands during teach mode.

| Command | Description |
|---------|-------------|
| `teach on` | Enter teach mode (disables torque on bus servos so they can be moved by hand) |
| `teach capture` | Save current arm position as a waypoint (max 20) |
| `teach play` | Replay all recorded waypoints with synchronized movement |
| `teach off` | Exit teach mode (re-enables torque on bus servos) |
| `teach` | Show teach mode status and waypoint count |

### Vision Tracking

HuskyLens is **optional**. The arm starts in manual mode and accepts serial commands without HuskyLens connected. Use the `tracking` command to enable auto tracking at runtime.

| Command | Description |
|---------|-------------|
| `tracking on` | Enable HuskyLens auto tracking (retries connection if needed) |
| `tracking off` | Disable tracking and return to home position |
| `tracking` | Show current tracking status |

## Wiring

### Servo Pin Mapping (Arduino Uno)

The arm uses a mix of standard PWM servos and Hiwonder LX-1501 bus servos.

**PWM Servos** (controlled via Arduino PWM pins):

| Servo | Digital Pin | Default Angle | Range |
|-------|-------------|---------------|-------|
| Base | 3 | 90° | 35–150° |
| Shoulder | 5 | 140° | 0–180° |
| Gripper | 11 | 20° | 20–90° |

**Bus Servos — Hiwonder LX-1501** (controlled via half-duplex serial):

| Servo | Bus ID | Default Angle | Range |
|-------|--------|---------------|-------|
| Elbow | 1 | 100° | 0–140° |
| Wrist | 2 | 135° | 89–180° |
| Hand | 3 | 90° | 0–180° |

**LX-1501 Specifications:**

| Parameter | Value |
|-----------|-------|
| Working voltage | 6–8.4V |
| Torque | 17 kg·cm (at 7.4V) |
| Speed | 0.16 sec/60° (at 7.4V) |
| Rotation range | 0–240° (mapped to 0–1000 position units) |
| Accuracy | 0.3° |
| No-load current | 100 mA |
| Stall current | 2.4–3 A |
| Communication | UART 115200 baud, servo ID 0–253 (default 1) |
| Feedback | temperature, voltage, position, angle |
| Connector | PH2.0-3P (20 cm default wire) |
| Size / Weight | 54.4 × 20.0 × 45.5 mm / 58 g |

### Bus Servo Wiring

The bus servos use the Hiwonder LX serial protocol (half-duplex UART at 115200 baud). A **BusLinker** board bridges the Arduino's full-duplex serial to the servo's single-wire bus.

```
Arduino Pin 7 (TX)  --> BusLinker RX
Arduino Pin 6 (RX)  <-- BusLinker TX
Arduino GND --------- BusLinker GND
BusLinker Vin <------ 6–8.4V power supply
BusLinker Servo Interface --> Servo 1 --> Servo 2 --> Servo 3
                              (PH2.0/3P daisy-chain cables)
```

- The BusLinker handles half-duplex conversion — no resistor needed
- All three bus servos are daisy-chained via the PH2.0/3P pass-through connectors on each servo
- Each servo must have a unique ID (1, 2, 3) set via the Hiwonder BusLinker software (Bus Servo Terminal) before wiring
- Bus servos support position readback and torque enable/disable (used for teach mode)
- Use the **Hiwonder BusLinker** debug board — it has matching PH2.0/3P connectors and the Bus Servo Terminal software for configuring servo IDs

### Power

- **PWM Servos (MG996R):** 6V DC from an external power supply, current limit at least 10A
- **Bus Servos (LX-1501) + BusLinker:** 6–8.4V from the same external supply into the BusLinker Vin terminal (powers both the BusLinker and the daisy-chained servos)
- **Arduino:** Powered via USB (separate from servo power)
- Connect the power supply GND to both the Arduino GND and the BusLinker GND (common ground)
- Do not power servos from the Arduino 5V pin

### HuskyLens (Optional)

Connected via I2C (SDA/SCL pins). Set Protocol Type to I2C in HuskyLens settings.

## Arm Geometry

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
