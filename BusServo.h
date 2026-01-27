#include <SoftwareSerial.h>

// Bus servo serial pins
// Wiring: TX (pin 7) through a 1K resistor to servo data line
//         RX (pin 6) directly to servo data line
//         All bus servos daisy-chained on the same data line
#define BUS_SERVO_RX_PIN 6
#define BUS_SERVO_TX_PIN 7

// Bus servo command IDs (Hiwonder LX protocol)
#define CMD_MOVE_TIME_WRITE    1
#define CMD_MOVE_TIME_READ     2
#define CMD_MOVE_STOP          12
#define CMD_POS_READ           28
#define CMD_LOAD_UNLOAD_WRITE  31

// Position range: 0-1000 maps to 0-240 degrees
#define BUS_POS_MAX   1000
#define BUS_DEG_MAX   240

SoftwareSerial busSerial(BUS_SERVO_RX_PIN, BUS_SERVO_TX_PIN);

void initBusServos() {
    busSerial.begin(115200);
}

// Send a command packet to a bus servo
void busServoWrite(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t paramLen) {
    uint8_t length = paramLen + 3;
    uint8_t buf[10];
    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = id;
    buf[3] = length;
    buf[4] = cmd;
    for (uint8_t i = 0; i < paramLen; i++) {
        buf[5 + i] = params[i];
    }
    // Checksum: ~(ID + Length + Cmd + Params)
    uint8_t cksum = 0;
    for (uint8_t i = 2; i < 5 + paramLen; i++) {
        cksum += buf[i];
    }
    buf[5 + paramLen] = ~cksum;

    busSerial.write(buf, 6 + paramLen);
    busSerial.flush();
}

// Move servo to position (0-1000) over time (0-30000 ms)
void busServoMove(uint8_t id, int position, int timeMs) {
    position = constrain(position, 0, BUS_POS_MAX);
    timeMs = constrain(timeMs, 0, 30000);
    uint8_t params[] = {
        (uint8_t)(position & 0xFF),
        (uint8_t)((position >> 8) & 0xFF),
        (uint8_t)(timeMs & 0xFF),
        (uint8_t)((timeMs >> 8) & 0xFF)
    };
    busServoWrite(id, CMD_MOVE_TIME_WRITE, params, 4);
}

// Stop servo movement
void busServoStop(uint8_t id) {
    busServoWrite(id, CMD_MOVE_STOP, NULL, 0);
}

// Enable or disable torque (true = hold position, false = free to move by hand)
void busServoSetTorque(uint8_t id, bool enabled) {
    uint8_t param = enabled ? 1 : 0;
    busServoWrite(id, CMD_LOAD_UNLOAD_WRITE, &param, 1);
}

// Read current position from servo. Returns 0-1000, or -1 on failure
int busServoReadPosition(uint8_t id) {
    // Flush any pending data
    while (busSerial.available()) busSerial.read();

    busServoWrite(id, CMD_POS_READ, NULL, 0);

    // Wait for response (8 bytes: header(2) + id + len + cmd + posLow + posHigh + checksum)
    unsigned long start = millis();
    while (busSerial.available() < 8 && millis() - start < 50) {
        // timeout after 50ms
    }

    if (busSerial.available() >= 8) {
        if (busSerial.read() != 0x55) return -1;
        if (busSerial.read() != 0x55) return -1;
        busSerial.read(); // id
        busSerial.read(); // length
        busSerial.read(); // cmd
        uint8_t posLow = busSerial.read();
        uint8_t posHigh = busSerial.read();
        busSerial.read(); // checksum
        return (posHigh << 8) | posLow;
    }
    return -1;
}

// Convert degrees (0-240) to bus servo position units (0-1000)
int degreesToBusPos(int degrees) {
    return map(constrain(degrees, 0, BUS_DEG_MAX), 0, BUS_DEG_MAX, 0, BUS_POS_MAX);
}

// Convert bus servo position units (0-1000) to degrees (0-240)
int busPosToDegrees(int position) {
    return map(constrain(position, 0, BUS_POS_MAX), 0, BUS_POS_MAX, 0, BUS_DEG_MAX);
}
