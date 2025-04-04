#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <ESP32Servo.h>
#include <CodeCell.h>

// Define pins
#define L_DRIVE_PIN 5
#define R_DRIVE_PIN 6
#define WPN_PIN 7

// Shorthand for the Xbox controller
#define A xboxNotif.btnA
#define B xboxNotif.btnB
#define X xboxNotif.btnX
#define Y xboxNotif.btnY
#define L1 xboxNotif.btnLB
#define R1 xboxNotif.btnRB
#define L2 xboxNotif.trigLT
#define R2 xboxNotif.trigRT
#define axisX xboxNotif.joyLHori
#define axisY xboxNotif.joyLVert
#define axisRX xboxNotif.joyRHori
#define axisRY xboxNotif.joyRVert
#define btnShare xboxNotif.btnShare
#define btnSelect xboxNotif.btnSelect

// Define more buttons here if needed...

// Controller Limits
#define TRIGGER_MIN 0
#define TRIGGER_MAX 1023
#define STICK_MIN 0
#define STICK_MID 32767
#define STICK_MAX 65534
#define STICK_DEADZONE 4096

// Weapon, throttle and steering limits
#define WPN_MIN -100.0  // Full speed in reverse
#define WPN_OFF 0.0     // Turn off weapon
#define WPN_LOW 25.0    // Low speed for maneuveing
#define WPN_MID 50.0    // Mid speed for max damage
#define WPN_HIGH 75.0   // High speed for charging
#define WPN_MAX 100.0   // Full speed for weapon-on-weapon hits
#define THROTTLE_MIN -100.0
#define THROTTLE_OFF 0.0
#define THROTTLE_MAX 100.0
#define STEER_MIN -100.0
#define STEER_OFF 0.0
#define STEER_MAX 100.0

// PWM Settings
#define PWM_MIN 1000
#define PWM_MID 1500
#define PWM_MAX 2000

// Robot states
#define COMBAT_MODE 0
#define PAIRING_MODE 1
#define FLASHING_MODE 2
#define ASSISTED_COMBAT_MODE 3

// Tuning
#define DRIVE_SPEED_MODIFIER 1.0    // [0, 1]
#define TURN_SPEED_MODIFIER 0.5     // [0, 1]
#define UPDATE_DELAY 10             // Unit: ms

// Global variables :)
int weaponSpeed = WPN_OFF;
int weaponIdleSpeed = WPN_OFF;
int throttle = THROTTLE_OFF;
int steer = STEER_OFF;

bool sharePressHandled = false;
bool selectPressHandled = false;

int mode;
float gyroX = 0.0;    // Unit: rad/s
float gyroY = 0.0;    // Unit: rad/s
float gyroZ = 0.0;    // Unit: rad/s, this is the one we are interested in for turning the robot


Servo lDriveESC;
Servo rDriveESC;
Servo wpnESC;

CodeCell myCodeCell;
XboxSeriesXControllerESP32_asukiaaa::Core ctl;



void dumpGamepad() {
    Serial.println("Address: " + ctl.buildDeviceAddressStr());
    Serial.println("battery " + String(ctl.battery) + "%");
    Serial.print(ctl.xboxNotif.toString());
}

void processGamepad() {
    // Select weapon idle speed. B=off, A=low, X=mid, Y=max
    if (ctl.B) {
        Serial.println("(B) Weapon Off");
        weaponIdleSpeed = WPN_OFF;
    } else if (ctl.A) {
        Serial.println("(A) Weapon Low");
        weaponIdleSpeed = WPN_LOW;
    } else if (ctl.X) {
        Serial.println("(X) Weapon Mid");
        weaponIdleSpeed = WPN_MID;
    } else if (ctl.Y) {
        Serial.println("(Y) Weapon High");
        weaponIdleSpeed = WPN_HIGH;
    }

    // Set weapon speed
    if (ctl.R1) {
        Serial.println("(R1) Overriding weapon idle speed");
        weaponSpeed = WPN_MAX; // Full forward weapon
    } else if (ctl.L1) {
        Serial.println("(L1) Overriding weapon idle speed (reverse)");
        weaponSpeed = WPN_MIN; // Full reverse weapon
    } else {
        weaponSpeed = weaponIdleSpeed; // No weapon
    }

    // Set throttle (I don't think deadzone is needed here)
    throttle = map(ctl.R2-ctl.L2, -TRIGGER_MAX, TRIGGER_MAX, THROTTLE_MIN, THROTTLE_MAX);

    // Set steering
    if (abs(ctl.axisX-STICK_MID) > STICK_DEADZONE) {
        steer = map(ctl.axisX, STICK_MIN, STICK_MAX, STEER_MIN, STEER_MAX);
    } else {
        steer = STEER_OFF;
    }

    // Flash mode toggle
    if (ctl.btnShare) {
        if (!sharePressHandled) {
            if (mode != FLASHING_MODE) {
                setMode(FLASHING_MODE);
            } else {
                setMode(COMBAT_MODE);
            }
        }
        sharePressHandled = true;
    } else {
        sharePressHandled = false;
    }

    // Assistance toggle
    if (ctl.btnSelect) {
        if (!selectPressHandled) {
            if (mode == COMBAT_MODE) {
                setMode(ASSISTED_COMBAT_MODE);
            } else if (mode == ASSISTED_COMBAT_MODE) {
                setMode(COMBAT_MODE);
            }
        }
        selectPressHandled = true;
    } else {
        selectPressHandled = false;
    }


    Serial.println("(INPUT) Throttle: " + String(throttle) + ", Steer: " + String(steer) + ", Weapon: " + String(weaponSpeed));
}

void readSensors() {
    myCodeCell.PrintSensors();
    myCodeCell.Motion_GyroRead(gyroX, gyroY, gyroZ);
    Serial.println("(SENSING) gyroX: " + String(gyroX) + ", gyroY: " + String(gyroY) + ", gyroZ: " + String(gyroZ));
}

void applyPWM() {
    // Map throttle and steering to PWM values
    float mix_L = DRIVE_SPEED_MODIFIER*throttle - TURN_SPEED_MODIFIER*(steer-STEER_OFF);
    float mix_R = DRIVE_SPEED_MODIFIER*throttle + TURN_SPEED_MODIFIER*(steer-STEER_OFF);

    int leftPWM = map(mix_L, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX);
    int rightPWM = map(mix_R, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX);
    int weaponPWM = map(weaponSpeed, WPN_MIN, WPN_MAX, PWM_MIN, PWM_MAX);

    // Constrain PWM values
    leftPWM = constrain(leftPWM, PWM_MIN, PWM_MAX);
    rightPWM = constrain(rightPWM, PWM_MIN, PWM_MAX);
    weaponPWM = constrain(weaponPWM, PWM_MIN, PWM_MAX);

    // Write PWM values to ESCs
    lDriveESC.writeMicroseconds(leftPWM);
    rDriveESC.writeMicroseconds(rightPWM);
    wpnESC.writeMicroseconds(weaponPWM);

    Serial.println("(OUTPUT) leftPWM: " + String(leftPWM) + ", rightPWM: " + String(rightPWM) + ", weaponPWM: " + String(weaponPWM));
}

void attachMotorPins() {
    // Attach "servos"
    lDriveESC.attach(L_DRIVE_PIN, PWM_MIN, PWM_MAX);
    rDriveESC.attach(R_DRIVE_PIN, PWM_MIN, PWM_MAX);
    wpnESC.attach(WPN_PIN, PWM_MIN, PWM_MAX);
}

void detachMotorPins() {
    // Detach "servos"
    lDriveESC.detach();
    rDriveESC.detach();
    wpnESC.detach();

    // Set pins to input
    pinMode(L_DRIVE_PIN, INPUT);
    pinMode(R_DRIVE_PIN, INPUT);
    pinMode(WPN_PIN, INPUT);
}

void setMode(int newMode) {
    if (mode == newMode) {
      return;
    }

    mode = newMode;
    if (mode == COMBAT_MODE) {
        Serial.println("Entered combat mode");
        myCodeCell.LED(0, 255, 255);    // Cyan
        attachMotorPins();
    } else if (mode == ASSISTED_COMBAT_MODE) {
        Serial.println("Entered assisted combat mode");
        myCodeCell.LED(255, 0, 255);    // Magenta
        attachMotorPins();
    } else if (mode == PAIRING_MODE) {
        Serial.println("Entered pairing mode");
        myCodeCell.LED(255, 0, 0);      // Red
        weaponIdleSpeed = WPN_OFF; // Change weapon state to off upon restart.
        
        // Stop all motors
        lDriveESC.writeMicroseconds(PWM_MID);
        rDriveESC.writeMicroseconds(PWM_MID);
        wpnESC.writeMicroseconds(PWM_MID);
    } else if (mode == FLASHING_MODE) {
        Serial.println("Entered flashing mode");
        myCodeCell.LED(0, 255, 0);      // Green
        detachMotorPins();
    } else {
        Serial.println("Unknown mode! Defaulting to combat mode...");
        setMode(COMBAT_MODE);   // Default to combat mode
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");
    ctl.begin();

    myCodeCell.Init(MOTION_GYRO);
    myCodeCell.LED(255, 255, 255);

    setMode(PAIRING_MODE);
}

void loop() {
    ctl.onLoop();
    if (ctl.isConnected()) {
        if (ctl.isWaitingForFirstNotification()) {
            Serial.println("waiting for first notification");
        } else {
            if (mode == PAIRING_MODE) {
                setMode(COMBAT_MODE);
            }

            // Check whether in flash or control mode
            dumpGamepad();
            processGamepad();

            if (mode == COMBAT_MODE) {
                // TODO: Break up into elevonMixing() and applyPWM()
                applyPWM();
            } else if (mode == ASSISTED_COMBAT_MODE) {
                // readSensors();
                // TODO: Break up into elevonMixing() and applyPWM()
                applyPWM();
            }
        }
    } else {
        Serial.println("not connected");
        setMode(PAIRING_MODE);

        // Restart ESP to re-pair
        if (ctl.getCountFailedConnection() > 2) {
            ESP.restart();
        }
    }

    delay(UPDATE_DELAY);
}

// TODOS:
// - Run loop with myCodeCell.Run(UPDATE_FREQUENCY) instead
//   - Fix LED resetting to blue breathing when using myCodeCell.Run()
// - Break applyPWM() into elevonMixing()/PIDControl() and applyPWM()
// - Add gyro stabilization