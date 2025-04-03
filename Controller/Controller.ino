//#include <Bluepad32.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Define pins
#define L_DRIVE_PIN 7
#define R_DRIVE_PIN 8
#define WPN_PIN 6

// Failsafe settings
#define FAILSAFE_TIMEOUT 1000 // 1 second

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
// Define more buttons here if needed...

// Controller Limits
#define TRIGGER_MIN 0
#define TRIGGER_MAX 1023
// #define TRIGGER_DEADZONE 16
#define STICK_MIN 0
#define STICK_MID 32767
#define STICK_MAX 65534
#define STICK_DEADZONE 4096

// Weapon Settings
#define WPN_MIN 0   // Full speed in reverse
#define WPN_OFF 127 // Turn off weapon
#define WPN_LOW 159 // Low speed for maneuveing
#define WPN_MID 191 // Mid speed for max damage
#define WPN_MAX 255 // Full speed for weapon-on-weapon hits

// Throttle Settings
#define THROTTLE_MIN 0
#define THROTTLE_OFF 127
#define THROTTLE_MAX 255

// Steering Settings
#define STEER_MIN 0
#define STEER_OFF 127
#define STEER_MAX 255


// Global variables :)
int weaponSpeed = WPN_OFF;
int weaponIdleSpeed = WPN_OFF;
int throttle = THROTTLE_OFF;
int steer = STEER_OFF;

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
        Serial.println("(Y) Weapon Max");
        weaponIdleSpeed = WPN_MAX;
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
    Serial.println("Deadzone check: " + String(ctl.axisX-STEER_OFF));
    if (abs(ctl.axisX-STICK_MID) > STICK_DEADZONE) {
        steer = map(ctl.axisX, STICK_MIN, STICK_MAX, STEER_MIN, STEER_MAX);
    } else {
        steer = STEER_OFF;
    }

    Serial.println("Throttle: " + String(throttle) + ", Steer: " + String(steer) + ", Weapon: " + String(weaponSpeed));
}

void applyPWM() {

    float mix_L = (throttle) - (steer-STEER_OFF);
    float mix_R = (throttle) + (steer-STEER_OFF);

    int leftPWM = constrain(map(mix_L, THROTTLE_MIN, THROTTLE_MAX, 0, 255), 0, 255);
    int rightPWM = constrain(map(mix_R, THROTTLE_MIN, THROTTLE_MAX, 0, 255), 0, 255);
    int weaponPWM = constrain(map(weaponSpeed, WPN_MIN, WPN_MAX, 0, 255), 0, 255);

    analogWrite(L_DRIVE_PIN, leftPWM);
    analogWrite(R_DRIVE_PIN, rightPWM);
    analogWrite(WPN_PIN, weaponPWM);
    
    Serial.println("leftPWM: " + String(leftPWM));
    Serial.println("rightPWM: " + String(rightPWM));
    Serial.println("WeaponPWM: " + String(weaponPWM));
}

void enterFailsafe() {
  Serial.println("ENTERING FAILSAFE!");
  analogWrite(L_DRIVE_PIN, 127);
  analogWrite(R_DRIVE_PIN, 127);
  analogWrite(WPN_PIN, 127);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");

    ctl.begin();

    pinMode(L_DRIVE_PIN, OUTPUT);
    pinMode(R_DRIVE_PIN, OUTPUT);
    pinMode(WPN_PIN, OUTPUT);
}

void loop() {
    ctl.onLoop();
    if (ctl.isConnected()) {
        if (ctl.isWaitingForFirstNotification()) {
            Serial.println("waiting for first notification");
        } else {
            // Run code here
            dumpGamepad();
            processGamepad();
            applyPWM();
        }
    } else {
        enterFailsafe();
        Serial.println("not connected");
        if (ctl.getCountFailedConnection() > 2) {
            ESP.restart();
        }
    }
    Serial.println("at " + String(millis()));

    //     vTaskDelay(1);
    delay(150);
}
