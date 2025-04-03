#include <Bluepad32.h>

// Failsafe settings
#define FAILSAFE_TIMEOUT 1000 // 1 second

// Controller Limits
#define TRIGGER_DEADZONE 10
#define TRIGGER_MIN 0
#define TRIGGER_MAX 1023
#define STICK_DEADZONE 10
#define STICK_MIN -511
#define STICK_MAX 512

// Weapon Settings
#define WPN_MAX_REVERSE -511
#define WPN_OFF 0
#define WPN_LOW 128
#define WPN_MID 256
#define WPN_HIGH 384
#define WPN_MAX 512

// Throttle Settings
#define THROTTLE_MIN -511
#define THROTTLE_OFF 0
#define THROTTLE_MAX 512

// Steering Settings
#define STEER_MIN -511
#define STEER_OFF 0
#define STEER_MAX 512


// Global variables :)
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

int weaponSpeed = WPN_OFF;
int weaponIdleSpeed = WPN_OFF;
int throttle = THROTTLE_OFF;
int steer = STEER_OFF;


// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    // Select weapon idle speed. B=off, A=low, X=mid, Y=max
    if (ctl->b()) {
        weaponIdleSpeed = WPN_OFF;
    } else if (ctl->a()) {
        weaponIdleSpeed = WPN_LOW;
    } else if (ctl->x()) {
        weaponIdleSpeed = WPN_MID;
    } else if (ctl->y()) {
        weaponIdleSpeed = WPN_MAX;
    }

    // Set weapon speed
    if (ctl->r1()) {
        weaponSpeed = WPN_MAX; // Full forward weapon
    } else if (ctl->l1()) {
        weaponSpeed = WPN_MAX_REVERSE; // Full reverse weapon
    } else {
        weaponSpeed = weaponIdleSpeed; // No weapon
    }

    // Set throttle
    if (abs(ctl->throttle()()) > TRIGGER_DEADZONE) {
        throttle = map(ctl->throttle(), TRIGGER_MIN, TRIGGER_MAX, THROTTLE_MIN, THROTTLE_MAX);
    } else {
        throttle = THROTTLE_OFF;
    }

    // Set steering
    if (abs(ctl->axisX()) > STICK_DEADZONE) {
        steer = map(ctl->axisX(), STICK_MIN, STICK_MAX, STEER_MIN, STEER_MAX);
    } else {
        steer = STEER_OFF;
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void setup() {
    Serial.begin(9600);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    // BP32.forgetBluetoothKeys(); // Uncomment to rebind all gamepads.
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}
