//set partition scheme to Huge APP (3MB No OTA/1MB SPIFFS)

#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <WiFi.h>
#include <soc/gpio_reg.h>
#include <esp_bt.h>
#include <esp_bt_main.h>

// Constants and configuration
namespace Constants {
    const uint8_t IN1 = 26;
    const uint8_t IN2 = 25;
    const uint8_t IN3 = 14;
    const uint8_t IN4 = 27;
    const uint8_t ENA = 33;
    const uint8_t ENB = 32;
    const uint8_t MIN_PWM = 50;
    const uint8_t MAX_PWM = 255;
    const uint8_t JOYSTICK_MAX = 7;
    const uint8_t DEADZONE = 0;
    const uint16_t SCALE_FACTOR = 512;
    const int16_t RAMP_STEP = 150;
    const unsigned long DATA_TIMEOUT = 200;  // 200ms input timeout
    const unsigned long BT_INIT_DELAY = 500; // BT stabilization delay
    static_assert(MIN_PWM < MAX_PWM, "MIN_PWM must be less than MAX_PWM");
}

// Motor direction enum
enum class MotorDirection { STOP, FORWARD, REVERSE };

// Motor configuration structure
struct MotorConfig {
    uint8_t in_pos : 6;
    uint8_t in_neg : 6;
    uint8_t pwm_pin : 6;
};

// Motor configurations
constexpr MotorConfig MOTORS[] = {
    {Constants::IN1, Constants::IN2, Constants::ENA},
    {Constants::IN3, Constants::IN4, Constants::ENB}
};

enum MotorIndex { LEFT = 0, RIGHT = 1 };

// Boost curve for low joystick inputs
constexpr uint8_t BOOST_TABLE[] = {
    Constants::MIN_PWM,     // 0: 50
    Constants::MIN_PWM,     // 1: 50
    Constants::MIN_PWM + 20,  // 2: 70
    Constants::MIN_PWM + 50,  // 3: 100
    Constants::MIN_PWM + 90,  // 4: 140
    Constants::MIN_PWM + 140, // 5: 190
    Constants::MIN_PWM + 190, // 6: 240
    Constants::MAX_PWM      // 7: 255
};

// PWM scaling
static const uint16_t PWM_SCALE_FACTOR = (Constants::MAX_PWM << 8) / Constants::JOYSTICK_MAX;

// Apply motor speed and direction with immediate stop
void setMotor(MotorIndex motor, int16_t speed) {
    static int16_t last_speeds[2] = {0, 0};
    const MotorConfig& cfg = MOTORS[motor];
    MotorDirection dir = MotorDirection::STOP;
    uint8_t pwm_value = 0;

    // Clear pins on direction change or stop
    if (speed * last_speeds[motor] < 0 || speed == 0) {
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << cfg.in_pos) | (1 << cfg.in_neg));
        analogWrite(cfg.pwm_pin, 0);
    }

    // Immediate stop if speed is 0
    if (speed == 0) {
        last_speeds[motor] = 0; // Reset stored speed
        return;
    }

    // Optimized speed ramping for non-zero speeds
    int16_t& current_speed = last_speeds[motor];
    int16_t delta = speed - current_speed;
    int16_t abs_delta = abs(delta);
    
    if (abs_delta > Constants::RAMP_STEP) {
        current_speed += (delta > 0) ? Constants::RAMP_STEP : -Constants::RAMP_STEP;
    } else {
        current_speed = speed;
    }

    // Clamp speed to valid range
    current_speed = constrain(current_speed, -Constants::MAX_PWM, Constants::MAX_PWM);

    // Determine direction and PWM value
    if (current_speed > 0) {
        dir = MotorDirection::FORWARD;
        pwm_value = (current_speed < Constants::MIN_PWM) ? Constants::MIN_PWM : current_speed;
    } else if (current_speed < 0) {
        dir = MotorDirection::REVERSE;
        pwm_value = (-current_speed < Constants::MIN_PWM) ? Constants::MIN_PWM : -current_speed;
    }

    // Set direction pins
    uint32_t pos_bit = 1 << cfg.in_pos;
    uint32_t neg_bit = 1 << cfg.in_neg;

    if (dir == MotorDirection::FORWARD) {
        REG_WRITE(GPIO_OUT_W1TS_REG, pos_bit);
        REG_WRITE(GPIO_OUT_W1TC_REG, neg_bit);
    } else if (dir == MotorDirection::REVERSE) {
        REG_WRITE(GPIO_OUT_W1TC_REG, pos_bit);
        REG_WRITE(GPIO_OUT_W1TS_REG, neg_bit);
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, pos_bit | neg_bit);
    }

    analogWrite(cfg.pwm_pin, pwm_value);
}

// Input timeout management
bool checkInputTimeout() {
    static unsigned long lastInputTime = 0;
    static bool firstRun = true;
    
    if (firstRun) {
        lastInputTime = millis();
        firstRun = false;
    }
    
    if (millis() - lastInputTime > Constants::DATA_TIMEOUT) {
        setMotor(LEFT, 0);
        setMotor(RIGHT, 0);
        return true;
    }
    return false;
}

// Joystick control with deadzone filtering and calibration
void controlMotors(int16_t raw_X, int16_t raw_Y) {
    static int16_t x_offset = 0, y_offset = 0;
    static bool calibrating = false;
    static unsigned long calibrationStartTime = 0;

    // Non-blocking calibration
    if (GamePad.isStartPressed()) {
        if (!calibrating) {
            calibrating = true;
            calibrationStartTime = millis();
        } else if (millis() - calibrationStartTime > 50) {
            x_offset = raw_X;
            y_offset = raw_Y;
            calibrating = false;
        }
    } else {
        calibrating = false;
    }

    // Apply calibration
    raw_X -= x_offset;
    raw_Y -= y_offset;

    // Deadzone filtering
    if (abs(raw_X) <= Constants::DEADZONE) raw_X = 0;
    if (abs(raw_Y) <= Constants::DEADZONE) raw_Y = 0;

    // Input validation
    if (abs(raw_X) > Constants::JOYSTICK_MAX || abs(raw_Y) > Constants::JOYSTICK_MAX) {
        setMotor(LEFT, 0);
        setMotor(RIGHT, 0);
        return;
    }

    // Calculate motor speeds
    int16_t left_raw_speed = raw_Y - raw_X;
    int16_t right_raw_speed = raw_Y + raw_X;

    // Apply PWM scaling
    int16_t target_left_speed = (left_raw_speed * PWM_SCALE_FACTOR) >> 8;
    int16_t target_right_speed = (right_raw_speed * PWM_SCALE_FACTOR) >> 8;

    // Apply boost curve
    if (target_left_speed != 0) {
        int16_t abs_speed = abs(target_left_speed);
        if (abs_speed <= Constants::JOYSTICK_MAX) {
            abs_speed = BOOST_TABLE[abs_speed];
        } else {
            abs_speed = Constants::MAX_PWM;
        }
        target_left_speed = (target_left_speed > 0) ? abs_speed : -abs_speed;
    }

    if (target_right_speed != 0) {
        int16_t abs_speed = abs(target_right_speed);
        if (abs_speed <= Constants::JOYSTICK_MAX) {
            abs_speed = BOOST_TABLE[abs_speed];
        } else {
            abs_speed = Constants::MAX_PWM;
        }
        target_right_speed = (target_right_speed > 0) ? abs_speed : -abs_speed;
    }

    // Set motors
    setMotor(LEFT, target_left_speed);
    setMotor(RIGHT, target_right_speed);
}

// Optimized Bluetooth initialization
void initBluetooth() {
    WiFi.mode(WIFI_OFF);
    btStart();
    esp_bluedroid_init();
    esp_bluedroid_enable();
    delay(Constants::BT_INIT_DELAY); // Allow BT stack to stabilize
}

// Setup function
void setup() {
    initBluetooth();
    
    for (const auto& motor : MOTORS) {
        pinMode(motor.in_pos, OUTPUT);
        pinMode(motor.in_neg, OUTPUT);
        pinMode(motor.pwm_pin, OUTPUT);
        digitalWrite(motor.in_pos, LOW);
        digitalWrite(motor.in_neg, LOW);
    }
    
    Dabble.begin("MyEsp32");
}

// Override joystick with D-Pad
int16_t overrideX = 0;
int16_t overrideY = 0;
bool overrideActive = false;

// Main loop with continuous joystick processing
void loop() {
    static bool lastOverrideActive = false;
    static bool buttonWasPressed = false;
    bool currentButtonPressed = false;
    bool inputProcessed = false;
    static unsigned long lastInputTime = millis();

    Dabble.processInput();

    // Reset override
    overrideActive = false;
    overrideY = 0;
    overrideX = 0;

    // D-Pad control with press detection
    if (GamePad.isTrianglePressed() || GamePad.isUpPressed()) {
        overrideY = Constants::JOYSTICK_MAX;
        overrideActive = true;
        currentButtonPressed = true;
        inputProcessed = true;
    } else if (GamePad.isCrossPressed() || GamePad.isDownPressed()) {
        overrideY = -Constants::JOYSTICK_MAX;
        overrideActive = true;
        currentButtonPressed = true;
        inputProcessed = true;
    } else if (GamePad.isCirclePressed() || GamePad.isRightPressed()) {
        overrideX = Constants::JOYSTICK_MAX;
        overrideActive = true;
        currentButtonPressed = true;
        inputProcessed = true;
    } else if (GamePad.isSquarePressed() || GamePad.isLeftPressed()) {
        overrideX = -Constants::JOYSTICK_MAX;
        overrideActive = true;
        currentButtonPressed = true;
        inputProcessed = true;
    }
    // Start button also counts as input
    else if (GamePad.isStartPressed()) {
        inputProcessed = true;
        currentButtonPressed = true;
    }

    // Immediate stop when button is released
    if (buttonWasPressed && !currentButtonPressed) {
        setMotor(LEFT, 0);
        setMotor(RIGHT, 0);
        inputProcessed = true; // Reset timeout timer
    }

    buttonWasPressed = currentButtonPressed;

    // Process joystick if no override
    if (!overrideActive) {
        int16_t x = GamePad.getXaxisData();
        int16_t y = GamePad.getYaxisData();
        
        // Process joystick input if not neutral
        if (x != 0 || y != 0) {
            controlMotors(x, y);
            inputProcessed = true;
        } else {
            // Immediate stop when joystick returns to neutral
            setMotor(LEFT, 0);
            setMotor(RIGHT, 0);
            inputProcessed = true;
        }
    } else {
        controlMotors(overrideX, overrideY);
    }

    // Update input timestamp
    if (inputProcessed) {
        lastInputTime = millis();
    }
    // Handle input timeout for joystick (not buttons)
    else if (!currentButtonPressed && millis() - lastInputTime > Constants::DATA_TIMEOUT) {
        setMotor(LEFT, 0);
        setMotor(RIGHT, 0);
    }
    
    lastOverrideActive = overrideActive;
}
//don't remove ANY LINE OF CODE if you don't know how this code work