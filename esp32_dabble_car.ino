#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <WiFi.h>
#include <soc/gpio_reg.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_err.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

// Logging tag for debugging
static const char* TAG = "RobotControl";

// Constants and configuration
namespace Constants {
    const uint8_t IN1 = 26;           // Motor A positive pin
    const uint8_t IN2 = 25;           // Motor A negative pin
    const uint8_t IN3 = 14;           // Motor B positive pin
    const uint8_t IN4 = 27;           // Motor B negative pin
    const uint8_t ENA = 33;           // Motor A PWM pin
    const uint8_t ENB = 32;           // Motor B PWM pin
    const uint8_t LED_PIN = 2;        // Onboard LED for status
    const uint8_t MIN_PWM = 50;       // Minimum PWM value
    const uint8_t MAX_PWM = 255;      // Maximum PWM value
    const uint8_t JOYSTICK_MAX = 7;   // Maximum joystick value
    const uint8_t DEADZONE = 0;       // Deadzone set to 0 for immediate response
    const uint16_t SCALE_FACTOR = 512; // Scaling factor for calculations
    const unsigned long DATA_TIMEOUT = 500; // Input timeout (ms)
    const unsigned long BT_INIT_DELAY = 80; // Bluetooth stabilization delay
    const int PWM_FREQ_DEFAULT = 5000; // Default PWM frequency (Hz)
    const int PWM_FREQ_LOW = 1000;     // Low PWM frequency for high torque
    const int PWM_RESOLUTION = 8;      // PWM resolution (bits)
    const ledc_channel_t LEDC_CHANNEL_LEFT = LEDC_CHANNEL_0; // Left motor channel
    const ledc_channel_t LEDC_CHANNEL_RIGHT = LEDC_CHANNEL_1; // Right motor channel
    const uint8_t DIAGNOSTIC_INTERVAL = 80; // Diagnostic interval (ms)
    const uint8_t CALIBRATION_SAMPLES = 12; // Samples for joystick calibration
    const uint8_t MAX_ERROR_COUNT = 5; // Max errors before fault
    const uint8_t INPUT_SMOOTHING_SAMPLES = 1; // Set to 1 to disable smoothing
    const unsigned long LED_BLINK_INTERVAL = 500; // LED blink interval (ms)
    static_assert(MIN_PWM < MAX_PWM, "MIN_PWM must be less than MAX_PWM");
}

// Motor direction enum
enum class MotorDirection { STOP, FORWARD, REVERSE };

// Motor state enum for state machine
enum class MotorState { IDLE, RUNNING, FAULT, CALIBRATING };

// Motor mode enum for different driving profiles
enum class DriveMode { PRECISION, NORMAL, HIGH_SPEED };

// Motor configuration structure
struct MotorConfig {
    uint8_t in_pos : 6; // Positive direction pin
    uint8_t in_neg : 6; // Negative direction pin
    uint8_t pwm_pin : 6; // PWM pin
    ledc_channel_t ledc_channel; // LEDC channel
};

// Motor configurations
constexpr MotorConfig MOTORS[] = {
    {Constants::IN1, Constants::IN2, Constants::ENA, Constants::LEDC_CHANNEL_LEFT},
    {Constants::IN3, Constants::IN4, Constants::ENB, Constants::LEDC_CHANNEL_RIGHT}
};

enum MotorIndex { LEFT = 0, RIGHT = 1 };

// Boost curves for different drive modes
constexpr uint8_t PRECISION_BOOST_TABLE[] = {
    Constants::MIN_PWM,     // 0: 50
    Constants::MIN_PWM,     // 1: 50
    Constants::MIN_PWM + 10,  // 2: 60
    Constants::MIN_PWM + 30,  // 3: 80
    Constants::MIN_PWM + 60,  // 4: 110
    Constants::MIN_PWM + 100, // 5: 150
    Constants::MIN_PWM + 150, // 6: 200
    Constants::MAX_PWM - 50   // 7: 205
};

constexpr uint8_t NORMAL_BOOST_TABLE[] = {
    Constants::MIN_PWM,     // 0: 50
    Constants::MIN_PWM,     // 1: 50
    Constants::MIN_PWM + 20,  // 2: 70
    Constants::MIN_PWM + 50,  // 3: 100
    Constants::MIN_PWM + 90,  // 4: 140
    Constants::MIN_PWM + 140, // 5: 190
    Constants::MIN_PWM + 190, // 6: 240
    Constants::MAX_PWM      // 7: 255
};

constexpr uint8_t HIGH_SPEED_BOOST_TABLE[] = {
    Constants::MIN_PWM + 20,  // 0: 70
    Constants::MIN_PWM + 20,  // 1: 70
    Constants::MIN_PWM + 40,  // 2: 90
    Constants::MIN_PWM + 80,  // 3: 130
    Constants::MIN_PWM + 120, // 4: 170
    Constants::MIN_PWM + 170, // 5: 220
    Constants::MIN_PWM + 205, // 6: 255
    Constants::MAX_PWM      // 7: 255
};

// Motor diagnostic data
struct MotorDiagnostics {
    uint32_t last_pwm_value;
    MotorDirection last_direction;
    uint8_t error_count;
    bool is_overloaded;
};

// Joystick input structure
struct JoystickInput {
    int16_t x;
    int16_t y;
};

// Global variables
static MotorDiagnostics motor_diagnostics[2] = {{0, MotorDirection::STOP, 0, false}, {0, MotorDirection::STOP, 0, false}};
static MotorState motor_state = MotorState::IDLE;
static DriveMode drive_mode = DriveMode::NORMAL;
static bool is_initialized = false;
static int current_pwm_freq = Constants::PWM_FREQ_DEFAULT;
static bool led_state = false;

// Initialize LEDC for PWM
void initPWM() {
    ledc_channel_config_t ledc_channel[2] = {
        {
            .gpio_num = Constants::ENA,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = Constants::LEDC_CHANNEL_LEFT,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = Constants::ENB,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = Constants::LEDC_CHANNEL_RIGHT,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        }
    };

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = static_cast<ledc_timer_bit_t>(Constants::PWM_RESOLUTION),
        .timer_num = LEDC_TIMER_0,
        .freq_hz = Constants::PWM_FREQ_DEFAULT,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[0]));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[1]));
    ESP_LOGI(TAG, "PWM initialized: freq=%d Hz, resolution=%d bits", Constants::PWM_FREQ_DEFAULT, Constants::PWM_RESOLUTION);
}

// Adjust PWM frequency dynamically
void adjustPWMFrequency(int freq) {
    if (freq == current_pwm_freq) return;
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = static_cast<ledc_timer_bit_t>(Constants::PWM_RESOLUTION),
        .timer_num = LEDC_TIMER_0,
        .freq_hz = freq,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    current_pwm_freq = freq;
    ESP_LOGI(TAG, "PWM frequency adjusted to %d Hz", freq);
}

// Apply motor speed and direction with immediate response
void setMotor(MotorIndex motor, int16_t speed) {
    const MotorConfig& cfg = MOTORS[motor];
    MotorDirection dir = MotorDirection::STOP;
    uint8_t pwm_value = 0;

    if (speed > 0) {
        dir = MotorDirection::FORWARD;
        pwm_value = (speed < Constants::MIN_PWM) ? Constants::MIN_PWM : speed;
    } else if (speed < 0) {
        dir = MotorDirection::REVERSE;
        pwm_value = (-speed < Constants::MIN_PWM) ? Constants::MIN_PWM : -speed;
    } else {
        dir = MotorDirection::STOP;
        pwm_value = 0;
    }

    adjustPWMFrequency(pwm_value < Constants::MIN_PWM + 50 && drive_mode != DriveMode::HIGH_SPEED ? Constants::PWM_FREQ_LOW : Constants::PWM_FREQ_DEFAULT);

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

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, cfg.ledc_channel, pwm_value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, cfg.ledc_channel));

    motor_diagnostics[motor].last_pwm_value = pwm_value;
    motor_diagnostics[motor].last_direction = dir;

    ESP_LOGV(TAG, "Motor %d: Speed=%d, Direction=%d, PWM=%d", motor, speed, static_cast<int>(dir), pwm_value);
}

// Perform motor diagnostics
void runMotorDiagnostics() {
    static unsigned long last_diagnostic_time = 0;
    if (millis() - last_diagnostic_time < Constants::DIAGNOSTIC_INTERVAL) {
        return;
    }
    last_diagnostic_time = millis();

    for (int i = 0; i < 2; ++i) {
        if (motor_diagnostics[i].last_pwm_value > Constants::MAX_PWM * 0.9) {
            motor_diagnostics[i].error_count++;
            if (motor_diagnostics[i].error_count >= Constants::MAX_ERROR_COUNT) {
                motor_diagnostics[i].is_overloaded = true;
                motor_state = MotorState::FAULT;
                ESP_LOGE(TAG, "Motor %d overload detected!", i);
                setMotor(static_cast<MotorIndex>(i), 0);
            }
        } else {
            motor_diagnostics[i].error_count = 0;
            motor_diagnostics[i].is_overloaded = false;
        }
    }
}

// Update LED indicator based on state and mode
void updateLED() {
    static unsigned long last_led_time = 0;
    if (millis() - last_led_time < Constants::LED_BLINK_INTERVAL) return;
    last_led_time = millis();

    if (motor_state == MotorState::FAULT) {
        digitalWrite(Constants::LED_PIN, !led_state);
    } else {
        switch (drive_mode) {
            case DriveMode::PRECISION:
                digitalWrite(Constants::LED_PIN, HIGH);
                break;
            case DriveMode::NORMAL:
                digitalWrite(Constants::LED_PIN, !led_state);
                break;
            case DriveMode::HIGH_SPEED:
                digitalWrite(Constants::LED_PIN, !led_state);
                break;
        }
    }
    led_state = !led_state;
}

// Smooth joystick input (disabled with INPUT_SMOOTHING_SAMPLES=1)
JoystickInput smoothInput(int16_t raw_X, int16_t raw_Y) {
    return {raw_X, raw_Y}; // No smoothing applied
}

// Select boost table based on drive mode
const uint8_t* selectBoostTable() {
    switch (drive_mode) {
        case DriveMode::PRECISION: return PRECISION_BOOST_TABLE;
        case DriveMode::NORMAL: return NORMAL_BOOST_TABLE;
        case DriveMode::HIGH_SPEED: return HIGH_SPEED_BOOST_TABLE;
        default: return NORMAL_BOOST_TABLE;
    }
}

// Joystick control with no deadzone and immediate response
void controlMotors(int16_t raw_X, int16_t raw_Y) {
    static int16_t x_offset = 0, y_offset = 0;
    static bool calibrating = false;
    static unsigned long calibration_start_time = 0;
    static int16_t x_samples[Constants::CALIBRATION_SAMPLES] = {0};
    static int16_t y_samples[Constants::CALIBRATION_SAMPLES] = {0};
    static uint8_t sample_index = 0;
    static uint8_t sample_count = 0;

    if (GamePad.isStartPressed()) {
        if (!calibrating) {
            calibrating = true;
            calibration_start_time = millis();
            sample_index = 0;
            sample_count = 0;
            for (uint8_t i = 0; i < Constants::CALIBRATION_SAMPLES; ++i) {
                x_samples[i] = 0;
                y_samples[i] = 0;
            }
            ESP_LOGI(TAG, "Starting joystick calibration");
        } else if (millis() - calibration_start_time > 50 && sample_count < Constants::CALIBRATION_SAMPLES) {
            x_samples[sample_index] = raw_X;
            y_samples[sample_index] = raw_Y;
            sample_index = (sample_index + 1) % Constants::CALIBRATION_SAMPLES;
            sample_count++;
            if (sample_count >= Constants::CALIBRATION_SAMPLES) {
                int32_t x_sum = 0, y_sum = 0;
                for (uint8_t i = 0; i < Constants::CALIBRATION_SAMPLES; ++i) {
                    x_sum += x_samples[i];
                    y_sum += y_samples[i];
                }
                x_offset = x_sum / Constants::CALIBRATION_SAMPLES;
                y_offset = y_sum / Constants::CALIBRATION_SAMPLES;
                calibrating = false;
                ESP_LOGI(TAG, "Calibration complete: x_offset=%d, y_offset=%d", x_offset, y_offset);
            }
        }
    } else {
        calibrating = false;
    }

    raw_X -= x_offset;
    raw_Y -= y_offset;

    JoystickInput smoothed = smoothInput(raw_X, raw_Y);
    raw_X = smoothed.x;
    raw_Y = smoothed.y;

    if (abs(raw_X) <= Constants::DEADZONE) raw_X = 0;
    if (abs(raw_Y) <= Constants::DEADZONE) raw_Y = 0;

    if (abs(raw_X) > Constants::JOYSTICK_MAX || abs(raw_Y) > Constants::JOYSTICK_MAX) {
        ESP_LOGW(TAG, "Invalid joystick input: x=%d, y=%d", raw_X, raw_Y);
        setMotor(LEFT, 0);
        setMotor(RIGHT, 0);
        return;
    }

    // Calculate and clip raw motor speeds
    int16_t left_raw_speed = raw_Y - raw_X;
    int16_t right_raw_speed = raw_Y + raw_X;
    int16_t clipped_left_speed = constrain(left_raw_speed, -Constants::JOYSTICK_MAX, Constants::JOYSTICK_MAX);
    int16_t clipped_right_speed = constrain(right_raw_speed, -Constants::JOYSTICK_MAX, Constants::JOYSTICK_MAX);

    // Apply boost table to clipped speeds
    const uint8_t* boost_table = selectBoostTable();
    uint8_t abs_left = abs(clipped_left_speed);
    uint8_t abs_right = abs(clipped_right_speed);
    uint8_t left_pwm = (abs_left == 0) ? 0 : boost_table[abs_left];
    uint8_t right_pwm = (abs_right == 0) ? 0 : boost_table[abs_right];
    int16_t target_left_speed = (clipped_left_speed > 0) ? left_pwm : -left_pwm;
    int16_t target_right_speed = (clipped_right_speed > 0) ? right_pwm : -right_pwm;

    if (motor_state != MotorState::FAULT) {
        setMotor(LEFT, target_left_speed);
        setMotor(RIGHT, target_right_speed);
    } else {
        setMotor(LEFT, 0);
        setMotor(RIGHT, 0);
        ESP_LOGE(TAG, "Motor fault state: stopping motors");
    }
}

// Optimized Bluetooth initialization
void initBluetooth() {
    esp_err_t ret;
    WiFi.mode(WIFI_OFF);
    ESP_LOGI(TAG, "Disabling WiFi to reduce interference");

    ret = btStart();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth start failed: %s", esp_err_to_name(ret));
        motor_state = MotorState::FAULT;
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        motor_state = MotorState::FAULT;
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        motor_state = MotorState::FAULT;
        return;
    }

    vTaskDelay(Constants::BT_INIT_DELAY / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Bluetooth initialized successfully");
}

// Initialize GPIO pins
void initGPIO() {
    for (const auto& motor : MOTORS) {
        pinMode(motor.in_pos, OUTPUT);
        pinMode(motor.in_neg, OUTPUT);
        pinMode(motor.pwm_pin, OUTPUT);
        digitalWrite(motor.in_pos, LOW);
        digitalWrite(motor.in_neg, LOW);
        ESP_LOGI(TAG, "Initialized GPIO pins: pos=%d, neg=%d, pwm=%d", motor.in_pos, motor.in_neg, motor.pwm_pin);
    }
    pinMode(Constants::LED_PIN, OUTPUT);
    digitalWrite(Constants::LED_PIN, LOW);
    ESP_LOGI(TAG, "Initialized LED pin: %d", Constants::LED_PIN);
}

// Switch drive mode
void switchDriveMode() {
    static unsigned long last_switch_time = 0;
    if (millis() - last_switch_time < 1000) return;
    last_switch_time = millis();

    if (GamePad.isSelectPressed()) {
        drive_mode = static_cast<DriveMode>((static_cast<int>(drive_mode) + 1) % 3);
        ESP_LOGI(TAG, "Switched to drive mode: %d (0=Precision, 1=Normal, 2=HighSpeed)", static_cast<int>(drive_mode));
    }
}

// Setup function
void setup() {
    Serial.begin(115200);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    ESP_LOGI(TAG, "Starting robot control setup");

    vTaskPrioritySet(NULL, 5);

    initBluetooth();
    initGPIO();
    initPWM();

    Dabble.begin("MyEsp32");
    ESP_LOGI(TAG, "Dabble Bluetooth initialized");

    is_initialized = true;
    motor_state = MotorState::IDLE;
    ESP_LOGI(TAG, "Setup complete, entering idle state");
}

// Override joystick with D-Pad
int16_t overrideX = 0;
int16_t overrideY = 0;
bool overrideActive = false;

// Main loop with state machine
void loop() {
    static bool lastOverrideActive = false;
    static bool buttonWasPressed = false;
    bool currentButtonPressed = false;
    bool inputProcessed = false;
    static unsigned long lastInputTime = millis();

    if (!is_initialized) {
        ESP_LOGE(TAG, "System not initialized, halting");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        return;
    }

    switch (motor_state) {
        case MotorState::IDLE:
            Dabble.processInput();
            runMotorDiagnostics();
            switchDriveMode();
            updateLED();
            break;
        case MotorState::RUNNING:
            Dabble.processInput();
            runMotorDiagnostics();
            switchDriveMode();
            updateLED();
            break;
        case MotorState::FAULT:
            ESP_LOGE(TAG, "Fault state: motors stopped");
            setMotor(LEFT, 0);
            setMotor(RIGHT, 0);
            switchDriveMode();
            updateLED();
            return;
        case MotorState::CALIBRATING:
            Dabble.processInput();
            updateLED();
            break;
        default:
            ESP_LOGE(TAG, "Unknown motor state");
            motor_state = MotorState::FAULT;
            return;
    }

    overrideActive = false;
    overrideY = 0;
    overrideX = 0;

    if (GamePad.isUpPressed()) {
        overrideY = Constants::JOYSTICK_MAX;
        overrideActive = true;
        currentButtonPressed = true;
        inputProcessed = true;
        ESP_LOGV(TAG, "D-Pad Up pressed");
    } else if (GamePad.isDownPressed()) {
        overrideY = -Constants::JOYSTICK_MAX;
        overrideActive = true;
        currentButtonPressed = true;
        inputProcessed = true;
        ESP_LOGV(TAG, "D-Pad Down pressed");
    } else if (GamePad.isRightPressed()) {
        overrideX = Constants::JOYSTICK_MAX;
        overrideActive = true;
        currentButtonPressed = true;
        inputProcessed = true;
        ESP_LOGV(TAG, "D-Pad Right pressed");
    } else if (GamePad.isLeftPressed()) {
        overrideX = -Constants::JOYSTICK_MAX;
        overrideActive = true;
        currentButtonPressed = true;
        inputProcessed = true;
        ESP_LOGV(TAG, "D-Pad Left pressed");
    } else if (GamePad.isStartPressed()) {
        inputProcessed = true;
        currentButtonPressed = true;
        ESP_LOGV(TAG, "Start button pressed");
    }

    if (buttonWasPressed && !currentButtonPressed) {
        setMotor(LEFT, 0);
        setMotor(RIGHT, 0);
        inputProcessed = true;
        motor_state = MotorState::IDLE;
        ESP_LOGI(TAG, "Button released, motors stopped");
    }

    buttonWasPressed = currentButtonPressed;

    if (!overrideActive) {
        int16_t x = GamePad.getXaxisData();
        int16_t y = GamePad.getYaxisData();
        ESP_LOGI(TAG, "Joystick input: x=%d, y=%d", x, y);

        if (x != 0 || y != 0) {
            controlMotors(x, y);
            inputProcessed = true;
            motor_state = MotorState::RUNNING;
            ESP_LOGV(TAG, "Joystick input processed: x=%d, y=%d", x, y);
        } else {
            setMotor(LEFT, 0);
            setMotor(RIGHT, 0);
            inputProcessed = true;
            motor_state = MotorState::IDLE;
            ESP_LOGV(TAG, "Joystick neutral, motors stopped");
        }
    } else {
        controlMotors(overrideX, overrideY);
        motor_state = MotorState::RUNNING;
    }

    if (inputProcessed) {
        lastInputTime = millis();
    } else if (!currentButtonPressed && millis() - lastInputTime > Constants::DATA_TIMEOUT) {
        setMotor(LEFT, 0);
        setMotor(RIGHT, 0);
        motor_state = MotorState::IDLE;
        ESP_LOGI(TAG, "Input timeout, motors stopped");
    }

    lastOverrideActive = overrideActive;

    vTaskDelay(1 / portTICK_PERIOD_MS);
}
