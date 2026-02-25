#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PS4Controller.h>
#include <SpeedPID.h>
#include <cmath>

// ピン配置
constexpr int8_t PIN_ROTARY_A_1 = 36;
constexpr int8_t PIN_ROTARY_B_1 = 39;
constexpr int8_t PIN_PWM_1      = 32;
constexpr int8_t PIN_DIR_1      = 33;

constexpr int8_t PIN_ROTARY_A_2 = 34;
constexpr int8_t PIN_ROTARY_B_2 = 35;
constexpr int8_t PIN_PWM_2      = 25;
constexpr int8_t PIN_DIR_2      = 26;

constexpr int8_t PIN_ROTARY_A_3 = 27;
constexpr int8_t PIN_ROTARY_B_3 = 23;
constexpr int8_t PIN_PWM_3      = 22;
constexpr int8_t PIN_DIR_3      = 21;

constexpr int8_t PIN_PWM_ARM        = 19;
constexpr int8_t PIN_DIR_ARM        = 18;
constexpr int8_t PIN_LIMIT_SWITCH_1 = 17;
constexpr int8_t PIN_LIMIT_SWITCH_2 = 16;

ESP32Encoder enc1, enc2, enc3;

const int    CONTROL_CYCLE = 5000;
const double RESOLUTION    = 4096.;

constexpr double ROBOT_RADIUS = 0.3;
constexpr double WHEEL_RADIUS = 0.10;

static unsigned long last;

constexpr int8_t MOTOR1_CH = 0;
constexpr int8_t MOTOR2_CH = 1;
constexpr int8_t MOTOR3_CH = 2;

// radに変換
constexpr double WHEEL_ANGLE1 = 0. * PI / 180.;
constexpr double WHEEL_ANGLE2 = 120. * PI / 180.;
constexpr double WHEEL_ANGLE3 = 240. * PI / 180.;

long prev1 = 0, prev2 = 0, prev3 = 0;

SpeedPID pid1(1.0, 0.3, 0.0, -255, 255);
SpeedPID pid2(1.0, 0.3, 0.0, -255, 255);
SpeedPID pid3(1.0, 0.3, 0.0, -255, 255);

double pulseToRad(long deltaCount) {
    return (double(deltaCount) / RESOLUTION) * 2.0 * PI;
}

void setMotor(int8_t ch, int8_t dirPin, double pwm) {
    bool dir = (pwm >= 0) ? HIGH : LOW;
    digitalWrite(dirPin, dir);
    pwm = constrain(abs(pwm), 0., 255.);
    ledcWrite(ch, pwm);
}

void setup() {
    Serial.begin(115200);
    PS4.begin("");

    ESP32Encoder::useInternalWeakPullResistors = puType::none;
    enc1.attachHalfQuad(PIN_ROTARY_A_1, PIN_ROTARY_B_1);
    enc2.attachHalfQuad(PIN_ROTARY_A_2, PIN_ROTARY_B_2);
    enc3.attachHalfQuad(PIN_ROTARY_A_3, PIN_ROTARY_B_3);

    enc1.clearCount();
    enc2.clearCount();
    enc3.clearCount();

    ledcSetup(MOTOR1_CH, 20000, 8);
    ledcSetup(MOTOR2_CH, 20000, 8);
    ledcSetup(MOTOR3_CH, 20000, 8);

    ledcAttachPin(PIN_PWM_1, MOTOR1_CH);
    ledcAttachPin(PIN_PWM_2, MOTOR2_CH);
    ledcAttachPin(PIN_PWM_3, MOTOR3_CH);

    pinMode(PIN_DIR_1, OUTPUT);
    pinMode(PIN_DIR_2, OUTPUT);
    pinMode(PIN_DIR_3, OUTPUT);
}

void loop() {
    unsigned long now = micros();

    if (now - last >= CONTROL_CYCLE) {
        long enc_count1, enc_count2, enc_count3;
        last += CONTROL_CYCLE;
        double dt = CONTROL_CYCLE / 1e6;

        double target_vx    = PS4.LStickX() / 128.0 * 0.5;
        double target_vy    = PS4.LStickY() / 128.0 * 0.5;
        double target_omega = PS4.RStickX() / 128.0 * 3.0;

        const double DEADZONE_V     = 0.05;
        const double DEADZONE_OMEGA = 0.3;
        if (abs(target_vx) < DEADZONE_V) target_vx = 0;
        if (abs(target_vy) < DEADZONE_V) target_vy = 0;
        if (abs(target_omega) < DEADZONE_OMEGA) target_omega = 0;

        enc_count1 = enc1.getCount();
        enc_count2 = enc2.getCount();
        enc_count3 = enc3.getCount();

        long deltaCount1 = enc_count1 - prev1;
        long deltaCount2 = enc_count2 - prev2;
        long deltaCount3 = enc_count3 - prev3;

        prev1 = enc_count1;
        prev2 = enc_count2;
        prev3 = enc_count3;

        double wheelSpeed1 = pulseToRad(deltaCount1) / dt;
        double wheelSpeed2 = pulseToRad(deltaCount2) / dt;
        double wheelSpeed3 = pulseToRad(deltaCount3) / dt;

        // 逆運動学、
        double targetOmega1 =
            (-sin(WHEEL_ANGLE1) * target_vx + cos(WHEEL_ANGLE1) * target_vy + ROBOT_RADIUS * target_omega) / WHEEL_RADIUS;
        double targetOmega2 =
            (-sin(WHEEL_ANGLE2) * target_vx + cos(WHEEL_ANGLE2) * target_vy + ROBOT_RADIUS * target_omega) / WHEEL_RADIUS;
        double targetOmega3 =
            (-sin(WHEEL_ANGLE3) * target_vx + cos(WHEEL_ANGLE3) * target_vy + ROBOT_RADIUS * target_omega) / WHEEL_RADIUS;

        double pwm1 = pid1.update(targetOmega1, wheelSpeed1, dt);
        double pwm2 = pid2.update(targetOmega2, wheelSpeed2, dt);
        double pwm3 = pid3.update(targetOmega3, wheelSpeed3, dt);

        setMotor(MOTOR1_CH, PIN_DIR_1, pwm1);
        setMotor(MOTOR2_CH, PIN_DIR_2, pwm2);
        setMotor(MOTOR3_CH, PIN_DIR_3, pwm3);
    }
}
