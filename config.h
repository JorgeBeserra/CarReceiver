#pragma once
//============================================================
// ⚙️ Mapeamento de Hardware e Parâmetros
//============================================================
#define FIRMWARE_VERSION "1.0.10"

#define WIFI_TIMEOUT 20000
#define PORTAL_TIMEOUT 300000
#define AP_PASSWORD "12345678"
#define AP_IP "192.168.4.1"

// Parametros PWM
const int PWMFreq = 2000;
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;
const int stopPWMChannel = 6;
const int lanternaPWMChannel = 7;

const int BRILHO_MAX = 255;
const int BRILHO_LANTERNA = 102; // 40% de 255

const unsigned long STOP_PULSE_MS = 1200;
const uint16_t BLINK_INTERVAL_MS = 450;

// ====== CLICK DO PISCA (rele) ======
const int BLINK_CLICK_FREQ = 900;          // som do "tic"
const int BLINK_CLICK_DUTY = 70;            // baixinho (0..255)
const unsigned long BLINK_CLICK_MS = 15;    // duração do "tic"

const int BUZZER_CHANNEL = 0;
const int HORN_FREQUENCY_HIGH = 400;
const int HORN_FREQUENCY_LOW = 330;
const int HORN_DURATION_MS = 500;
const int HORN_SWEEP_MS = 100;

// ====== BUZINA "FON FON" (buzzer passivo) ======
const int HORN_FON1 = 430;
const int HORN_FON2 = 330;
const int HORN_FON_INTERVAL_MS = 110;
const int HORN_DUTY_ON = 160;   // volume (0..255)
const int HORN_DUTY_OFF = 0;

// ===== DIRIGIBILIDADE (4x4 tank) =====
const int AXIS_MAX = 512;

// deadzones
const int DEADZONE_THROTTLE = 40;   // 25..70
const int DEADZONE_STEER    = 55;   // 40..90

// expo: 0.0 linear / 0.6 bem suave
const float EXPO_THROTTLE = 0.45f;
const float EXPO_STEER    = 0.60f;

// rampa (suaviza tranco)
const int RAMP_STEP = 6;            // 4..12