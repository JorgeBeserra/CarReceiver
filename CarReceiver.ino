#include <Bluepad32.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include "esp_wifi.h"

static const char* GITHUB_LATEST = "https://api.github.com/repos/JorgeBeserra/CarReceiver/releases/latest";

//============================================================
// ⚙️ Mapeamento de Hardware e Parâmetros
//============================================================
#define FIRMWARE_VERSION "1.0.1"

// 🟦 Pinos
// Controle
// Quando o Stiek da esquerda ficar no neutro tem que parar os motores e tem que acender as luzes do stop durante 
// Quando o Stick do lado esquerdo for para tras tem que acender a luz de ré
// Botao X -> Buzina
// Botao Quadrado -> Farol Baixo e Farol Alto
// Botao triangulo -> Pisca Alerta
// R1 -> Pisca Direita
// L1 -> Pisca Esquerda
// R2 -> Acelerador
// L2 -> Freio tem que acender as luzes de Freio e tem que parar os motores

// Pinos
// Controle dos Motores (PONTE H)
const int enableRightMotor = 14; // PWM pin for motor A - Marrom
const int rightMotorPin1 = 13; // Motor esquerdo - Azul
const int rightMotorPin2 = 12; // Vermelho
const int leftMotorPin1 = 27;// Motor direito - Verde
const int leftMotorPin2 = 32;// Laranja
const int enableLeftMotor = 26;// PWM pin for motor B - Amarelo

// Pinos dos Leds
const int FAROL_BAIXO = 15;  // PWM pin for motor B
const int FAROL_ALTO = 22;  // PWM pin for motor B
const int RE = 2;  // PWM pin for motor B
const int FREIO = 4;  // PWM pin for motor B
const int STOP = 21;  // PWM pin for motor B
const int SETA_DIREITA = 16;  // PWM pin for motor B
const int SETA_ESQUERDA = 17;  // PWM pin for motor B

// Parametros PWM
const int PWMFreq = 2000;
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

int motorDirection = 1;
int currentSentidoMovimento = 0; // 0: Parado, -1: Ré, 1: Frente
int currentSentidoDirecao = 0; // -1: Esquerda, 0: Neutro, 1: Direita
int speed1;
int speed2;
int direction1;
int direction2;
const int STEERING_DEADZONE = 50;

// Variáveis de Estado dos Controles e Funcionamentosbool isXButtonPressed = false;
bool isSquarePressed = false;
bool isTrianglePressed = false;
bool isCirclePressed = false;
bool isXButtonPressed = false;
bool isR1Pressed = false;
bool isL1Pressed = false;
bool isR2Pressed = false;
bool isL2Pressed = false;

bool farolBaixoAcendido = false;
bool farolAltoAcendido = false;
bool freioAcendido = false;
bool stopAcendido = false;
bool reAcendido = false;
bool piscaEsquerdo = false;
bool piscaDireito = false;

// Buzzer
const int BUZZER_PIN = 23; 
const int BUZZER_CHANNEL = 0;
const int HORN_FREQUENCY_HIGH = 400;
const int HORN_FREQUENCY_LOW = 330;
const int HORN_DURATION_MS = 500;
const int HORN_SWEEP_MS = 100;
int currentHornFrequency = HORN_FREQUENCY_HIGH;
bool isHornSweepUp = false;
unsigned long hornStartTime = 0;
bool isHornPlaying = false;

// ====== BUZINA "FON FON" (buzzer passivo) ======
const int HORN_FON1 = 430;
const int HORN_FON2 = 330;
const int HORN_FON_INTERVAL_MS = 110;
const int HORN_DUTY_ON = 160;   // volume (0..255)
const int HORN_DUTY_OFF = 0;


bool prevX = false;
bool hornToggle = false;
unsigned long lastHornToggle = 0;

// ====== PISCA ======
const unsigned long BLINK_INTERVAL_MS = 450;
unsigned long lastBlinkMs = 0;
bool blinkState = false;

bool hazardOn = false;      // pisca alerta (triângulo)
bool leftBlinkOn = false;   // seta esquerda (L1)
bool rightBlinkOn = false;  // seta direita (R1)

bool hornOn = false;

bool prevTriangle = false;
bool prevL1 = false;
bool prevR1 = false;

bool prevB = false;
bool farolAltoLigado = false;


// ====== CLICK DO PISCA (rele) ======
const int BLINK_CLICK_FREQ = 900;          // som do "tic"
const int BLINK_CLICK_DUTY = 70;            // baixinho (0..255)
const unsigned long BLINK_CLICK_MS = 15;    // duração do "tic"

bool clickActive = false;
unsigned long clickEndMs = 0;

bool manualBlinkMode = false;   // true = L1/R1 está controlando

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

// trim p/ corrigir lado que puxa (ajuste fino)
float TRIM_LEFT  = 1.00f;           // ex: 0.96
float TRIM_RIGHT = 1.00f;           // ex: 1.04

// limite de direção em alta (quanto maior, MAIS reduz)
const float STEER_REDUCE = 0.55f;   // 0.35..0.70

int pwmL = 0, pwmR = 0;

static unsigned long otaPressStart = 0;
static bool otaHolding = false;

static inline int applyDeadzone(int v, int dz) {
  if (abs(v) <= dz) return 0;
  int s = (v > 0) ? 1 : -1;
  int vv = abs(v) - dz;
  int maxv = AXIS_MAX - dz;
  return s * (vv * AXIS_MAX / maxv);
}

static inline float applyExpo(float x, float expo) {
  return (1.0f - expo) * x + expo * x * x * x;
}

static inline int rampTo(int cur, int target, int step) {
  if (cur < target) return min(cur + step, target);
  if (cur > target) return max(cur - step, target);
  return cur;
}

static inline int clamp255(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

static bool testDNS(const char* host) {
  IPAddress ip;
  Serial.printf("[DNS] Resolvendo %s ... ", host);
  bool ok = WiFi.hostByName(host, ip);
  if (ok) {
    Serial.println(ip);
  } else {
    Serial.println("FALHOU");
  }
  return ok;
}



ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Helpers
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);
  } else {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);
  }
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);
  } else {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);
  }
  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
}

void startBlinkClick() {
  if (hornOn) return;   // não briga com buzina

  // "Relé antigo": mais grave
  // Um tom quando LIGA a seta e outro quando DESLIGA
  int freq = blinkState ? 520 : 420;   // ajuste aqui se quiser mais grave: 450/350

  ledcChangeFrequency(BUZZER_CHANNEL, freq, PWMResolution);
  ledcWrite(BUZZER_CHANNEL, 200);      // “pancada” do tac (0..255)

  clickActive = true;
  clickEndMs = millis() + 10;          // 8~12ms fica ótimo
}

void updateBlinkClick() {
  if (!clickActive) return;
  if (millis() >= clickEndMs) {
    ledcWrite(BUZZER_CHANNEL, 0);
    clickActive = false;
  }
}

void updateBlinkInputs(ControllerPtr ctl) {
  bool tri = ctl->y();
  bool l1  = ctl->l1();
  bool r1  = ctl->r1();

  // ===== TRIÂNGULO (alerta) =====
  if (tri && !prevTriangle) {
    hazardOn = !hazardOn;

    if (hazardOn) {
      leftBlinkOn = false;
      rightBlinkOn = false;
      manualBlinkMode = false;
    }
  }

  // Se alerta estiver ativo, nada mais mexe
  if (hazardOn) {
    prevTriangle = tri;
    prevL1 = l1;
    prevR1 = r1;
    return;
  }

  // ===== L1 manual esquerda =====
  if (l1 && !prevL1) {
    manualBlinkMode = true;

    leftBlinkOn = !leftBlinkOn;
    rightBlinkOn = false;

    if (!leftBlinkOn) manualBlinkMode = false;  // se desligou, volta pro automático
  }

  // ===== R1 manual direita =====
  if (r1 && !prevR1) {
    manualBlinkMode = true;

    rightBlinkOn = !rightBlinkOn;
    leftBlinkOn = false;

    if (!rightBlinkOn) manualBlinkMode = false;
  }

  prevTriangle = tri;
  prevL1 = l1;
  prevR1 = r1;
}

bool bdEquals(const uint8_t* bd,
              uint8_t a, uint8_t b, uint8_t c,
              uint8_t d, uint8_t e, uint8_t f) {
  return bd[0]==a && bd[1]==b && bd[2]==c &&
         bd[3]==d && bd[4]==e && bd[5]==f;
}

void setControllerColorByCarMac(ControllerPtr ctl) {
  const uint8_t* bd = BP32.localBdAddress(); // MAC do carrinho (ESP32)

  // Carrinho A
  if (bdEquals(bd, 0x98,0xF4,0xAB,0x6D,0x25,0xDA)) {
    ctl->setColorLED(255, 0, 0);   // vermelho
  }
  // Carrinho B (exemplo)
  else if (bdEquals(bd, 0x24,0x6F,0x28,0x11,0x22,0x33)) {
    ctl->setColorLED(0, 0, 255);   // azul
  }
  // Outros
  else {
    ctl->setColorLED(255, 0, 255); // rosa
  }
}

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

            setControllerColorByCarMac(ctl);
            
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


void dumpBalanceBoard(ControllerPtr ctl) {
    Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

// int moveMotores(int sentidoX, int steeringY, int throttleR2) {
//   int sentido = map(sentidoX, -120, 120, -255, 255);
//   int steer = map(steeringY, -120, 120, -255, 255);
//   Serial.printf("rawY=%d rawX=%d -> sentido=%d steer=%d\n", sentidoX, steeringY, sentido, steer);
//   // Mapeia throttle (0-1023) para fator de velocidade (10% a 100%)
//   float throttle = constrain(throttleR2, 0, 1023);
//   float factor = 0.1f + (throttle / 1023.0f) * 0.9f; // 0.1 a 1.0
//   int rightSpeed = (int)((abs(sentido) - steer) * factor);
//   int leftSpeed = (int)((abs(sentido) + steer) * factor);
//   rightSpeed = constrain(rightSpeed, 0, 255);
//   leftSpeed = constrain(leftSpeed, 0, 255);
//   int rDir = (sentido < 0) ? -1 : 1;
//   int lDir = (sentido < 0) ? -1 : 1;
//   rotateMotor(rightSpeed * rDir, leftSpeed * lDir);
//   return 0;
// }

int moveMotores(int sentidoX, int steeringY, int throttleR2) {
  sentidoX = -sentidoX;

  int y = map(sentidoX,  -511, 512, -255, 255);
  int x = map(steeringY, -511, 512, -255, 255);

  y = constrain(y, -255, 255);
  x = constrain(x, -255, 255);

  const int DZ = 10;
  if (abs(y) < DZ) y = 0;
  if (abs(x) < DZ) x = 0;

  float t = constrain(throttleR2, 0, 1023) / 1023.0f;
  float factor = 0.45f + t * 0.55f;  // <<< mais forte no thr=0

  int right = (int)((y - x) * factor);
  int left  = (int)((y + x) * factor);

  right = constrain(right, -255, 255);
  left  = constrain(left,  -255, 255);

  // <<< mínimo para vencer atrito
  const int MIN_PWM = 80; // ajuste fino
  auto applyMinPwm = [&](int v) -> int {
    if (v == 0) return 0;
    int s = (v > 0) ? 1 : -1;
    int a = abs(v);
    if (a < MIN_PWM) a = MIN_PWM;
    return s * a;
  };
  left  = applyMinPwm(left);
  right = applyMinPwm(right);

  //Serial.printf("rawY=%d rawX=%d thr=%d -> y=%d x=%d -> L=%d R=%d\n", -sentidoX, steeringY, throttleR2, y, x, left, right);

  rotateMotor(-right, -left);
  return 0;
}

void updateHornFonFon(ControllerPtr ctl) {
  bool xNow = ctl->a();
  unsigned long now = millis();

  // Apertou (borda de subida)
  if (xNow && !prevX) {
    hornOn = true;
    hornToggle = false;
    lastHornToggle = now;

    ledcChangeFrequency(BUZZER_CHANNEL, HORN_FON1, PWMResolution);
    ledcWrite(BUZZER_CHANNEL, HORN_DUTY_ON);
  }

  // Segurando: alterna os tons
  if (xNow) {
    if (now - lastHornToggle >= (unsigned long)HORN_FON_INTERVAL_MS) {
      lastHornToggle = now;
      hornToggle = !hornToggle;

      int f = hornToggle ? HORN_FON1 : HORN_FON2;
      ledcChangeFrequency(BUZZER_CHANNEL, f, PWMResolution);

      // mantém tocando
      ledcWrite(BUZZER_CHANNEL, HORN_DUTY_ON);
    }
  }

  // Soltou (borda de descida)
  if (!xNow && prevX) {
    hornOn = false;
    ledcWrite(BUZZER_CHANNEL, HORN_DUTY_OFF);
  }

  prevX = xNow;
}

void updateBlinkFromSteering() {

  if (hazardOn) return;
  if (manualBlinkMode) return;   // <<< IMPORTANTE

  int x = currentSentidoDirecao;

  const int STEER_ON  = 150;
  const int STEER_OFF = 80;

  if (leftBlinkOn) {
    if (x > -STEER_OFF) leftBlinkOn = false;
    rightBlinkOn = false;
    return;
  }

  if (rightBlinkOn) {
    if (x < STEER_OFF) rightBlinkOn = false;
    leftBlinkOn = false;
    return;
  }

  if (x < -STEER_ON) {
    leftBlinkOn = true;
    rightBlinkOn = false;
  } 
  else if (x > STEER_ON) {
    rightBlinkOn = true;
    leftBlinkOn = false;
  }
}


static bool isNewerVersionESP32(String current, String latest) {
  current.replace("v", "");
  latest.replace("v", "");
  current.replace(".", "");
  latest.replace(".", "");
  return latest.toInt() > current.toInt();
}

static bool githubGetLatest(String& outVersion, String& outBinUrl) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  https.setTimeout(12000);

  Serial.printf("[OTA] GET: %s\n", GITHUB_LATEST);

  if (!https.begin(client, GITHUB_LATEST)) {
    Serial.println("[OTA] https.begin() falhou");
    return false;
  }

  https.addHeader("User-Agent", "ESP32");
  https.addHeader("Accept", "application/vnd.github+json");

  int code = https.GET();
  Serial.printf("[OTA] HTTP code: %d\n", code);

  String payload = https.getString();
  https.end();

  Serial.print("[OTA] Body (inicio): ");
  Serial.println(payload.substring(0, 200));

  if (code != HTTP_CODE_OK) return false;

  int tagIndex = payload.indexOf("\"tag_name\":\"");
  if (tagIndex < 0) {
    Serial.println("[OTA] tag_name nao encontrado");
    return false;
  }
  int vStart = tagIndex + strlen("\"tag_name\":\"");
  int vEnd = payload.indexOf("\"", vStart);
  outVersion = payload.substring(vStart, vEnd);

  // pega o PRIMEIRO browser_download_url
  int urlIndex = payload.indexOf("\"browser_download_url\":\"");
  if (urlIndex < 0) {
    Serial.println("[OTA] browser_download_url nao encontrado");
    return false;
  }
  int uStart = urlIndex + strlen("\"browser_download_url\":\"");
  int uEnd = payload.indexOf("\"", uStart);
  outBinUrl = payload.substring(uStart, uEnd);
  outBinUrl.replace("\\u0026", "&");
  outBinUrl.trim();

  Serial.printf("[OTA] tag=%s\n", outVersion.c_str());
  Serial.printf("[OTA] bin=%s\n", outBinUrl.c_str());
  return true;
}


const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

static bool wifiConnectedForOTA() {
  return WiFi.status() == WL_CONNECTED;
}

static void wifiOff() {
  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);
  delay(50);
  Serial.println("[WiFi] OFF");
}

static bool wifiOnAndConnect(uint32_t timeoutMs = 15000) {  
  Serial.println("[WiFi] Ligando p/ OTA...");
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, IPAddress(1,1,1,1), IPAddress(8,8,8,8));
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] OK IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("[WiFi] Falhou conectar.");
  return false;
}

static void otaCheckAndUpdateESP32() {
  if (!wifiConnectedForOTA()) {
    if (!wifiOnAndConnect(15000)) {
      wifiOff();
      return;
    }
  }

  Serial.printf("[OTA] Firmware atual: v%s\n", FIRMWARE_VERSION);

  testDNS("api.github.com");
  testDNS("github.com");

  String latestVersion, binUrl;
  if (!githubGetLatest(latestVersion, binUrl)) {
    Serial.println("[OTA] Falha ao consultar GitHub.");
    return;
  }

  Serial.printf("[OTA] Última versão: %s\n", latestVersion.c_str());
  Serial.printf("[OTA] Bin: %s\n", binUrl.c_str());

  if (!isNewerVersionESP32(String(FIRMWARE_VERSION), latestVersion)) {
    Serial.println("[OTA] Já está atualizado.");
    return;
  }

  // Segurança: para motores, apaga saídas críticas

  Serial.println("[OTA] Pausando Bluetooth p/ atualizar...");
  BP32.enableNewBluetoothConnections(false); // pelo menos impede novas conexões
  rotateMotor(0,0);
  otaCheckAndUpdateESP32();
  // se não reiniciar, reabilita conexões:
  BP32.enableNewBluetoothConnections(true);

  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  Serial.println("[OTA] Iniciando update...");
  t_httpUpdate_return ret = httpUpdate.update(client, binUrl);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("[OTA] FALHOU (%d): %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      wifiOff();
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA] Sem updates.");
      wifiOff();
      break;

    case HTTP_UPDATE_OK:
      Serial.println("[OTA] OK! Reiniciando...");
      // aqui normalmente reinicia sozinho. Se não reiniciar, o Wi-Fi vai cair no reboot.
      break;
  }
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), 
    // b(), 
    // x() ->  
    // y() -> 
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    bool bNow = ctl->b();

    if (bNow && !prevB) {
        farolAltoAcendido = !farolAltoAcendido;
        farolBaixoAcendido = !farolBaixoAcendido;
        stopAcendido = !stopAcendido;
        Serial.println(">>> BOTAO B PRESSIONADO (toggle)");
    }

    prevB = bNow;

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }  

    bool otaCombo = ctl->y() && ctl->l1();

    if (otaCombo && !otaHolding) {
  otaHolding = true;
  otaPressStart = millis();
}

if (otaCombo && otaHolding && (millis() - otaPressStart >= 5000)) {
  otaHolding = false;

  Serial.println("[OTA] Triângulo+L1 5s -> checar update");

  // Segurança: para tudo antes
  rotateMotor(0, 0);
  digitalWrite(FREIO, HIGH);
  digitalWrite(STOP, HIGH);

  otaCheckAndUpdateESP32();

  // (se atualizar, reinicia; se não atualizar, Wi-Fi desliga dentro da função)
}

  if (!otaCombo) {
    otaHolding = false;
  }

    moveMotores( ctl->axisY() , ctl->axisRX(), ctl->throttle() );

    //driveTank(ctl);

    currentSentidoMovimento = ctl->axisY();
    currentSentidoDirecao = ctl->axisRX();

    updateBlinkInputs(ctl);

    updateHornFonFon(ctl);

}

void processBalanceBoard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->topLeft() > 10000) {
        // Do Something
    }

    // See "dumpBalanceBoard" for possible things to query.
    dumpBalanceBoard(ctl);
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

void handleBuzzer() {    unsigned long currentTime = millis();

    if (isHornPlaying) {
        // Parar buzina se a duração máxima foi atingida
        if (currentTime - hornStartTime >= HORN_DURATION_MS) {
            Serial.println("Horn finished.");
            ledcDetachPin(BUZZER_PIN); // Desliga o PWM
            isHornPlaying = false;
            // Garante que o freio desligue se estava ativo APENAS por causa da buzina
            if (currentSentidoMovimento == 0 && isXButtonPressed) { // Se estava parado e X pressionado
                digitalWrite(FREIO, LOW);
                freioAcendido = false;
                Serial.println("Brake Light OFF after horn pulse.");
            }
            return;
        }

        // Variação de frequência para a buzina
        if ((currentTime - hornStartTime) % HORN_SWEEP_MS == 0 && (currentTime - hornStartTime) > 0) {
            if (isHornSweepUp) {
                currentHornFrequency += 10;
                if (currentHornFrequency > HORN_FREQUENCY_HIGH) {
                    currentHornFrequency = HORN_FREQUENCY_HIGH;
                    isHornSweepUp = false; // Começa a descer
                }
            } else {
                currentHornFrequency -= 10;
                if (currentHornFrequency < HORN_FREQUENCY_LOW) {
                    currentHornFrequency = HORN_FREQUENCY_LOW;
                    isHornSweepUp = true; // Começa a subir
                }
            }
            ledcChangeFrequency(BUZZER_CHANNEL, currentHornFrequency, PWMResolution);
        }
    }
}



// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    BP32.setup(&onConnectedController, &onDisconnectedController);    

    // Configura os pinos da ponte H
    pinMode(enableRightMotor, OUTPUT);
    pinMode(enableLeftMotor, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);

    // Configura os pinos dos Leds
    pinMode(FAROL_BAIXO, OUTPUT);
    pinMode(FAROL_ALTO, OUTPUT);
    pinMode(RE, OUTPUT);
    pinMode(FREIO, OUTPUT);
    pinMode(STOP, OUTPUT);
    pinMode(SETA_DIREITA, OUTPUT);
    pinMode(SETA_ESQUERDA, OUTPUT);

    digitalWrite(FAROL_BAIXO, LOW);
    digitalWrite(FAROL_ALTO, LOW);
    digitalWrite(RE, LOW);
    digitalWrite(FREIO, LOW);
    digitalWrite(STOP, LOW);
    digitalWrite(SETA_DIREITA, LOW);
    digitalWrite(SETA_ESQUERDA, LOW);

    ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
    ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
    ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

    // Configuração do buzzer PWM
    ledcSetup(BUZZER_CHANNEL, HORN_FREQUENCY_HIGH, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, 0); // começa desligado

    

    BP32.forgetBluetoothKeys();    
    BP32.enableVirtualDevice(false);

    wifiOff();
}

void updateBlinkOutputs() {
  unsigned long now = millis();

  if (hazardOn || leftBlinkOn || rightBlinkOn) {
    if (now - lastBlinkMs >= BLINK_INTERVAL_MS) {
        lastBlinkMs = now;
        blinkState = !blinkState;
        startBlinkClick();   // <<< TIC junto com a piscada
    }
  } else {
    blinkState = false; // garante apagado quando não usa
  }

  // Prioridade: alerta
  if (hazardOn) {
    digitalWrite(SETA_ESQUERDA, blinkState);
    digitalWrite(SETA_DIREITA, blinkState);
    return;
  }

  // Setas individuais
  digitalWrite(SETA_ESQUERDA, leftBlinkOn ? blinkState : LOW);
  digitalWrite(SETA_DIREITA, rightBlinkOn ? blinkState : LOW);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    
    updateBlinkFromSteering();
    updateBlinkOutputs();
    updateBlinkClick();

    if (currentSentidoMovimento < 0){
        digitalWrite(FAROL_BAIXO, HIGH);
    } else {
        digitalWrite(FAROL_BAIXO, LOW);
    }

    if (currentSentidoMovimento < 0){
        digitalWrite(RE, HIGH);
    } else {
        digitalWrite(RE, LOW);
    }

    if (currentSentidoMovimento == 4){
        digitalWrite(FREIO, HIGH);
    } else {
        digitalWrite(FREIO, LOW);
    }

    digitalWrite(FAROL_ALTO, farolAltoAcendido ? HIGH : LOW);
    digitalWrite(FAROL_BAIXO, farolBaixoAcendido ? HIGH : LOW);
    digitalWrite(STOP, stopAcendido ? HIGH : LOW);

    handleBuzzer();

    // vTaskDelay(1);
    delay(5);
}
