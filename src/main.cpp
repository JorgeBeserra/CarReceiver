#include <Arduino.h>
#include <Bluepad32.h>

// CarReceiver ESP32 - PlatformIO integration
// This file is the PlatformIO-adapted entry point. It preserves the original logic
// from CarReceiver.ino, wiring up Bluepad32 controllers and motor/LED pins.

#define FIRMWARE_VERSION "1.0.0"

// Pinos (ajuste conforme seu hardware se necessário)
const int enableRightMotor = 14;
const int rightMotorPin1 = 13;
const int rightMotorPin2 = 12;
const int leftMotorPin1 = 27;
const int leftMotorPin2 = 32;
const int enableLeftMotor = 26;

// Pinos dos LEDs
const int FAROL = 15;
const int RE = 2;
const int FREIO = 4;
const int SETA_DIREITA = 16;
const int SETA_ESQUERDA = 17;

const int PWMFreq = 1000;
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

int motorDirection = 1;
int speed1;
int speed2;
int direction1;
int direction2;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                    ctl->getModelName().c_str(),
                    properties.vendor_id,
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

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);

  // Configuração dos pinos
  pinMode(enableRightMotor, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(FAROL, OUTPUT);
  pinMode(RE, OUTPUT);
  pinMode(FREIO, OUTPUT);
  pinMode(SETA_DIREITA, OUTPUT);
  pinMode(SETA_ESQUERDA, OUTPUT);

  // PWM setup
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

  // Initialize Bluepad32
  BP32.begin("CarReceiver ESP32");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    // Lógica de processamento pode ser adicionada aqui
  }
  delay(1);
}
