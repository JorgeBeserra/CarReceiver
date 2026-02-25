#include <Arduino.h>
#include <Bluepad32.h>

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

int moveMotores(int sentidoX, int steeringY, int throttleR2) {
  int sentido = map(sentidoX, -127, 127, -255, 255);
  int steer = map(steeringY, -127, 127, -255, 255);
  // Mapeia throttle (0-1023) para fator de velocidade (10% a 100%)
  float throttle = constrain(throttleR2, 0, 1023);
  float factor = 0.1f + (throttle / 1023.0f) * 0.9f; // 0.1 a 1.0
  int rightSpeed = (int)((abs(sentido) - steer) * factor);
  int leftSpeed = (int)((abs(sentido) + steer) * factor);
  rightSpeed = constrain(rightSpeed, 0, 255);
  leftSpeed = constrain(leftSpeed, 0, 255);
  int rDir = (sentido < 0) ? -1 : 1;
  int lDir = (sentido < 0) ? -1 : 1;
  rotateMotor(rightSpeed * rDir, leftSpeed * lDir);
  return 0;
}

void processGamepad(ControllerPtr ctl) {
  moveMotores( ctl->axisY() , ctl->axisRX(), ctl->throttle() );
  // Debug dump
  // dumpGamepad(ctl);
}

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

  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

  BP32.begin("CarReceiver ESP32");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    // Process controllers
  }
  // Simple iterate and handle anything in connected controllers
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] && myControllers[i]->isConnected()) {
      processGamepad(myControllers[i]);
    }
  }
  delay(1);
}
