#include <Bluepad32.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include "esp_wifi.h"
#include <DNSServer.h>
#include <WebServer.h>
#include <Preferences.h>
#include "ESP32MQTTClient.h"
#include <ArduinoJson.h>
#include <Update.h>
#include "esp_ota_ops.h"
#include "esp_heap_caps.h"
#include "config.h"

class Scheduler {
private:
    struct Task {
        bool enabled;
        unsigned long lastRun;
        unsigned long interval;
        std::function<void()> callback;
        String name;
    };
    
    static const int MAX_TASKS = 20;
    Task tasks[MAX_TASKS];
    int taskCount = 0;
    public:
    // Adiciona uma tarefa
    int addTask(unsigned long interval, std::function<void()> callback, String name = "") {
        if (taskCount >= MAX_TASKS) {
            Serial.println("❌ Erro: Número máximo de tarefas!");
            return -1;
        }
        
        tasks[taskCount] = {true, 0, interval, callback, name};
        if (name == "") tasks[taskCount].name = "Task_" + String(taskCount);
        
        taskCount++;
        return taskCount - 1;
    }
    
    // Executa as tarefas
    void run() {
        unsigned long currentMillis = millis();
        
        for (int i = 0; i < taskCount; i++) {
            if (!tasks[i].enabled) continue;
            
            if (currentMillis - tasks[i].lastRun >= tasks[i].interval) {
                tasks[i].lastRun = currentMillis;
                tasks[i].callback();
            }
        }
    }
    
    // Habilita/Desabilita tarefa
    void enableTask(int taskId, bool enabled) {
        if (taskId >= 0 && taskId < taskCount) {
            tasks[taskId].enabled = enabled;
        }
    }
};


Scheduler scheduler;

static const char* GITHUB_LATEST = "https://api.github.com/repos/JorgeBeserra/CarReceiver/releases/latest";

//============================================================
// ⚙️ Mapeamento de Hardware e Parâmetros
//============================================================
#define FIRMWARE_VERSION "1.0.4"

//============================================================
// 📡 Configuração WiFi com Portal
//============================================================
#define WIFI_TIMEOUT 20000          // 20 segundos para tentar conectar
#define PORTAL_TIMEOUT 300000       // 5 minutos no portal (depois reinicia)
String ap_ssid;
#define AP_PASSWORD "12345678"      // Senha do portal (mínimo 8 caracteres)
#define AP_IP "192.168.4.1"         // IP do Access Point

#define MQTT_SOCKET_TIMEOUT 15  

// DNS para captive portal
WebServer server(80);
DNSServer dnsServer;

// Preferences para salvar na flash
Preferences preferences;

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
// L2 -> Stop tem que acender as luzes de Stop e tem que parar os motores

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
const int STOP = 4;  // PWM pin for motor B
const int LANTERNA_TRASEIRA = 21;  // PWM pin for motor B
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

// Variáveis de Estado dos Controles e Funcionamentos
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
bool stopAcendido = false;
bool lanternaTraseiraAcendido = false;
bool reAcendido = false;
bool piscaEsquerdo = false;
bool piscaDireito = false;

// Buzzer
const int BUZZER_PIN = 23; 

int currentHornFrequency = HORN_FREQUENCY_HIGH;
bool isHornSweepUp = false;
uint32_t hornStartTime = 0;
bool isHornPlaying = false;

bool prevX = false;
bool hornToggle = false;
uint32_t lastHornToggle = 0;

// ====== PISCA ======

uint32_t lastBlinkMs = 0;
bool blinkState = false;

bool hazardOn = false;      // pisca alerta (triângulo)
bool leftBlinkOn = false;   // seta esquerda (L1)
bool rightBlinkOn = false;  // seta direita (R1)

bool hornOn = false;
bool autoHazardNoController = false;

bool prevTriangle = false;
bool prevL1 = false;
bool prevR1 = false;

bool prevB = false;
bool farolAltoLigado = false;

bool clickActive = false;
unsigned long clickEndMs = 0;

bool manualBlinkMode = false;   // true = L1/R1 está controlando



// trim p/ corrigir lado que puxa (ajuste fino)
float TRIM_LEFT  = 1.00f;           // ex: 0.96
float TRIM_RIGHT = 1.00f;           // ex: 1.04

// limite de direção em alta (quanto maior, MAIS reduz)
const float STEER_REDUCE = 0.55f;   // 0.35..0.70

int pwmL = 0, pwmR = 0;

static unsigned long otaPressStart = 0;
static bool otaHolding = false;

char *subscribeTopic = "foo";
char *publishTopic = "bar/bar";

bool btKnown = false;                 // já houve pareamento conhecido
bool silentHazardNoController = false; // alerta silencioso quando não há controle


// ===== CONFIGURAÇÕES MQTT =====
struct MQTTConfig {
  String server;
  int port;
  String user;
  String password;
  String topic_status;
  String topic_comandos;
  String topic_telemetria;
  bool enabled;
};


MQTTConfig mqtt_config;
ESP32MQTTClient mqttClient;
bool mqtt_connected = false;
String lastMqttMessage = "";
int lastMqttAttempt = 0;

void loadMQTTConfig() {
  preferences.begin("mqtt", true);
  mqtt_config.server = preferences.getString("server", "");
  mqtt_config.port = preferences.getInt("port", 1883);
  mqtt_config.user = preferences.getString("user", "");
  mqtt_config.password = preferences.getString("password", "");
  mqtt_config.topic_status = preferences.getString("topic_status", "carrinho/status");
  mqtt_config.topic_comandos = preferences.getString("topic_comandos", "carrinho/comandos");
  mqtt_config.topic_telemetria = preferences.getString("topic_telemetria", "carrinho/telemetria");
  mqtt_config.enabled = preferences.getBool("enabled", false);
  preferences.end();
  
  Serial.println(F("\n📂 CONFIGURAÇÕES MQTT CARREGADAS:"));
  Serial.print(F("  Servidor: "));
  Serial.println(mqtt_config.server);
  Serial.print(F("  Porta: "));
  Serial.println(mqtt_config.port);
  Serial.print(F("  Usuário: "));
  Serial.println(mqtt_config.user);
  Serial.print(F("  Status Topic: "));
  Serial.println(mqtt_config.topic_status);
  Serial.print(F("  Comandos Topic: "));
  Serial.println(mqtt_config.topic_comandos);
  Serial.print(F("  Enabled: "));
  Serial.println(mqtt_config.enabled ? "Sim" : "Não");
}

void saveMQTTConfig(MQTTConfig config) {
  preferences.begin("mqtt", false);
  preferences.putString("server", config.server);
  preferences.putInt("port", config.port);
  preferences.putString("user", config.user);
  preferences.putString("password", config.password);
  preferences.putString("topic_status", config.topic_status);
  preferences.putString("topic_comandos", config.topic_comandos);
  preferences.putString("topic_telemetria", config.topic_telemetria);
  preferences.putBool("enabled", config.server.length() > 0);
  preferences.end();
  
  mqtt_config = config;
  Serial.println(F("✅ Configurações MQTT salvas"));
}

void solicitarModoOTA() {
  preferences.begin("sys", false);
  preferences.putBool("ota_mode", true);
  preferences.end();
}

void iniciarMQTT() {
  if (!mqtt_config.enabled || mqtt_config.server.length() == 0) {
    Serial.println(F("⚠️ MQTT não configurado"));
    return;
  }
  
  Serial.println(F("\n📡 Iniciando MQTT com configurações salvas..."));
  
  // Monta URI no formato correto
  String uri = "mqtt://" + mqtt_config.server + ":" + String(mqtt_config.port);
  
  // Configura cliente MQTT
  mqttClient.setURI(uri.c_str(), 
                    mqtt_config.user.c_str(), 
                    mqtt_config.password.c_str());
  mqttClient.setMqttClientName("BattleCar");

  
  Serial.println(F("✅ MQTT iniciado"));
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  lastMqttMessage = "[" + String(topic) + "] " + message;
  Serial.printf("📩 MQTT: %s\n", lastMqttMessage.c_str());
  
  // Processa comandos aqui
  if (String(topic) == mqtt_config.topic_comandos) {
    if (message == "buzina") {
      ledcWrite(BUZZER_CHANNEL, HORN_DUTY_ON);
      delay(500);
      ledcWrite(BUZZER_CHANNEL, HORN_DUTY_OFF);
    }
    else if (message == "status") {
      enviarStatusMQTT();
    }
  }
}

void enviarStatusMQTT() {
  if (!mqttClient.isConnected()) return;

  String statusTopic = "tele/battlecar_" + getMacSuffix() + "/STATE";
  
  String json = "{";
  json += "\"versao\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"farol_baixo\":" + String(farolBaixoAcendido ? "true" : "false") + ",";
  json += "\"farol_alto\":" + String(farolAltoAcendido ? "true" : "false") + ",";
  json += "\"seta_esquerda\":" + String(leftBlinkOn ? "true" : "false") + ",";
  json += "\"seta_direita\":" + String(rightBlinkOn ? "true" : "false") + ",";
  json += "\"pisca_alerta\":" + String(hazardOn ? "true" : "false") + ",";
  json += "\"movimento\":" + String(currentSentidoMovimento) + ",";
  json += "\"direcao\":" + String(currentSentidoDirecao);
  json += "}";
  
  mqttClient.publish(statusTopic.c_str(), json.c_str(), 0, false);
  Serial.println(F("📤 Status publicado"));
}


void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("⚠️ WiFi desconectado, MQTT não será reconectado agora"));
    mqtt_connected = false;
    return;
  }

  if (!mqtt_config.enabled || mqtt_config.server.length() == 0) {
    Serial.println(F("⚠️ MQTT não configurado"));
    return;
  }

  if (mqttClient.isConnected()) {
    return;
  }

  if (millis() - lastMqttAttempt < 5000) return;  // Tenta a cada 5s
  
  lastMqttAttempt = millis();
  
  Serial.println(F("\n📡 Tentando reconectar MQTT..."));


  // Monta URI com as configurações atuais
  String uri = "mqtt://" + mqtt_config.server + ":" + String(mqtt_config.port);

  Serial.println(uri);
  
  String clientId = "BattleCar-" + String(random(0xffff), HEX);

  // Configura cliente MQTT
  mqttClient.setURI(uri.c_str(), 
                    mqtt_config.user.c_str(), 
                    mqtt_config.password.c_str());
  mqttClient.setMqttClientName("BattleCar");

  mqttClient.loopStart();

  // Pequeno delay para verificar se conectou
  delay(500);

  
  if (mqttClient.isConnected()) {
    Serial.println(F("✅ Reconexão bem sucedida!"));
  } else {
    Serial.println(F("❌ Falha na reconexão"));
    mqtt_connected = false;
  }
}

// Variáveis de controle
String savedSSID = "";
String savedPassword = "";
bool portalAtivo = false;
bool wifiConectado = false;
unsigned long portalStartTime = 0;
unsigned long lastWifiAttempt = 0;
int wifiAttempts = 0;

// HTML mínimo e eficiente
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>Configurar WiFi</title>
  <style>
    body{font-family:Arial;text-align:center;margin:20px;background:#f0f0f0}
    .container{background:white;padding:20px;border-radius:10px;max-width:400px;margin:0 auto}
    input,select,button{width:100%;padding:12px;margin:8px 0;border:1px solid #ddd;border-radius:5px;box-sizing:border-box}
    button{background:#4CAF50;color:white;border:none;cursor:pointer}
    button:hover{background:#45a049}
    #status{margin-top:15px;padding:10px;border-radius:5px;display:none}
    .success{background:#d4edda;color:#155724;display:block}
    .error{background:#f8d7da;color:#721c24;display:block}
  </style>
</head>
<body>
  <div class="container">
    <h2>📶 Configurar WiFi</h2>
    <form id="wifiForm" onsubmit="connect(event)">
      <select id="ssid" required>
        <option value="">Selecione uma rede</option>
      </select>
      <input type="password" id="password" placeholder="Senha">
      <button type="submit">Conectar</button>
    </form>
    <div id="status"></div>
  </div>
  <script>
    fetch('/scan').then(r=>r.json()).then(networks=>{
      const select = document.getElementById('ssid');
      networks.sort((a,b)=>b.rssi-a.rssi).forEach(n=>{
        const option = document.createElement('option');
        option.value = n.ssid;
        option.text = n.ssid + (n.secure ? ' 🔒' : ' 🔓');
        select.appendChild(option);
      });
    });
    function connect(e){
      e.preventDefault();
      const ssid = document.getElementById('ssid').value;
      const pwd = document.getElementById('password').value;
      if(!ssid) return;
      document.querySelector('button').disabled = true;
      fetch('/connect', {
        method:'POST',
        headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:`ssid=${encodeURIComponent(ssid)}&password=${encodeURIComponent(pwd)}`
      }).then(()=>{
        showStatus('Conectando...', 'success');
        setTimeout(()=>window.location.href='/', 2000);
      });
    }
    function showStatus(msg,type){
      const div = document.getElementById('status');
      div.className = type;
      div.textContent = msg;
    }
  </script>
</body>
</html>
)rawliteral";

// HTML para configuração MQTT (adicione após o index_html existente)
const char mqtt_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>Configurar MQTT - Carrinho</title>
  <style>
    body{font-family:Arial;text-align:center;margin:20px;background:#f0f0f0}
    .container{background:white;padding:20px;border-radius:10px;max-width:400px;margin:0 auto}
    input,button{width:100%;padding:12px;margin:8px 0;border:1px solid #ddd;border-radius:5px;box-sizing:border-box}
    button{background:#2196F3;color:white;border:none;cursor:pointer}
    button:hover{background:#1976D2}
    button.danger{background:#f44336}
    button.danger:hover{background:#d32f2f}
    .status{margin-top:15px;padding:10px;border-radius:5px;display:none}
    .success{background:#d4edda;color:#155724;display:block}
    .error{background:#f8d7da;color:#721c24;display:block}
    .info{background:#d1ecf1;color:#0c5460;display:block}
    .header{display:flex;justify-content:space-between;align-items:center}
    .title{color:#2196F3}
    .card{background:#f8f9fa;border-radius:8px;padding:15px;margin:15px 0;text-align:left}
    .card h3{margin-top:0;color:#666}
    .info-text{color:#666;font-size:12px;margin-top:20px}
    .tab{overflow:hidden;border:1px solid #ccc;background-color:#f1f1f1;border-radius:5px 5px 0 0}
    .tab button{background-color:inherit;float:left;border:none;outline:none;cursor:pointer;padding:12px 16px;transition:0.3s;width:50%}
    .tab button:hover{background-color:#ddd}
    .tab button.active{background-color:#2196F3;color:white}
    .tabcontent{display:none;padding:20px;border:1px solid #ccc;border-top:none;border-radius:0 0 5px 5px}
    .version-row {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-top: 4px;
}

.btn-update {
  background: #2d8cff;
  color: white;
  border: none;
  border-radius: 6px;
  padding: 4px 10px;
  font-size: 13px;
  cursor: pointer;
}

.btn-update:hover {
  background: #1f6ed4;
}

#updateStatus {
  font-size: 13px;
  margin-top: 6px;
  color: #555;
}
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h2 class="title">⚙️ Configurar MQTT</h2>
      <div>🔌 <span id="mqttStatus">Desconectado</span></div>
    </div>
    
    <div class="tab">
      <button class="tablinks active" onclick="openTab(event, 'Config')">Configurações</button>
      <button class="tablinks" onclick="openTab(event, 'Status')">Status</button>
    </div>

    <div id="Config" class="tabcontent" style="display:block">
      <form id="mqttForm" onsubmit="saveMQTT(event)">
        <div class="card">
          <h3>📡 Servidor MQTT</h3>
          <input type="text" id="server" placeholder="IP do Broker (ex: 192.168.10.100)" required>
          <input type="text" id="port" placeholder="Porta (padrão: 1883)" value="1883" required>
        </div>
        
        <div class="card">
          <h3>🔐 Autenticação</h3>
          <input type="text" id="user" placeholder="Usuário">
          <input type="password" id="password" placeholder="Senha">
        </div>
        
        <div class="card">
          <h3>📋 Tópicos</h3>
          <input type="text" id="topic_status" placeholder="Tópico Status" value="/tele/battlecar_%/status">
          <input type="text" id="topic_comandos" placeholder="Tópico Comandos" value="/carrinho/comandos">
          <input type="text" id="topic_telemetria" placeholder="Tópico Telemetria" value="/carrinho/telemetria">
        </div>
        
        <button type="submit">💾 Salvar e Testar Conexão</button>
      </form>
      
      <button class="danger" onclick="testMQTT()" style="margin-top:10px">🔌 Testar Conexão</button>
      <button class="danger" onclick="clearMQTT()" style="margin-top:10px">🗑️ Limpar Configurações</button>
    </div>

    <div id="Status" class="tabcontent">
      <div class="card">
        <h3>📊 Informações do Carrinho</h3>
        <p><strong>IP:</strong> <span id="ipInfo">carregando...</span></p>
        <p><strong>MAC:</strong> <span id="macInfo">carregando...</span></p>
        <p><strong>WiFi:</strong> <span id="wifiInfo">carregando...</span></p>
        <div class="version-row">
          <strong>Versão:</strong>
          <span id="versaoInfo">carregando...</span>
          <button class="btn-update" id="btnCheckUpdate" onclick="checkUpdate()">Checar</button>
          <button class="btn-update" id="btnDoUpdate" onclick="doUpdate()" style="display:none;background:#28a745;">Atualizar</button>
        </div>
        <div id="updateStatus"></div>
      </div>

      <div class="card">
        <h3>🎮 Controle Bluetooth</h3>
        <p><strong>Pareado:</strong> <span id="btPaired">-</span></p>
        <p><strong>Conectado:</strong> <span id="btConnected">-</span></p>
        <p><strong>Modelo:</strong> <span id="btName">-</span></p>
        <p><strong>Bateria:</strong> <span id="btBattery">-</span></p>
        <button class="danger" onclick="forgetBT()">🗑️ Esquecer pareamento</button>
      </div>
      
      <div class="card">
        <h3>📡 Status MQTT</h3>
        <p><strong>Servidor:</strong> <span id="mqttServer">-</span></p>
        <p><strong>Usuário:</strong> <span id="mqttUser">-</span></p>
        <p><strong>Conexão:</strong> <span id="mqttConn">Desconectado</span></p>
        <p><strong>Última mensagem:</strong> <span id="lastMsg">-</span></p>
      </div>
      
      <div class="card">
        <h3>🚗 Status do Carrinho</h3>
        <p><strong>Faróis:</strong> <span id="farois">-</span></p>
        <p><strong>Setas:</strong> <span id="setas">-</span></p>
        <p><strong>Movimento:</strong> <span id="movimento">-</span></p>
      </div>
    </div>

    <div id="status" class="status"></div>
    <div class="info-text">
      Após salvar, o carrinho tentará conectar ao broker MQTT automaticamente
    </div>
  </div>

  <script>
    function openTab(evt, tabName) {
      var i, tabcontent, tablinks;
      tabcontent = document.getElementsByClassName("tabcontent");
      for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
      }
      tablinks = document.getElementsByClassName("tablinks");
      for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
      }
      document.getElementById(tabName).style.display = "block";
      evt.currentTarget.className += " active";
    }

    // Carrega configurações ao iniciar
    window.onload = function() {
      loadConfig();
      loadStatus();
      setInterval(loadStatus, 5000); // Atualiza status a cada 5s
    };

    function loadConfig() {
      fetch('/mqtt_config')
        .then(r => r.json())
        .then(config => {
          document.getElementById('server').value = config.server || '';
          document.getElementById('port').value = config.port || '1883';
          document.getElementById('user').value = config.user || '';
          document.getElementById('topic_status').value = config.topic_status || 'carrinho/status';
          document.getElementById('topic_comandos').value = config.topic_comandos || 'carrinho/comandos';
          document.getElementById('topic_telemetria').value = config.topic_telemetria || 'carrinho/telemetria';
          
          document.getElementById('mqttServer').textContent = config.server || '-';
          document.getElementById('mqttUser').textContent = config.user || 'anônimo';
        });
    }

    function loadStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(status => {
          document.getElementById('ipInfo').textContent = status.ip || '-';
          document.getElementById('macInfo').textContent = status.mac || '-';
          document.getElementById('wifiInfo').textContent = status.wifi || '-';
          document.getElementById('versaoInfo').textContent = status.versao || '-';

          document.getElementById('btPaired').textContent = status.bt_paired ? 'Sim' : 'Não';
          document.getElementById('btConnected').textContent = status.bt_connected ? 'Sim' : 'Não';
          document.getElementById('btName').textContent = status.bt_name || '-';
          document.getElementById('btBattery').textContent = status.bt_battery || '-';
          
          document.getElementById('mqttConn').textContent = status.mqtt_connected ? 'Conectado' : 'Desconectado';
          document.getElementById('mqttConn').style.color = status.mqtt_connected ? '#4CAF50' : '#f44336';
          document.getElementById('mqttStatus').textContent = status.mqtt_connected ? 'Conectado' : 'Desconectado';
          document.getElementById('mqttStatus').style.color = status.mqtt_connected ? '#4CAF50' : '#f44336';
          
          document.getElementById('farois').textContent = status.farois || '-';
          document.getElementById('setas').textContent = status.setas || '-';
          document.getElementById('movimento').textContent = status.movimento || '-';

          if (status.last_message) {
            document.getElementById('lastMsg').textContent = status.last_message;
          }
        });
    }

    function saveMQTT(event) {
      event.preventDefault();
      
      const config = {
        server: document.getElementById('server').value,
        port: document.getElementById('port').value,
        user: document.getElementById('user').value,
        password: document.getElementById('password').value,
        topic_status: document.getElementById('topic_status').value,
        topic_comandos: document.getElementById('topic_comandos').value,
        topic_telemetria: document.getElementById('topic_telemetria').value
      };

      fetch('/mqtt_save', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(config)
      })
      .then(r => r.text())
      .then(data => {
        showStatus('✅ Configurações salvas! Testando conexão...', 'success');
        setTimeout(() => testMQTT(), 1000);
      });
    }

    function testMQTT() {
      showStatus('🔌 Testando conexão MQTT...', 'info');
      fetch('/mqtt_test')
        .then(r => r.text())
        .then(data => {
          if (data == 'OK') {
            showStatus('✅ Conexão MQTT bem sucedida!', 'success');
          } else {
            showStatus('❌ Falha na conexão MQTT', 'error');
          }
          loadStatus();
        });
    }

    function clearMQTT() {
      if (confirm('Tem certeza que deseja limpar todas as configurações MQTT?')) {
        fetch('/mqtt_clear', {method: 'POST'})
          .then(r => r.text())
          .then(data => {
            showStatus('🗑️ Configurações apagadas', 'success');
            loadConfig();
          });
      }
    }

    function showStatus(msg, type) {
      const div = document.getElementById('status');
      div.className = 'status ' + type;
      div.textContent = msg;
    }
  </script>
</body>
</html>
)rawliteral";

void startConfigPortal() {
  
  String macSuffix = getMacSuffix();

  ap_ssid = "BattleCar_" + macSuffix;

  WiFi.mode(WIFI_AP);

  // ✅ CONFIGURAÇÕES CRÍTICAS PARA IPHONE
  WiFi.softAPConfig(IPAddress(192,168,4,1), 
                    IPAddress(192,168,4,1), 
                    IPAddress(255,255,255,0));
  
  // Parâmetros importantes para compatibilidade
  bool result = WiFi.softAP(ap_ssid.c_str(), AP_PASSWORD);
  
  Serial.print(F("softAP() retornou: "));
  Serial.println(result ? "✅SUCESSO" : "❌FALHA");

  if (!result) {
    Serial.println(F("❌ ERRO CRÍTICO: Falha ao criar Access Point!"));
    return;
  }

  Serial.println(F("⏳ Aguardando WiFi estabilizar..."));
  delay(500);  // CRÍTICO: Dá tempo para o WiFi iniciar completamente
  
  Serial.println(F("🌐 Iniciando DNS Server..."));
  dnsServer.start(53, "*", WiFi.softAPIP());
  delay(100);

   Serial.println(F("📡 Configurando servidor web..."));

  // Rota principal
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });
  
  // Escaneamento de redes
  server.on("/scan", HTTP_GET, []() {
    Serial.println(F("📡 Scan de redes iniciado..."));

    WiFi.scanDelete();
    
    // Inicia scan
    int n = WiFi.scanComplete();
    if (n == -2) {
      WiFi.scanNetworks(true);
      
      // Aguarda scan completar (até 5 segundos)
      int timeout = 0;
      while (WiFi.scanComplete() == -1 && timeout < 50) {
        delay(100);
        timeout++;
      }
      n = WiFi.scanComplete();
    }
    
    String json = "[";
    
    if (n > 0) {
      for (int i = 0; i < n; i++) {
        if (i) json += ",";
        
        String ssid = WiFi.SSID(i);
        ssid.replace("\"", "\\\"");  // Escapa aspas
        
        json += "{\"ssid\":\"" + ssid + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
        json += "\"secure\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? "true" : "false") + "}";
      }
      WiFi.scanDelete();
    }
    
    json += "]";
    server.send(200, "application/json", json);
    Serial.printf("✅ %d redes enviadas\n", n > 0 ? n : 0);
  });
  
  // Recebe credenciais
  server.on("/connect", HTTP_POST, []() {
    if (server.hasArg("ssid") && server.hasArg("password")) {
      String ssid = server.arg("ssid");
      String pass = server.arg("password");
      
      preferences.begin("wifi", false);
      preferences.putString("ssid", ssid);
      preferences.putString("password", pass);
      preferences.end();
      
      server.send(200, "text/plain", "OK");
      delay(100);
      ESP.restart();
    } else {
      server.send(400, "text/plain", "BAD");
    }
  });
  
  // Captive portal
  server.onNotFound([]() {
    server.sendHeader("Location", "http://" + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
  });
  
  server.begin();
  delay(100);
  // ===== 7. VERIFICA SE TUDO ESTÁ OK =====
  Serial.println(F("\n🔍 VERIFICAÇÃO DO PORTAL:"));
  Serial.print(F("📱 Rede: "));
  Serial.println(ap_ssid);
  Serial.print(F("🔑 Senha: "));
  Serial.println(AP_PASSWORD);
  Serial.print(F("🌐 IP do AP: "));
  Serial.println(WiFi.softAPIP());
  Serial.print(F("📡 DNS Server: "));
  
  Serial.print(F("🌍 Servidor Web: "));
  
  Serial.println(F("================================\n"));

  portalAtivo = true;
  portalStartTime = millis();
}


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
    Serial.println(F("FALHOU"));
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

String getMacAddress() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char macStr[18] = {0};
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

String getMacSuffix() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char macStr[18] = {0};
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr).substring(6); // Retorna os últimos 6 dígitos
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

void updateIdleHazardNoController() {
  bool connected = hasConnectedController();

  if (!connected) {
    silentHazardNoController = true;

    // só ativa o alerta automático se o usuário não ligou seta manual
    if (!hazardOn && !leftBlinkOn && !rightBlinkOn) {
      hazardOn = true;
    }
  } else {
    if (silentHazardNoController) {
      silentHazardNoController = false;

      // desliga apenas o alerta automático
      if (hazardOn && !manualBlinkMode) {
        hazardOn = false;
      }
    }
  }
}

int getConnectedControllerBatteryRaw() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected()) {
      return myController->battery();
    }
  }
  return 0;  // desconhecido
}

String getConnectedControllerBatteryText() {
  int raw = getConnectedControllerBatteryRaw();

  if (raw <= 0) {
    return "Desconhecida";
  }

  int percent = map(raw, 1, 255, 0, 100);
  percent = constrain(percent, 0, 100);

  return String(percent) + "%";
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
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

            setControllerColorByCarMac(ctl);
            saveBTState(true);

            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println(F("CALLBACK: Controller connected, but could not found empty slot"));
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

  if (hazardOn || silentHazardNoController) return;
  if (manualBlinkMode) return;

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

bool githubGetLatest(String &latestVersion, String &binUrl) {

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(12000);

  Serial.printf("[OTA] GET: %s\n", GITHUB_LATEST);

  HTTPClient https;
  https.useHTTP10(true);      // importante
  https.setReuse(false);      // importante
  https.setTimeout(15000);

  Serial.printf("[MEM] Heap antes HTTP: %u\n", ESP.getFreeHeap());

  if (!https.begin(client, GITHUB_LATEST)) {
    Serial.println(F("[OTA] https.begin() falhou"));
    return false;
  }

  https.addHeader("User-Agent", "ESP32-OTA-Updater");
  https.addHeader("Accept", "application/vnd.github+json");
  https.addHeader("Connection", "close");

  const char* headerKeys[] = {"Content-Length", "Transfer-Encoding", "Content-Type"};
  https.collectHeaders(headerKeys, 3);

  int httpCode = https.GET();
  Serial.printf("[OTA] HTTP code: %d\n", httpCode);
  Serial.printf("[OTA] Content-Length: %s\n", https.header("Content-Length").c_str());
  Serial.printf("[OTA] Transfer-Encoding: %s\n", https.header("Transfer-Encoding").c_str());
  Serial.printf("[OTA] Content-Type: %s\n", https.header("Content-Type").c_str());

  if (httpCode != HTTP_CODE_OK) {
    String err = https.getString();
    Serial.println("[OTA] Resposta de erro:");
    Serial.println(err);
    https.end();
    return false;
  }

  StaticJsonDocument<256> filter;
  filter["tag_name"] = true;
  filter["assets"][0]["name"] = true;
  filter["assets"][0]["browser_download_url"] = true;

  DynamicJsonDocument doc(1024);

  //WiFiClient *stream = https.getStreamPtr();
  DeserializationError error = deserializeJson(
    doc,
    https.getStream(),
    DeserializationOption::Filter(filter)
  );

  if (error) {
    Serial.print("[OTA] Erro JSON: ");
    Serial.println(error.c_str());
    https.end();
    return false;
  }

  latestVersion = doc["tag_name"].as<String>();

  if (!doc.containsKey("tag_name")) {
    Serial.println("[OTA] tag_name nao encontrado");
    https.end();
    return false;
  }

  latestVersion = String((const char*)doc["tag_name"]);
  Serial.printf("[OTA] tag_name: %s\n", latestVersion.c_str());

  JsonArray assets = doc["assets"].as<JsonArray>();
  for (JsonObject asset : assets) {
    String name = asset["name"] | "";
    String url  = asset["browser_download_url"] | "";

    Serial.printf("[OTA] Asset: %s\n", name.c_str());

    if (name.endsWith(".bin")) {
      binUrl = url;
      Serial.printf("[OTA] BIN: %s\n", binUrl.c_str());
      https.end();
      return true;
    }
  }

  Serial.println("[OTA] Nenhum arquivo .bin encontrado");
  https.end();
  return false;
}

static bool wifiConnectedForOTA() {
  return WiFi.status() == WL_CONNECTED;
}

static void wifiOff() {
  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);
  delay(50);
  Serial.println(F("[WiFi] OFF"));
}

static bool wifiOnAndConnect(uint32_t timeoutMs = 15000) {  
  Serial.println(F("[WiFi] Ligando p/ OTA..."));

  if (savedSSID.length() == 0) {
    Serial.println(F("[WiFi] Sem credenciais salvas."));
    return false;
  }

  Serial.print(F("[OTA] Firmware atual: "));
  Serial.printf(FIRMWARE_VERSION);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, IPAddress(1,1,1,1), IPAddress(8,8,8,8));
  WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WiFi] OK IP: "));
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println(F("[WiFi] Falhou conectar."));
  return false;
}
bool otaEmAndamento = false;

void finalizarOTA() {
  otaEmAndamento = false;
}

static void otaCheckAndUpdateESP32() {
  

  if (otaEmAndamento) {
    Serial.println(F("[OTA] Já em andamento, ignorando nova chamada."));
    return;
  }
  return false;
}

  otaEmAndamento = true;

  if (!wifiConnectedForOTA()) {
    if (!wifiOnAndConnect(15000)) {
      wifiOff();
      finalizarOTA();
      return;
    }
  }

  Serial.print(F("[OTA] Firmware atual: "));
  Serial.println(FIRMWARE_VERSION);

  testDNS("api.github.com");
  testDNS("github.com");

  String latestVersion, binUrl;
  if (!githubGetLatest(latestVersion, binUrl)) {
    Serial.println(F("[OTA] Falha ao consultar GitHub."));
    return;
  }

  Serial.print(F("[OTA] Última versão: "));
  Serial.println(latestVersion.c_str());
  Serial.print(F("[OTA] Bin: "));
  Serial.println(binUrl.c_str());

  if (!isNewerVersionESP32(String(FIRMWARE_VERSION), latestVersion)) {
    Serial.println(F("[OTA] Já está atualizado."));
    finalizarOTA();
    return;
  }

  // Segurança: para motores, apaga saídas críticas
  rotateMotor(0,0);

  Serial.println(F("[OTA] Pausando Bluetooth p/ atualizar..."));
  BP32.enableNewBluetoothConnections(false); // pelo menos impede novas conexões
  delay(50);
  // se não reiniciar, reabilita conexões:
 

  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  Serial.println(F("[OTA] Iniciando update..."));
  t_httpUpdate_return ret = httpUpdate.update(client, binUrl);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("[OTA] FALHOU (%d): %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      wifiOff();
      BP32.enableNewBluetoothConnections(true);
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F("[OTA] Sem updates."));
      BP32.enableNewBluetoothConnections(true);
      wifiOff();
      break;

    case HTTP_UPDATE_OK:
      Serial.println(F("[OTA] OK! Reiniciando..."));
      // aqui normalmente reinicia sozinho. Se não reiniciar, o Wi-Fi vai cair no reboot.
      break;
  }

  Serial.printf("[OTA] Total gravado: %u bytes\n", totalWritten);

  if (totalWritten != (size_t)contentLength) {
    Serial.println("[OTA] Tamanho gravado diferente do esperado");
    Update.abort();
    http.end();
    return;
  }

  if (!Update.end(true)) {
    Serial.printf("[OTA] Update.end falhou: %s\n", Update.errorString());
    http.end();
    return;
  }

  if (!Update.isFinished()) {
    Serial.println("[OTA] Update não finalizado corretamente");
    http.end();
    return;
  }

  http.end();
  Serial.println("[OTA] Atualização concluída. Reiniciando...");
  delay(500);
  ESP.restart();
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
        Serial.println(F(">>> BOTAO B PRESSIONADO (toggle)"));
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

      Serial.println(F("[OTA] Triângulo+L1 5s -> checar update"));

      // Segurança: para tudo antes
      rotateMotor(0, 0);
      digitalWrite(STOP, HIGH);
      digitalWrite(LANTERNA_TRASEIRA, HIGH);

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

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println(F("Unsupported controller"));
            }
        }
    }
}

void handleBuzzer() {    unsigned long currentTime = millis();

    if (isHornPlaying) {
        // Parar buzina se a duração máxima foi atingida
        if (currentTime - hornStartTime >= HORN_DURATION_MS) {
            Serial.println(F("Horn finished."));
            ledcDetachPin(BUZZER_PIN); // Desliga o PWM
            isHornPlaying = false;
            // Garante que o stop desligue se estava ativo APENAS por causa da buzina
            if (currentSentidoMovimento == 0 && isXButtonPressed) { // Se estava parado e X pressionado
                digitalWrite(STOP, LOW);
                stopAcendido = false;
                Serial.println(F("Brake Light OFF after horn pulse."));
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


bool loadWiFiCredentials() {
    preferences.begin("wifi", true);
    savedSSID = preferences.getString("ssid", "");
    savedPassword = preferences.getString("password", "");
    preferences.end();
    
    if (savedSSID.length() > 0) {
        Serial.print(F("📂 Credenciais carregadas: "));
        Serial.println(savedSSID.c_str());
        return true;
    }
    Serial.println(F("📂 Nenhuma credencial salva"));
    return false;
}


// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200); 
 
    bool otaMode = bootInOtaMode();

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
    pinMode(STOP, OUTPUT);
    pinMode(LANTERNA_TRASEIRA, OUTPUT);
    pinMode(SETA_DIREITA, OUTPUT);
    pinMode(SETA_ESQUERDA, OUTPUT);

    digitalWrite(FAROL_BAIXO, LOW);
    digitalWrite(FAROL_ALTO, LOW);
    digitalWrite(RE, LOW);
    digitalWrite(STOP, LOW);
    digitalWrite(LANTERNA_TRASEIRA, LOW);
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

    ledcSetup(stopPWMChannel, PWMFreq, PWMResolution);
    ledcAttachPin(STOP, stopPWMChannel);

    ledcSetup(lanternaPWMChannel, PWMFreq, PWMResolution);
    ledcAttachPin(LANTERNA_TRASEIRA, lanternaPWMChannel);

    ledcWrite(stopPWMChannel, 0);
    ledcWrite(lanternaPWMChannel, 0);



    // 1. Tenta carregar credenciais salvas
    bool hasCredentials = loadWiFiCredentials();

    // 2. Se tiver credenciais, tenta conectar
    if (hasCredentials) {
        Serial.println(F("📡 Tentando conectar ao WiFi salvo..."));
        WiFi.mode(WIFI_STA);
        WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

        // Aguarda conexão por WIFI_TIMEOUT
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < WIFI_TIMEOUT / 500) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        Serial.println();
        
        // Verifica se conectou
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print(F("✅ WiFi conectado! IP: "));
            Serial.println(WiFi.localIP());
            Serial.println(F("\n📡 DIAGNÓSTICO DE REDE:"));
            Serial.print(F("  IP: "));
            Serial.println(WiFi.localIP());
            Serial.print(F("  Gateway: "));
            Serial.println(WiFi.gatewayIP());
            Serial.print(F("  Máscara: "));
            Serial.println(WiFi.subnetMask());
            Serial.print(F("  DNS: "));
            Serial.println(WiFi.dnsIP());
            
            // Teste 1: Gateway está configurado?
            if (WiFi.gatewayIP() == IPAddress(0,0,0,0)) {
              Serial.println(F("  ❌ GATEWAY NÃO CONFIGURADO!"));
            } else {
              Serial.println(F("  ✅ Gateway configurado"));
            }
            wifiConectado = true;
            loadMQTTConfig();
            iniciarMQTT();

        } else {
            Serial.println(F("❌ Falha na conexão WiFi"));
            wifiConectado = false;
        }
        
    }

    // Bluepad32
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    BP32.setup(&onConnectedController, &onDisconnectedController);       
    BP32.enableVirtualDevice(false);

    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    // 3. Se NÃO conectou, inicia o portal
    if (!wifiConectado) {
        Serial.println(F("🔴 Iniciando portal de configuração WiFi..."));
        startConfigPortal();
    } 

    if (wifiConectado) {
          loadMQTTConfig();

           // Adiciona rotas para configuração MQTT
      server.on("/mqtt", HTTP_GET, []() {
      server.send(200, "text/html", mqtt_html);
  });
  
  server.on("/mqtt_config", HTTP_GET, []() {
    String json = "{";
    json += "\"server\":\"" + mqtt_config.server + "\",";
    json += "\"port\":" + String(mqtt_config.port) + ",";
    json += "\"user\":\"" + mqtt_config.user + "\",";
    json += "\"topic_status\":\"" + mqtt_config.topic_status + "\",";
    json += "\"topic_comandos\":\"" + mqtt_config.topic_comandos + "\",";
    json += "\"topic_telemetria\":\"" + mqtt_config.topic_telemetria + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.on("/mqtt_save", HTTP_POST, []() {
    String json = server.arg("plain");
    
    // Parse JSON
    int serverStart = json.indexOf("\"server\":\"") + 10;
    int serverEnd = json.indexOf("\"", serverStart);
    String server_ip = json.substring(serverStart, serverEnd);
    
    int portStart = json.indexOf("\"port\":\"") + 8;  // +8 porque tem aspas
    int portEnd = json.indexOf("\"", portStart);
    String portStr = json.substring(portStart, portEnd);
    int port = portStr.toInt();  // Converte string para int
    
    int userStart = json.indexOf("\"user\":\"") + 8;
    int userEnd = json.indexOf("\"", userStart);
    String user = json.substring(userStart, userEnd);
    
    int passStart = json.indexOf("\"password\":\"") + 12;
    int passEnd = json.indexOf("\"", passStart);
    String pass = json.substring(passStart, passEnd);
    
    MQTTConfig newConfig;
    newConfig.server = server_ip;
    newConfig.port = port;
    newConfig.user = user;
    newConfig.password = pass;
    newConfig.topic_status = "tele/battlecar_" + getMacSuffix() + "/STATE";
    newConfig.topic_comandos = "cmnd/battlecar_" + getMacSuffix() + "/COMMAND";
    newConfig.topic_telemetria = "carrinho/telemetria";
    
    saveMQTTConfig(newConfig);
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/mqtt_test", HTTP_GET, []() {
    reconnectMQTT();
    server.send(200, "text/plain", mqtt_connected ? "OK" : "FAIL");
  });
  
  server.on("/status", HTTP_GET, []() {
    bool btConnected = hasConnectedController();
    String btName = getConnectedControllerName();
    int btBatteryRaw = getConnectedControllerBatteryRaw();
    String btBattery = getConnectedControllerBatteryText();

    String json = "{";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"mac\":\"" + WiFi.macAddress() + "\",";
    json += "\"wifi\":\"" + String(WiFi.SSID()) + "\",";
    json += "\"versao\":\"" + String(FIRMWARE_VERSION) + "\",";
    json += "\"bt_paired\":" + String(btKnown ? "true" : "false") + ",";
    json += "\"bt_connected\":" + String(btConnected ? "true" : "false") + ",";
    json += "\"bt_name\":\"" + btName + "\",";
    json += "\"bt_battery_raw\":" + String(btBatteryRaw) + ",";
    json += "\"bt_battery\":\"" + btBattery + "\",";
    json += "\"mqtt_connected\":" + String(mqtt_connected ? "true" : "false") + ",";
    json += "\"farois\":\"" + String(farolBaixoAcendido ? "Baixo" : (farolAltoAcendido ? "Alto" : "Desligado")) + "\",";
    json += "\"setas\":\"" + String(leftBlinkOn ? "Esquerda" : (rightBlinkOn ? "Direita" : "Desligado")) + "\",";
    json += "\"movimento\":\"" + String(currentSentidoMovimento) + "\",";
    json += "\"last_message\":\"" + lastMqttMessage + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/ota_update", HTTP_GET, []() {
    solicitarModoOTA();
    server.send(200, "text/plain", "Atualização solicitada. Verifique o serial.");
    delay(300);
    ESP.restart();
  });

    otaRequestPending = true;

    server.send(200, "text/plain", "OTA solicitado. Verifique o serial.");
    
  });
  
  server.begin();  // Reinicia servidor com novas rotas
  }

}

void updateBlinkOutputs() {
  unsigned long now = millis();

  if (hazardOn || leftBlinkOn || rightBlinkOn) {
    if (now - lastBlinkMs >= BLINK_INTERVAL_MS) {
      lastBlinkMs = now;
      blinkState = !blinkState;

      // só toca tic se NÃO estiver no alerta automático silencioso
      if (!silentHazardNoController) {
        startBlinkClick();
      }
    }
  } else {
    blinkState = false;
  }

  if (hazardOn) {
    digitalWrite(SETA_ESQUERDA, blinkState);
    digitalWrite(SETA_DIREITA, blinkState);
    return;
  }

  digitalWrite(SETA_ESQUERDA, leftBlinkOn ? blinkState : LOW);
  digitalWrite(SETA_DIREITA, rightBlinkOn ? blinkState : LOW);
}

// Arduino loop function. Runs in CPU 1.
void loop() {

    if (portalAtivo) {
      dnsServer.processNextRequest();
      server.handleClient();
      delay(10); // Delay de sobrecarga
      return;
    }

    if (wifiConectado) {
        server.handleClient();  
        
        // Gerencia MQTT
        if (mqtt_config.enabled && WiFi.status() == WL_CONNECTED) {
            if (!mqttClient.isConnected()) {
                reconnectMQTT();
            }
        }
    }

 
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    
    //autoHazardNoController = !hasConnectedController();
    
    updateIdleHazardNoController();
    updateBlinkFromSteering();
    updateBlinkOutputs();
    updateBlinkClick();

    if (currentSentidoMovimento < 0){
        digitalWrite(RE, HIGH);
    } else {
        digitalWrite(RE, LOW);
    }

    digitalWrite(FAROL_ALTO, farolAltoAcendido ? HIGH : LOW);
    digitalWrite(FAROL_BAIXO, farolBaixoAcendido ? HIGH : LOW);

    updateStopPulseOnRelease();
    updateLuzesTraseiras();

    handleBuzzer();

    delay(5);
}

void onMqttConnect(esp_mqtt_client_handle_t client)
{
    Serial.println(F("✅ MQTT Conectado!"));
    mqtt_connected = true;
    if (mqttClient.isMyTurn(client)) // can be omitted if only one client
    {
        mqttClient.subscribe(mqtt_config.topic_comandos.c_str(), [](const std::string &payload) { 
          String msg = payload.c_str();
          Serial.printf("📩 Comando recebido: %s\n", msg.c_str());
          
          if (msg == "buzina") {
            ledcWrite(BUZZER_CHANNEL, HORN_DUTY_ON);
            delay(500);
            ledcWrite(BUZZER_CHANNEL, HORN_DUTY_OFF);
          }
          else if (msg == "alerta" || msg == "pisca") {
          Serial.println(F("  → PISCA ALERTA!"));
          hazardOn = !hazardOn;  // Alterna o estado do pisca alerta
          
          // Se desligou o alerta, garante que as setas individuais voltam ao normal
            if (!hazardOn) {
              leftBlinkOn = false;
              rightBlinkOn = false;
            }
          }
          else if (msg == "status") {
            enviarStatusMQTT();
          }
        });

        // Publica status inicial
        enviarStatusMQTT();
    }
}


#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
esp_err_t handleMQTT(esp_mqtt_event_handle_t event)
{
    mqttClient.onEventCallback(event);
    return ESP_OK;
}
#else  // IDF CHECK
void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
    mqttClient.onEventCallback(event);
}
#endif // // IDF CHECK
