#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================== Pines SPI para el ADS131M02 ==================
#define CS_PIN      5
#define SCLK_PIN    37
#define DIN_PIN     35    // MOSI
#define DOUT_PIN    36    // MISO
#define DRDY_PIN    4
#define RESET_PIN   2

// ================== Constantes del ADS131M02 ==================
const float VREF = 1.2;
const float FULL_SCALE = 8388607.0;

// Parámetros de diseño
const float AMPLIFIER_GAIN = 15.0;
const float SHUNT_RESISTANCE = 0.005;
const float VOLTAGE_DIVIDER_RATIO = 169.7;  // relación del divisor

// ================== Configuración WiFi ==================
const char* ssid = "Veronica";
const char* password = "vero160524";

// ================== Configuración EMQX Broker ==================
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;

// ================== Topics ==================
const char* mqtt_topic_request = "ESP32_mediciones/request";
const char* mqtt_topic_data    = "ESP32_mediciones/data";
const char* mqtt_topic_status  = "ESP32_mediciones/status";

// ================== Objetos WiFi y MQTT ==================
WiFiClient espClient;
PubSubClient client(espClient);
#define MQTT_MAX_PACKET_SIZE 4096

// ================== Buffer ==================
const int BUFFER_SIZE = 100;
float voltage_history[BUFFER_SIZE];
float current_history[BUFFER_SIZE];
int currentIndex = 0;
unsigned long lastMeasurementTime = 0;
const long MEASUREMENT_INTERVAL = 1000;

// ================== Control MQTT ==================
bool dataRequested = false;

// ================== Variables ADS ==================
volatile bool dataReady = false;
int32_t voltage_raw, current_raw;

// ================== Configuración SPI ==================
SPIClass hspi(HSPI);
SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE1);

// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== INICIANDO SISTEMA DE MEDICIÓN ===");

  setupWiFi();
  setupMQTT();
  setupADS131M02();

  Serial.println("✅ Sistema iniciado correctamente");
}

// ---------------------------------------------------
void setupWiFi() {
  Serial.print("📶 Conectando a WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi conectado!");
  Serial.print("📍 IP local: ");
  Serial.println(WiFi.localIP());
}

// ---------------------------------------------------
void setupMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
  reconnectMQTT();
}

// ---------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) message += (char)payload[i];
  Serial.println("\n📨 Mensaje MQTT recibido: " + message);

  if (String(topic) == mqtt_topic_request &&
      (message == "get_data" || message == "get_history")) {
    dataRequested = true;
    Serial.println("✅ Solicitud de datos detectada");
  }
}

// ---------------------------------------------------
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("🔌 Conectando a broker...");
    String clientId = "ESP32Medidor-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("✅ Conectado a broker EMQX!");
      client.subscribe(mqtt_topic_request);
      client.publish(mqtt_topic_status, "conectado_broker_emqx");
    } else {
      Serial.print("❌ Falló, rc="); Serial.println(client.state());
      delay(5000);
    }
  }
}

// ---------------------------------------------------
void setupADS131M02() {
  pinMode(CS_PIN, OUTPUT);
  pinMode(DRDY_PIN, INPUT);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  digitalWrite(RESET_PIN, LOW); delay(10);
  digitalWrite(RESET_PIN, HIGH); delay(100);

  hspi.begin(SCLK_PIN, DOUT_PIN, DIN_PIN, CS_PIN);
  attachInterrupt(digitalPinToInterrupt(DRDY_PIN), drdy_ISR, FALLING);
  init_ADS131M02();
  Serial.println("✅ ADS131M02 inicializado");
}

void drdy_ISR() { dataReady = true; }

void init_ADS131M02() {
  digitalWrite(CS_PIN, LOW);
  write_register(0x02, 0x8510);
  write_register(0x09, 0x0001);
  write_register(0x0A, 0x0001);
  digitalWrite(CS_PIN, HIGH);
  delay(10);
}

void write_register(uint8_t address, uint16_t data) {
  uint8_t cmd[3] = {0x06, address, 0x00};
  uint8_t resp[3];
  hspi.transferBytes(cmd, resp, 3);
  delayMicroseconds(10);
  cmd[0] = 0x08;
  cmd[1] = data >> 8;
  cmd[2] = data & 0xFF;
  hspi.transferBytes(cmd, resp, 3);
}

// ---------------------------------------------------
int32_t read_ADC_channel(uint8_t channel) {
  uint8_t cmd[3] = {0x12, 0x00, 0x00};
  uint8_t resp[9];
  hspi.transferBytes(cmd, resp, 9);

  int32_t data = ((int32_t)resp[3*channel + 3] << 16) |
                 ((int32_t)resp[3*channel + 4] << 8) |
                 resp[3*channel + 5];
  if (data & 0x800000) data |= 0xFF000000;
  return data;
}

// ---------------------------------------------------
float code_to_voltage(int32_t code) {
  return ((float)code / FULL_SCALE) * VREF;
}

// ---------------------------------------------------
void storeMeasurement(float voltage_rms, float current_rms) {
  voltage_history[currentIndex] = voltage_rms;
  current_history[currentIndex] = current_rms;
  currentIndex = (currentIndex + 1) % BUFFER_SIZE;
}

// ---------------------------------------------------
void publishHistory() {
  String data = "";
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int index = (currentIndex + i) % BUFFER_SIZE;
    data += String(i + 1) + ". V=" + String(voltage_history[index], 1)
          + "V C=" + String(current_history[index], 3) + "A\n";
  }
  if (client.publish(mqtt_topic_data, data.c_str())) {
    Serial.println("📤 Datos enviados a EMQX (" + String(BUFFER_SIZE) + " muestras)");
  } else Serial.println("❌ Error enviando datos MQTT");
}

// ---------------------------------------------------
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  if (dataRequested) { dataRequested = false; publishHistory(); }

  if (dataReady) {
    dataReady = false;
    digitalWrite(CS_PIN, LOW);
    current_raw = read_ADC_channel(0);  // AIN0 → Corriente
    voltage_raw = read_ADC_channel(1);  // AIN1 → Tensión
    digitalWrite(CS_PIN, HIGH);

    float current_adc = code_to_voltage(current_raw);
    float voltage_adc = code_to_voltage(voltage_raw);

    // === Corriente ===
    float shunt_voltage = current_adc / AMPLIFIER_GAIN;
    float current_peak = shunt_voltage / SHUNT_RESISTANCE;
    float current_rms = current_peak / 1.4142;

    // === Tensión ===
    float voltage_peak = voltage_adc * VOLTAGE_DIVIDER_RATIO;
    float voltage_rms = voltage_peak / 1.4142;

    if (millis() - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
      lastMeasurementTime = millis();
      storeMeasurement(voltage_rms, current_rms);
      Serial.print("V="); Serial.print(voltage_rms,1);
      Serial.print("V  I="); Serial.print(current_rms,3); Serial.println("A");
    }
  }
}
