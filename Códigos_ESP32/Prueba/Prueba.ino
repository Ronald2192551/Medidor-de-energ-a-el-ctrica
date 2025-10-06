#include <WiFi.h>
#include <PubSubClient.h>

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

// ================== Buffer MQTT ==================
#define MQTT_MAX_PACKET_SIZE 4096

// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("🚀 ESP32 Medidor - Iniciando...");

  setupWiFi();
  setupMQTT();

  Serial.println("✅ ESP32 lista para recibir solicitudes");
}

// =========================================================
// Conexión WiFi
// =========================================================
void setupWiFi() {
  Serial.print("📡 Conectando WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi conectado");
  Serial.print("📶 IP address: ");
  Serial.println(WiFi.localIP());
}

// =========================================================
// Configuración MQTT
// =========================================================
void setupMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);

  reconnectMQTT();
}

// =========================================================
// Callback MQTT
// =========================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("📨 Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == mqtt_topic_request) {
    if (message == "get_data" || message == "get_history") {
      Serial.println("✅ SOLICITUD DETECTADA - Generando y enviando datos...");
      client.publish(mqtt_topic_status, "iniciando_envio");
      sendDataPlainText();
    }
  }
}

// =========================================================
// Reconección MQTT
// =========================================================
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("🔌 Conectando MQTT...");

    String clientId = "ESP32-Medidor-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("✅ Conectado a broker.emqx.io!");
      client.subscribe(mqtt_topic_request);
      Serial.println("✅ Suscrito a: ESP32_mediciones/request");

      client.publish(mqtt_topic_status, "conectado_broker_emqx");
    } else {
      Serial.print("❌ Falló, rc=");
      Serial.print(client.state());
      Serial.println(" - Reintentando...");
      delay(5000);
    }
  }
}

// =========================================================
// Enviar los 100 datos como TEXTO con saltos de línea
// =========================================================
void sendDataPlainText() {
  Serial.println("📊 Generando y enviando 100 valores...");
  client.publish(mqtt_topic_status, "generando_datos");

  String data = "";

  for (int i = 0; i < 100; i++) {
    // Voltaje entre 112.0 y 118.0 V aprox
    float voltage = 115.0 + random(-30, 30) / 10.0;

    // Corriente entre 0.01 A (10 mA) y 10 A
    float current;

    if (i < 25) {
      current = random(10, 1000) / 1000.0;      // 0.01 – 1.0 A
    } else if (i < 75) {
      current = random(1000, 5000) / 1000.0;    // 1.0 – 5.0 A
    } else {
      current = random(5000, 10000) / 1000.0;   // 5.0 – 10.0 A
    }

    data += String(i + 1) + ". V=" + String(voltage, 1) + "V I=" + String(current, 3) + "A\n";
  }

  Serial.println("📤 Enviando datos en formato plano...");
  bool success = client.publish(mqtt_topic_data, data.c_str());

  if (success) {
    Serial.println("✅ Datos enviados exitosamente");
    client.publish(mqtt_topic_status, "envio_completado");
  } else {
    Serial.println("❌ Error enviando datos");
    client.publish(mqtt_topic_status, "error_envio");
  }
}

// =========================================================
// LOOP
// =========================================================
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Heartbeat cada 30 segundos
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 30000) {
    lastHeartbeat = millis();
    if (client.connected()) {
      client.publish(mqtt_topic_status, "esp32_activa");
      Serial.println("🟢 ESP32 activa - Esperando solicitudes...");
    }
  }

  delay(100);
}
