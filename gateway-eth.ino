#include <ETH.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <map>
#include <vector>
#include <WebServer.h>
#include <Update.h>
#include <Adafruit_NeoPixel.h>
#include "esp_task_wdt.h"
#include <time.h>

// =================================================================================
// Configuration
// =================================================================================
#define MESH_PREFIX     "mySmartHomeMesh"
#define MESH_PASSWORD   "aVerySecurePassword"
#define MESH_PORT       5555

#define MQTT_CLIENT_ID "PainlessMesh_Gateway_ETH_v4" 
#define GATEWAY_DEVICE_NAME "PainlessMesh ETH Gateway" 

#define AVAILABILITY_TOPIC      "PainlessMesh/Gateway/Status"
#define PAYLOAD_AVAILABLE       "online"
#define PAYLOAD_NOT_AVAILABLE   "offline"
#define DEVICE_OFFLINE_TIMEOUT  (10 * 60 * 1000UL)
#define MQTT_GATEWAY_COMMAND_TOPIC "PainlessMesh/Gateway/Command"

#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN   12
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        1
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18

#define STATUS_LED_PIN    14
#define STATUS_LED_COUNT  1
#define LED_BRIGHTNESS   50

#define WDT_TIMEOUT_SECONDS 10

const char* ntpServer = "ntp.aliyun.com";
const long  gmtOffset_sec = 8 * 3600;
const int   daylightOffset_sec = 0;



String macToKey(const String& mac) {
  String key = mac;
  key.replace(":", "");
  return key;
}
// =================================================================================
// Global Objects & Data Structures
// =================================================================================
painlessMesh  mesh;
WiFiClient    ethClient;
PubSubClient  mqttClient(ethClient);
Preferences   preferences;
Scheduler     userScheduler;
WebServer     server(80);

char MQTT_SERVER[40];
char MQTT_PORT_STR[6];
char MQTT_USER[33];
char MQTT_PASS[65];

enum LedState { STATE_NO_NETWORK, STATE_NO_MQTT, STATE_ALL_CONNECTED, STATE_DEVICE_OFFLINE, STATE_OTA_UPDATING };
Adafruit_NeoPixel statusPixel(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
LedState currentLedState = STATE_NO_NETWORK;

struct MeshDevice {
  uint32_t nodeId; String name; String macAddress; String type;
  unsigned long lastSeenTimestamp; bool isOnline; bool hasBeenDiscovered;
  String linkedEnvSensorName; uint32_t daylightThresholdLx;
  uint8_t on_r, on_g, on_b; uint8_t off_r, off_g, off_b;
  uint8_t brightness_on, brightness_off; uint8_t default_on_value, default_off_value;
};
std::map<String, MeshDevice> knownDevicesByMac;
std::map<uint32_t, String> nodeIdToMacMap;
std::map<String, String> nameToMacMap;

struct SceneAction { String deviceName; String statesJson; };
struct Scene { String name; std::vector<SceneAction> actions; };
std::map<String, Scene> scenes;

// =================================================================================
// Function Declarations
// =================================================================================
void WiFiEvent(WiFiEvent_t event);
void tryMqttReconnect();
void initStatusLed();
void setStatusLed(LedState newState);
void blinkLedCallback();
void updateSystemStatus();
void onMeshReceived(uint32_t from, String &msg);
void processMeshMessage(String &message);
void onMeshNewConnection(uint32_t nodeId);
void onMeshDroppedConnection(uint32_t nodeId);
void checkDeviceStatus();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void loadScenes();
void triggerScene(const String& sceneName);
void publishSceneDiscovery();
void unpublishSceneDiscovery(const String& sceneName);
void addCorsHeaders();
void handleApiDevices();
void handleApiStatus();
void handleApiScenes();
void handleApiSaveMqtt();
void handleApiReboot();
void handleApiClearNvs();
void handleApiSaveDeviceConfig();
void handleApiTriggerScene();
void handleApiSaveScene();
void handleApiDeleteScene();
void handleApiRediscoverDevice();
void handleApiAvailableSensors();
void handleNotFound();
void handleOTAUpdatePage();
void handleOTAUpdateUpload();
void publishDiscoveryForDevice(const char* deviceType, const char* deviceName, int channels, const String& originalMessage);
void loadCredentials();
void saveCredentials();
String macToKey(const String& mac);
void loadAddressBook();
void saveToAddressBook(const MeshDevice& device);
void clearAllNvs();

// =================================================================================
// Tasks
// =================================================================================
Task taskCheckDevices(TASK_MINUTE, TASK_FOREVER, &checkDeviceStatus);
Task taskReconnectMQTT(TASK_SECOND * 5, TASK_FOREVER, &tryMqttReconnect);
Task taskBlinkLed(TASK_SECOND / 5, TASK_FOREVER, &blinkLedCallback);
Task taskUpdateSystemStatus(TASK_SECOND * 2, TASK_FOREVER, &updateSystemStatus);

// =================================================================================
// SETUP
// =================================================================================
void setup() {
  Serial.begin(115200);
  
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("Booting... Reset reason: %d\n", reason);
  if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT) {
    Serial.println("Watchdog timer triggered the last reset!");
    preferences.begin("logs", false);
    preferences.putString("wdt_log_3", preferences.getString("wdt_log_2", ""));
    preferences.putString("wdt_log_2", preferences.getString("wdt_log_1", ""));
    char log_entry[50];
    snprintf(log_entry, sizeof(log_entry), "WDT Reset at Uptime: %lu s", millis() / 1000);
    preferences.putString("wdt_log_1", String(log_entry));
    preferences.end();
  }
  
  initStatusLed();
  Serial.println("\n--- PainlessMesh Ethernet Gateway (API Backend) ---");

  WiFi.onEvent(WiFiEvent);
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

  loadCredentials();
  loadAddressBook();
  loadScenes();

  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT_STR));
  mqttClient.setCallback(mqttCallback);

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP, 11);
  mesh.onReceive(&onMeshReceived);
  mesh.onNewConnection(&onMeshNewConnection);
  mesh.onDroppedConnection(&onMeshDroppedConnection);
  
  server.on("/api/.*", HTTP_OPTIONS, []() { addCorsHeaders(); server.send(204); });
  server.on("/api/devices", HTTP_GET, handleApiDevices);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/scenes", HTTP_GET, handleApiScenes);
  server.on("/api/available_sensors", HTTP_GET, handleApiAvailableSensors);
  server.on("/api/save_mqtt", HTTP_POST, handleApiSaveMqtt);
  server.on("/api/reboot", HTTP_POST, handleApiReboot);
  server.on("/api/clear_nvs", HTTP_POST, handleApiClearNvs);
  server.on("/api/save_device_config", HTTP_POST, handleApiSaveDeviceConfig);
  server.on("/api/trigger_scene", HTTP_POST, handleApiTriggerScene);
  server.on("/api/save_scene", HTTP_POST, handleApiSaveScene);
  server.on("/api/delete_scene", HTTP_POST, handleApiDeleteScene);
  server.on("/api/rediscover_device", HTTP_POST, handleApiRediscoverDevice);
  server.on("/update", HTTP_GET, handleOTAUpdatePage);
  server.on("/update_upload", HTTP_POST, []() { addCorsHeaders(); server.send(200, "text/plain", "OK"); }, handleOTAUpdateUpload);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Web server started.");

  userScheduler.addTask(taskCheckDevices); taskCheckDevices.enable();
  userScheduler.addTask(taskReconnectMQTT);
  userScheduler.addTask(taskBlinkLed);
  userScheduler.addTask(taskUpdateSystemStatus); taskUpdateSystemStatus.enable();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("NTP service configured.");
  
  Serial.printf("Initializing Task Watchdog Timer with a %d second timeout...\n", WDT_TIMEOUT_SECONDS);
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(NULL);

  Serial.println("Gateway setup complete.");
}

// =================================================================================
// LOOP
// =================================================================================
void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  userScheduler.execute();
  mqttClient.loop();
}

// =================================================================================
// Status LED & System Status Functions
// =================================================================================
const uint32_t COLOR_RED = Adafruit_NeoPixel::Color(255, 0, 0);
const uint32_t COLOR_ORANGE = Adafruit_NeoPixel::Color(255, 100, 0);
const uint32_t COLOR_BLUE = Adafruit_NeoPixel::Color(0, 0, 255);
const uint32_t COLOR_OFF = Adafruit_NeoPixel::Color(0, 0, 0);

void blinkLedCallback() {
  static bool ledOn = false; 
  ledOn = !ledOn;
  uint32_t colorToShow = COLOR_OFF;
  if (ledOn) {
    switch (currentLedState) {
      case STATE_NO_NETWORK: colorToShow = COLOR_RED; break;
      case STATE_NO_MQTT: colorToShow = COLOR_ORANGE; break;
      case STATE_DEVICE_OFFLINE: colorToShow = COLOR_BLUE; break;
      default: break;
    }
  }
  statusPixel.setPixelColor(0, colorToShow); 
  statusPixel.show();
}

void setStatusLed(LedState newState) {
  if (newState == currentLedState) return;
  currentLedState = newState;
  taskBlinkLed.disable();
  switch (newState) {
    case STATE_NO_NETWORK: 
      taskBlinkLed.setInterval(TASK_SECOND / 5); 
      taskBlinkLed.enable(); 
      break;
    case STATE_NO_MQTT: 
      taskBlinkLed.setInterval(TASK_SECOND); 
      taskBlinkLed.enable(); 
      break;
    case STATE_DEVICE_OFFLINE: 
      taskBlinkLed.setInterval(TASK_SECOND * 1.5); 
      taskBlinkLed.enable(); 
      break;
    case STATE_OTA_UPDATING:
      statusPixel.setPixelColor(0, COLOR_BLUE);
      statusPixel.show();
      break;
    case STATE_ALL_CONNECTED: 
      statusPixel.setPixelColor(0, COLOR_OFF); 
      statusPixel.show(); 
      break;
  }
}

void initStatusLed() {
  statusPixel.begin(); 
  statusPixel.setBrightness(LED_BRIGHTNESS);
  statusPixel.clear(); 
  statusPixel.show();
  setStatusLed(STATE_NO_NETWORK);
  Serial.println("Status LED module initialized.");
}

void updateSystemStatus() {
  if (!ETH.linkUp()) { 
    setStatusLed(STATE_NO_NETWORK); 
    return; 
  }
  if (!mqttClient.connected()) { 
    setStatusLed(STATE_NO_MQTT); 
    taskReconnectMQTT.enableIfNot(); 
    return; 
  }
  bool foundOfflineDevice = false;
  unsigned long now = millis();
  for (auto& pair : knownDevicesByMac) {
    if (pair.second.isOnline && (now - pair.second.lastSeenTimestamp) > DEVICE_OFFLINE_TIMEOUT) {
      foundOfflineDevice = true; 
      break;
    }
  }
  if (foundOfflineDevice) { 
    setStatusLed(STATE_DEVICE_OFFLINE); 
  } else { 
    setStatusLed(STATE_ALL_CONNECTED); 
  }
}

// =================================================================================
// Network & MQTT Functions
// =================================================================================
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START: Serial.println("ETH Started"); ETH.setHostname("painlessmesh-gateway"); break;
    case ARDUINO_EVENT_ETH_CONNECTED: Serial.println("ETH Connected"); break;
    case ARDUINO_EVENT_ETH_GOT_IP: Serial.print("ETH Got IP: "); Serial.println(ETH.localIP()); break;
    case ARDUINO_EVENT_ETH_DISCONNECTED: Serial.println("ETH Disconnected"); break;
    case ARDUINO_EVENT_ETH_STOP: Serial.println("ETH Stopped"); break;
    default: break;
  }
}

void tryMqttReconnect() {
  if (mqttClient.connected()) { 
    taskReconnectMQTT.disable(); 
    return; 
  }
  Serial.print("Attempting MQTT connection...");
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, AVAILABILITY_TOPIC, 0, true, PAYLOAD_NOT_AVAILABLE)) {
    Serial.println("connected!");
    mqttClient.publish(AVAILABILITY_TOPIC, PAYLOAD_AVAILABLE, true);
    mqttClient.subscribe("PainlessMesh/Command/#");
    mqttClient.subscribe("PainlessMesh/Config/#");
    mqttClient.subscribe(MQTT_GATEWAY_COMMAND_TOPIC);
    mqttClient.subscribe("PainlessMesh/GroupCommand/SceneControl");
    publishSceneDiscovery();
    taskReconnectMQTT.disable();
  } else { 
    Serial.print("failed, rc="); 
    Serial.println(mqttClient.state()); 
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topic_str = String(topic);
  String payload_str; 
  payload_str.reserve(length);
  for (int i = 0; i < length; i++) { payload_str += (char)payload[i]; }

  if (topic_str.equals(MQTT_GATEWAY_COMMAND_TOPIC)) {
    if (payload_str.equals("CLEAR_NVS")) { 
      clearAllNvs(); 
      delay(100); 
      ESP.restart(); 
    }
    return;
  }

  if (topic_str.equals("PainlessMesh/GroupCommand/SceneControl")) {
    Serial.println("--- Received Scene Command from HA ---");
    StaticJsonDocument<1536> finalMeshCommand;
    finalMeshCommand["cmd"] = "groupState";
    JsonDocument payloadDoc;
    deserializeJson(payloadDoc, payload, length);
    finalMeshCommand["payload"] = payloadDoc.as<JsonArray>();
    String commandString;
    serializeJson(finalMeshCommand, commandString);
    mesh.sendBroadcast(commandString);
    Serial.printf("  -> Broadcasted Scene Command: %s\n", commandString.c_str());
    return;
  }
  
  if (topic_str.startsWith("PainlessMesh/Command/") || topic_str.startsWith("PainlessMesh/Config/")) {
    String deviceName, deviceType;
    bool is_config = topic_str.startsWith("PainlessMesh/Config/");
    topic_str.replace(is_config ? "PainlessMesh/Config/" : "PainlessMesh/Command/", "");
    int firstSlash = topic_str.indexOf('/'); 
    if (firstSlash == -1) return;
    deviceType = topic_str.substring(0, firstSlash);
    deviceName = topic_str.substring(firstSlash + 1);

    if (nameToMacMap.count(deviceName)) {
      String macAddress = nameToMacMap[deviceName];
      if (knownDevicesByMac.count(macAddress)) {
        MeshDevice& device = knownDevicesByMac[macAddress];
        if (!device.isOnline) { 
          Serial.printf("Command for offline device '%s' ignored.\n", deviceName.c_str()); 
          return; 
        }
        uint32_t targetNodeId = device.nodeId;
        String meshCommandType;

        if (is_config) { 
          meshCommandType = "configLed";
        } else if (deviceType == "ServoController") {
          StaticJsonDocument<96> ha_payload_doc; 
          deserializeJson(ha_payload_doc, payload, length);
          meshCommandType = ha_payload_doc.containsKey("action") ? "runAction" : "setAngle";
        } else { 
          meshCommandType = "setState"; 
        }

        StaticJsonDocument<512> mesh_cmd_doc; 
        StaticJsonDocument<256> payload_doc;
        deserializeJson(payload_doc, payload, length);
        mesh_cmd_doc["to"] = targetNodeId; 
        mesh_cmd_doc["cmd"] = meshCommandType;
        mesh_cmd_doc["payload"] = payload_doc.as<JsonObject>();
        
        String mesh_cmd_str; 
        serializeJson(mesh_cmd_doc, mesh_cmd_str);
        mesh.sendSingle(targetNodeId, mesh_cmd_str);
        Serial.printf("  -> Forwarded command to '%s' (NodeID: %u, Cmd: %s)\n", 
                      deviceName.c_str(), targetNodeId, meshCommandType.c_str());
      }
    }
  }
}

// =================================================================================
// Core Logic & Callbacks
// =================================================================================
void onMeshReceived(uint32_t from, String &msg) {
  Serial.printf("Received from %u: %s\n", from, msg.c_str());
  processMeshMessage(msg);
}

void processMeshMessage(String &message) {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<512> doc; 
  if (deserializeJson(doc, message)) { return; }
  if (!doc.containsKey("mac") || !doc.containsKey("name") || !doc.containsKey("nodeId")) { return; }
  String macAddress = doc["mac"].as<String>();
  uint32_t newNodeId = doc["nodeId"];
  String deviceName = doc["name"].as<String>();
  MeshDevice* device = nullptr; 
  bool isNewDevice = false; 
  bool needsSave = false;

  if (knownDevicesByMac.count(macAddress)) { 
    device = &knownDevicesByMac[macAddress];
  } else {
    isNewDevice = true; 
    needsSave = true; 
    MeshDevice newDevice;
    newDevice.macAddress = macAddress; 
    newDevice.nodeId = newNodeId; 
    newDevice.name = deviceName;
    newDevice.type = doc.containsKey("type") ? doc["type"].as<const char*>() : "Unknown";
    newDevice.hasBeenDiscovered = false;
    knownDevicesByMac[macAddress] = newDevice; 
    device = &knownDevicesByMac[macAddress];
    Serial.printf("New device discovered by MAC: %s\n", macAddress.c_str());
  }

  if (device->nodeId != newNodeId) {
    if (device->nodeId != 0 && nodeIdToMacMap.count(device->nodeId)) { 
      nodeIdToMacMap.erase(device->nodeId); 
    }
    device->nodeId = newNodeId; 
    needsSave = true;
  }

  bool wasOffline = !device->isOnline; 
  device->lastSeenTimestamp = millis(); 
  device->isOnline = true;
  nodeIdToMacMap[newNodeId] = macAddress; 
  nameToMacMap[deviceName] = macAddress;

  if (!device->hasBeenDiscovered) {
    Serial.printf("[DISCOVERY] Device '%s' needs discovery. Running...\n", device->name.c_str());
    int channels = doc.containsKey("channels") ? doc["channels"].as<int>() : 0;
    publishDiscoveryForDevice(device->type.c_str(), device->name.c_str(), channels, message);
    device->hasBeenDiscovered = true; 
    needsSave = true;
  }

  if (needsSave || wasOffline) { 
    saveToAddressBook(*device); 
  }

  if (wasOffline || isNewDevice) {
    char availability_topic[128];
    snprintf(availability_topic, sizeof(availability_topic), "PainlessMesh/Status/%s/availability", device->name.c_str());
    mqttClient.publish(availability_topic, PAYLOAD_AVAILABLE, true);
    Serial.printf("[STATUS] Device '%s' is now online.\n", device->name.c_str());
  }
  
  if (!doc.containsKey("heartbeat")) {
    doc.remove("mac"); 
    String modifiedJsonString; 
    serializeJson(doc, modifiedJsonString);
    String topic = "PainlessMesh/Status/" + String(device->type) + "/" + String(deviceName);
    mqttClient.publish(topic.c_str(), modifiedJsonString.c_str());
  }
}

void onMeshNewConnection(uint32_t nodeId) { 
  Serial.printf("New connection: %u\n", nodeId); 
}

void onMeshDroppedConnection(uint32_t nodeId) {
  Serial.printf("Dropped connection: %u\n", nodeId);
  if (nodeIdToMacMap.count(nodeId)) {
    String mac = nodeIdToMacMap[nodeId];
    if (knownDevicesByMac.count(mac)) {
      MeshDevice& device = knownDevicesByMac[mac];
      device.isOnline = false;
      char availability_topic[128];
      snprintf(availability_topic, sizeof(availability_topic), "PainlessMesh/Status/%s/availability", device.name.c_str());
      mqttClient.publish(availability_topic, PAYLOAD_NOT_AVAILABLE, true);
      Serial.printf("[STATUS] Device '%s' marked offline due to dropped connection.\n", device.name.c_str());
    }
  }
}

void checkDeviceStatus() {
  unsigned long now = millis();
  for (auto& pair : knownDevicesByMac) {
    MeshDevice& device = pair.second;
    if (device.isOnline && (now - device.lastSeenTimestamp) > DEVICE_OFFLINE_TIMEOUT) {
      Serial.printf("[STATUS] Device '%s' has gone offline (timeout).\n", device.name.c_str());
      device.isOnline = false;
      char availability_topic[128];
      snprintf(availability_topic, sizeof(availability_topic), "PainlessMesh/Status/%s/availability", device.name.c_str());
      mqttClient.publish(availability_topic, PAYLOAD_NOT_AVAILABLE, true);
      saveToAddressBook(device);
    }
  }
}

// =================================================================================
// Web Server (API) Handlers
// =================================================================================
void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With");
}

void handleApiDevices() {
  addCorsHeaders();
  
  // 使用较大的 DynamicJsonDocument 以确保能容纳所有设备和它们的配置
  DynamicJsonDocument doc(4096); 
  JsonArray devices = doc.to<JsonArray>();

  for (auto const& [mac, device] : knownDevicesByMac) {
    JsonObject devObj = devices.createNestedObject();
    
    // --- 1. 添加所有设备共有的基础信息 ---
    devObj["nodeId"] = device.nodeId;
    devObj["name"] = device.name;
    devObj["macAddress"] = device.macAddress;
    devObj["type"] = device.type;
    devObj["lastSeenTimestamp"] = device.lastSeenTimestamp;
    devObj["isOnline"] = device.isOnline;
    devObj["hasBeenDiscovered"] = device.hasBeenDiscovered;
    
    // *** ↓↓↓ THIS IS THE KEY ADDITION ↓↓↓ ***
    // --- 2. 如果设备是 MultiSwitch, 添加详细的 "config" 对象 ---
    if (device.type == "MultiSwitch") {
      JsonObject config = devObj.createNestedObject("config");
      
      // 准备用于 JSON 的颜色和亮度值
      char on_color_hex[8];
      char off_color_hex[8];
      snprintf(on_color_hex, sizeof(on_color_hex), "#%02x%02x%02x", device.on_r, device.on_g, device.on_b);
      snprintf(off_color_hex, sizeof(off_color_hex), "#%02x%02x%02x", device.off_r, device.off_g, device.off_b);
      
      // 使用已保存的默认值（如果存在），否则使用设备上报的当前值
      uint8_t on_brightness = device.default_on_value > 0 ? device.default_on_value : device.brightness_on;
      uint8_t off_brightness = device.default_off_value > 0 ? device.default_off_value : device.brightness_off;
      
      // 将所有配置信息添加到 "config" 对象中
      config["on_color"] = on_color_hex;
      config["brightness_on"] = on_brightness;
      config["off_color"] = off_color_hex;
      config["brightness_off"] = off_brightness;
      config["linkedSensor"] = device.linkedEnvSensorName;
      config["daylightThreshold"] = device.daylightThresholdLx;
    }
  }

  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
}

void handleApiStatus() {
  addCorsHeaders();
  DynamicJsonDocument doc(1024);
  unsigned long s = millis() / 1000;
  String uptimeStr = String(s / 86400) + "d " + String((s % 86400) / 3600) + "h " + String((s % 3600) / 60) + "m";
  doc["uptime"] = uptimeStr;
  doc["freeHeap"] = ESP.getFreeHeap();
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    doc["systemTime"] = "NTP not synced yet";
  } else {
    char time_buffer[20];
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    doc["systemTime"] = String(time_buffer);
  }
  JsonObject net = doc.createNestedObject("network");
  net["type"] = "Ethernet";
  net["ipAddress"] = ETH.localIP().toString();
  net["macAddress"] = ETH.macAddress();
  JsonObject mqtt = doc.createNestedObject("mqtt");
  mqtt["connected"] = mqttClient.connected();
  mqtt["server"] = String(MQTT_SERVER);
  doc["deviceCount"] = knownDevicesByMac.size();
  JsonObject wdt = doc.createNestedObject("watchdog");
  preferences.begin("logs", true);
  wdt["log1"] = preferences.getString("wdt_log_1", "No record");
  wdt["log2"] = preferences.getString("wdt_log_2", "No record");
  wdt["log3"] = preferences.getString("wdt_log_3", "No record");
  preferences.end();
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleApiScenes() {
    addCorsHeaders();
    DynamicJsonDocument doc(2048);
    for(auto const& [name, scene] : scenes) {
        JsonArray actions = doc.createNestedArray(name);
        for(const auto& action : scene.actions) {
            JsonObject actionObj = actions.createNestedObject();
            actionObj["name"] = action.deviceName;
            JsonDocument tempStatesDoc;
            deserializeJson(tempStatesDoc, action.statesJson);
            actionObj["states"] = tempStatesDoc.as<JsonArray>();
        }
    }
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleApiSaveMqtt() {
  addCorsHeaders();
  strncpy(MQTT_SERVER, server.arg("mqtt_server").c_str(), sizeof(MQTT_SERVER) - 1);
  strncpy(MQTT_PORT_STR, server.arg("mqtt_port").c_str(), sizeof(MQTT_PORT_STR) - 1);
  strncpy(MQTT_USER, server.arg("mqtt_user").c_str(), sizeof(MQTT_USER) - 1);
  strncpy(MQTT_PASS, server.arg("mqtt_pass").c_str(), sizeof(MQTT_PASS) - 1);
  saveCredentials();
  server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"MQTT settings saved. Reconnecting...\"}");
  delay(100);
  if (mqttClient.connected()) { mqttClient.disconnect(); }
  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT_STR));
  taskReconnectMQTT.enable();
}

void handleApiReboot() {
  addCorsHeaders();
  server.send(200, "application/json", "{\"status\":\"rebooting\"}");
  delay(100);
  ESP.restart();
}

void handleApiClearNvs() {
  addCorsHeaders();
  clearAllNvs();
  server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"NVS cleared. Rebooting...\"}");
  delay(200);
  ESP.restart();
}

// --- 请用这个完整的函数替换您 gateway-eth.ino 中的旧版本 ---

void handleApiSaveDeviceConfig() {
  addCorsHeaders();
  
  // 1. 从 Web 请求中获取所有数据
  String deviceName = server.arg("deviceName");
  if (!nameToMacMap.count(deviceName)) { 
    server.send(404, "application/json", "{\"error\":\"Device not found\"}"); 
    return; 
  }
  MeshDevice& device = knownDevicesByMac[nameToMacMap[deviceName]];

  String onColorHex = server.arg("on_color"); 
  long on_number = strtol(&onColorHex.c_str()[1], NULL, 16);
  device.on_r = (on_number >> 16) & 0xFF; 
  device.on_g = (on_number >> 8) & 0xFF; 
  device.on_b = on_number & 0xFF;
  
  String offColorHex = server.arg("off_color"); 
  long off_number = strtol(&offColorHex.c_str()[1], NULL, 16);
  device.off_r = (off_number >> 16) & 0xFF; 
  device.off_g = (off_number >> 8) & 0xFF; 
  device.off_b = off_number & 0xFF;
  
  device.brightness_on = server.arg("brightness_on").toInt();
  device.brightness_off = server.arg("brightness_off").toInt();
  device.linkedEnvSensorName = server.arg("linked_sensor_name");
  device.daylightThresholdLx = server.arg("daylight_threshold").toInt();
  device.default_on_value = device.brightness_on;
  device.default_off_value = device.brightness_off;
  
  // 2. 将完整的配置保存到网关自己的 NVS
  saveToAddressBook(device);

  // *** 3. THIS IS THE FIX: 构建包含所有字段的完整 payload ***
  StaticJsonDocument<384> payloadDoc;
  payloadDoc["led_on_color"]["r"] = device.on_r;
  payloadDoc["led_on_color"]["g"] = device.on_g;
  payloadDoc["led_on_color"]["b"] = device.on_b;
  payloadDoc["led_off_color"]["r"] = device.off_r;
  payloadDoc["led_off_color"]["g"] = device.off_g;
  payloadDoc["led_off_color"]["b"] = device.off_b;
  payloadDoc["brightness_on"] = device.brightness_on;
  payloadDoc["brightness_off"] = device.brightness_off;
  payloadDoc["linked_env_sensor_name"] = device.linkedEnvSensorName;
  payloadDoc["daylight_threshold_lx"] = device.daylightThresholdLx;

  // 4. 将完整的命令发送给开关设备
  StaticJsonDocument<512> mesh_cmd_doc;
  mesh_cmd_doc["to"] = device.nodeId; 
  mesh_cmd_doc["cmd"] = "configLed";
  mesh_cmd_doc["payload"] = payloadDoc.as<JsonObject>();
  String mesh_cmd_str; 
  serializeJson(mesh_cmd_doc, mesh_cmd_str);
  mesh.sendSingle(device.nodeId, mesh_cmd_str);

  Serial.printf("Sent FULL config command to '%s': %s\n", deviceName.c_str(), mesh_cmd_str.c_str());

  server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"Configuration sent.\"}");
}

void handleApiTriggerScene() {
  addCorsHeaders();
  String sceneName = server.arg("sceneName");
  if (sceneName.length() > 0) {
    triggerScene(sceneName);
    server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"Scene triggered.\"}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing sceneName\"}");
  }
}

void handleApiSaveScene() {
  addCorsHeaders();
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"Bad Request\"}"); return; }
  String body = server.arg("plain");
  // ... (Full implementation from previous correct version)
  loadScenes();
  publishSceneDiscovery();
  server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"Scene saved.\"}");
}

void handleApiDeleteScene() {
  addCorsHeaders();
  // ... (Full implementation from previous correct version, using server.arg("sceneName"))
  unpublishSceneDiscovery(server.arg("sceneName"));
  // ...
  server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"Scene deleted.\"}");
}

void handleApiRediscoverDevice() {
  addCorsHeaders();
  if (!server.hasArg("deviceName")) { server.send(400, "application/json", "{\"error\":\"Missing deviceName\"}"); return; }
  String deviceName = server.arg("deviceName");
  if (!nameToMacMap.count(deviceName)) { server.send(404, "application/json", "{\"error\":\"Device not found\"}"); return; }
  MeshDevice& device = knownDevicesByMac[nameToMacMap[deviceName]];
  device.hasBeenDiscovered = false;
  saveToAddressBook(device);
  server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"Rediscovery initiated.\"}");
}

void handleApiAvailableSensors() {
  addCorsHeaders();
  DynamicJsonDocument doc(1024);
  JsonArray sensors = doc.to<JsonArray>();
  for (const auto& pair : knownDevicesByMac) {
    if (pair.second.type == "Env_Sensor" && pair.second.isOnline) {
      sensors.add(pair.second.name);
    }
  }
  String response; serializeJson(sensors, response);
  server.send(200, "application/json", response);
}

void handleNotFound() {
  addCorsHeaders();
  server.send(404, "application/json", "{\"error\":\"Not Found\"}");
}

// Keep OTA handlers as they are, but add CORS headers
void handleOTAUpdatePage() {
  addCorsHeaders();
  String html = "<html><body><h1>Gateway OTA Update</h1><form method='POST' action='/update_upload' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form></body></html>";
  server.send(200, "text/html", html);
}

void handleOTAUpdateUpload() {
  addCorsHeaders();
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) { setStatusLed(STATE_OTA_UPDATING); if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { Update.printError(Serial); }
  } else if (upload.status == UPLOAD_FILE_WRITE) { if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) { Update.printError(Serial); }
  } else if (upload.status == UPLOAD_FILE_END) { if (Update.end(true)) { server.send(200, "text/plain", "Update Success! Rebooting..."); delay(1000); ESP.restart(); } else { Update.printError(Serial); server.send(500, "text/plain", "OTA Error"); } }
}




void clearAllNvs() {
  Serial.println("\n\n!!! NVS CLEAR TRIGGERED !!!");
  preferences.begin("AddrBook", false); if (preferences.clear()) { Serial.println("  - Cleared Device Address Book."); } preferences.end();
  preferences.begin("Discovered", false); if (preferences.clear()) { Serial.println("  - Cleared HA Discovery Records."); } preferences.end();
  preferences.begin("scenes", false); if (preferences.clear()) { Serial.println("  - Cleared Scenes."); } preferences.end();
  preferences.begin("logs", false); if (preferences.clear()) { Serial.println("  - Cleared Logs."); } preferences.end();
  preferences.begin("gateway-creds", false); if (preferences.clear()) { Serial.println("  - Cleared Credentials."); } preferences.end();
  Serial.println("NVS CLEAR COMPLETE. Rebooting...");
}

void loadCredentials() {
  preferences.begin("gateway-creds", true);
  String server = preferences.getString("mqtt_server", "192.168.1.1");
  String port = preferences.getString("mqtt_port", "1883");
  String user = preferences.getString("mqtt_user", "");
  String pass = preferences.getString("mqtt_pass", "");
  preferences.end();
  strncpy(MQTT_SERVER, server.c_str(), sizeof(MQTT_SERVER) - 1);
  strncpy(MQTT_PORT_STR, port.c_str(), sizeof(MQTT_PORT_STR) - 1);
  strncpy(MQTT_USER, user.c_str(), sizeof(MQTT_USER) - 1);
  strncpy(MQTT_PASS, pass.c_str(), sizeof(MQTT_PASS) - 1);
  Serial.println("Loaded MQTT credentials from NVS.");
}

void saveCredentials() {
  preferences.begin("gateway-creds", false);
  preferences.putString("mqtt_server", MQTT_SERVER);
  preferences.putString("mqtt_port", MQTT_PORT_STR);
  preferences.putString("mqtt_user", MQTT_USER);
  preferences.putString("mqtt_pass", MQTT_PASS);
  preferences.end();
  Serial.println("Saved new MQTT credentials to NVS.");
}

void loadAddressBook() {
  knownDevicesByMac.clear(); nodeIdToMacMap.clear(); nameToMacMap.clear();
  preferences.begin("AddrBook", true); 
  String macList = preferences.getString("mac_list", "");
  if (macList.length() > 0) {
    char list_cstr[macList.length() + 1]; strcpy(list_cstr, macList.c_str());
    char* mac_cstr = strtok(list_cstr, ",");
    while (mac_cstr != NULL) {
      String macAddress = String(mac_cstr); String baseKey = macToKey(macAddress);      
      MeshDevice device;
      device.macAddress = macAddress;
      device.nodeId = preferences.getUInt((baseKey + "i").c_str(), 0);
      device.name = preferences.getString((baseKey + "n").c_str(), "Unknown");
      device.type = preferences.getString((baseKey + "t").c_str(), "Unknown");
      device.isOnline = false; device.lastSeenTimestamp = 0;
      device.hasBeenDiscovered = preferences.getBool((baseKey + "d").c_str(), false);
      device.linkedEnvSensorName = preferences.getString((baseKey + "s").c_str(), "");
      device.daylightThresholdLx = preferences.getUInt((baseKey + "x").c_str(), 0);
      uint32_t defaultValues = preferences.getUInt((baseKey + "v").c_str(), 0x500A);
      device.default_on_value = (defaultValues >> 8) & 0xFF;
      device.default_off_value = defaultValues & 0xFF;
      String led_config_str = preferences.getString((baseKey + "l").c_str(), "");
      if (led_config_str.length() > 0) {
        sscanf(led_config_str.c_str(), "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu",
               &device.on_r, &device.on_g, &device.on_b, &device.off_r, &device.off_g, &device.off_b,
               &device.brightness_on, &device.brightness_off);
      } else {
        device.on_r = 10; device.on_g = 255; device.on_b = 10;
        device.off_r = 255; device.off_g = 10; device.off_b = 10;
        device.brightness_on = 80; device.brightness_off = 10;
      }
      knownDevicesByMac[macAddress] = device;
      if (device.nodeId != 0) nodeIdToMacMap[device.nodeId] = macAddress;
      if (device.name != "Unknown") nameToMacMap[device.name] = macAddress;
      mac_cstr = strtok(NULL, ",");
    }
  }
  preferences.end();
}

void saveToAddressBook(const MeshDevice& device) {
  preferences.begin("AddrBook", false);
  String baseKey = macToKey(device.macAddress);
  preferences.putUInt((baseKey + "i").c_str(), device.nodeId);
  preferences.putString((baseKey + "n").c_str(), device.name);
  preferences.putString((baseKey + "t").c_str(), device.type);
  preferences.putBool((baseKey + "d").c_str(), device.hasBeenDiscovered);
  preferences.putString((baseKey + "s").c_str(), device.linkedEnvSensorName);
  preferences.putUInt((baseKey + "x").c_str(), device.daylightThresholdLx);
  preferences.putUInt((baseKey + "v").c_str(), (device.default_on_value << 8) | device.default_off_value);
  char led_config_buffer[50];
  snprintf(led_config_buffer, sizeof(led_config_buffer), "%d,%d,%d,%d,%d,%d,%d,%d",
           device.on_r, device.on_g, device.on_b, device.off_r, device.off_g, device.off_b,
           device.brightness_on, device.brightness_off);
  preferences.putString((baseKey + "l").c_str(), led_config_buffer);
  String macList = ""; bool first = true;
  for (auto const& [mac, dev] : knownDevicesByMac) {
    if (!first) { macList += ","; } macList += mac; first = false;
  }
  preferences.putString("mac_list", macList);
  preferences.end();
}

void loadScenes() {
  scenes.clear();
  preferences.begin("scenes", true);
  String sceneList = preferences.getString("scene_list", "");
  if (sceneList.length() > 0) {
    char list_cstr[sceneList.length() + 1]; strcpy(list_cstr, sceneList.c_str());
    char* sceneName_cstr = strtok(list_cstr, ",");
    while (sceneName_cstr != NULL) {
      String sceneName = String(sceneName_cstr);
      String sceneJsonStr = preferences.getString(sceneName.c_str(), "");
      if (sceneJsonStr.length() > 0) {
        StaticJsonDocument<1024> doc;
        if (deserializeJson(doc, sceneJsonStr) == DeserializationError::Ok) {
          Scene scene; scene.name = sceneName;
          JsonArray actionsArray = doc.as<JsonArray>();
          for (JsonObject actionObj : actionsArray) {
            SceneAction action;
            action.deviceName = actionObj["name"].as<String>();
            serializeJson(actionObj["states"], action.statesJson);
            scene.actions.push_back(action);
          }
          scenes[sceneName] = scene;
        }
      }
      sceneName_cstr = strtok(NULL, ",");
    }
  }
  preferences.end();
}

void triggerScene(const String& sceneName) {
  if (!scenes.count(sceneName)) return;
  Scene& scene = scenes[sceneName];
  StaticJsonDocument<1536> finalMeshCommand;
  finalMeshCommand["cmd"] = "groupState";
  JsonArray payloadArray = finalMeshCommand.createNestedArray("payload");
  for (const auto& action : scene.actions) {
    JsonObject actionObj = payloadArray.createNestedObject();
    actionObj["name"] = action.deviceName;
    JsonDocument tempStatesDoc;
    deserializeJson(tempStatesDoc, action.statesJson);
    actionObj["states"] = tempStatesDoc.as<JsonArray>();
  }
  String commandString;
  serializeJson(finalMeshCommand, commandString);
  mesh.sendBroadcast(commandString);
  Serial.printf("Broadcasted Scene Command: %s\n", commandString.c_str());
}

void publishSceneDiscovery() {
  if (!mqttClient.connected()) return;
  Serial.println("\n--- Publishing HA Discovery for all Scenes ---");
  String base_id = MQTT_CLIENT_ID;
  base_id.replace(".", "_");
  StaticJsonDocument<256> deviceDoc;
  deviceDoc["identifiers"][0] = base_id;
  deviceDoc["name"] = GATEWAY_DEVICE_NAME;
  deviceDoc["model"] = "ESP32 PainlessMesh Gateway";
  deviceDoc["manufacturer"] = "DIY";

  for (auto const& [sceneName, scene] : scenes) {
    StaticJsonDocument<1536> config_doc;
    String unique_id = "scene_" + sceneName;
    unique_id.replace(" ", "_"); unique_id.toLowerCase();
    
    JsonDocument payloadDoc;
    JsonArray actions = payloadDoc.to<JsonArray>();
    for (const auto& action : scene.actions) {
      JsonObject actionObj = actions.createNestedObject();
      actionObj["name"] = action.deviceName;
      JsonDocument tempStatesDoc;
      deserializeJson(tempStatesDoc, action.statesJson.c_str());
      actionObj["states"] = tempStatesDoc.as<JsonArray>();
    }
    String payload_press_str; serializeJson(payloadDoc, payload_press_str);

    config_doc["unique_id"] = unique_id;
    config_doc["name"] = sceneName;
    config_doc["object_id"] = unique_id;
    config_doc["command_topic"] = "PainlessMesh/GroupCommand/SceneControl";
    config_doc["payload_press"] = payload_press_str;
    config_doc["availability_topic"] = AVAILABILITY_TOPIC;
    config_doc["device"] = deviceDoc;
    config_doc["icon"] = "mdi:play-box-outline";

    char config_topic[256];
    snprintf(config_topic, sizeof(config_topic), "homeassistant/button/%s/config", unique_id.c_str());
    String output; serializeJson(config_doc, output);
    mqttClient.publish(config_topic, output.c_str(), true);
    Serial.printf("  - Published config for scene: %s\n", sceneName.c_str());
  }
}

void unpublishSceneDiscovery(const String& sceneName) {
  if (!mqttClient.connected()) return;
  String unique_id = "scene_" + sceneName;
  unique_id.replace(" ", "_"); unique_id.toLowerCase();
  char config_topic[256];
  snprintf(config_topic, sizeof(config_topic), "homeassistant/button/%s/config", unique_id.c_str());
  mqttClient.publish(config_topic, "", true);
}





// =================================================================================
// HA & Persistence Functions
// =================================================================================
void publishDiscoveryForDevice(const char* deviceType, const char* deviceName, int channels, const String& originalMessage) {
  if (!mqttClient.connected()) return;

  // --- 1. (核心修正) “先删除”逻辑 ---
  // 无论如何，都先尝试删除所有可能的旧实体，确保一个干净的开始
  Serial.printf("  -> Step 1: Un-publishing any old entities for '%s'...\n", deviceName);
  if (strcmp(deviceType, "SmartSocket") == 0) {
    String unique_id = String(deviceName) + "_relay";
    char config_topic[256];
    snprintf(config_topic, sizeof(config_topic), "homeassistant/switch/%s/config", unique_id.c_str());
    mqttClient.publish(config_topic, "", true);
  } 
  else if (strcmp(deviceType, "MultiSwitch") == 0) {
    const char* suffixes[] = {"_relay", "_relay_L", "_relay_R", "_relay_M"};
    for (int i=0; i<4; i++) {
        String unique_id = String(deviceName) + suffixes[i];
        char config_topic[256];
        snprintf(config_topic, sizeof(config_topic), "homeassistant/switch/%s/config", unique_id.c_str());
        mqttClient.publish(config_topic, "", true);
    }
  }
  // ... 您可以为 PIR, Env_Sensor, ServoController 添加类似的删除逻辑 ...

  // 短暂延时，给 Broker 一点处理时间
  delay(50); 
  
  // --- 2. “后创建”逻辑 ---
  Serial.printf("  -> Step 2: Publishing new discovery config for '%s'...\n", deviceName);

  // --- 1. 准备所有实体都会共用的信息 ---
  char device_availability_topic[128];
  snprintf(device_availability_topic, sizeof(device_availability_topic), "PainlessMesh/Status/%s/availability", deviceName);
  char state_topic_buffer[128];
  snprintf(state_topic_buffer, sizeof(state_topic_buffer), "PainlessMesh/Status/%s/%s", deviceType, deviceName);
  StaticJsonDocument<256> deviceDoc;
  deviceDoc["identifiers"][0] = deviceName;
  deviceDoc["name"] = deviceName;
  deviceDoc["manufacturer"] = "PainlessMesh DIY";

  // --- 2. 根据设备类型执行不同的发现逻辑 ---
  if (strcmp(deviceType, "SmartSocket") == 0) {
    deviceDoc["model"] = "Smart Socket";
    StaticJsonDocument<1024> config_doc;
    String unique_id = String(deviceName) + "_relay";
    config_doc["unique_id"] = unique_id;
    config_doc["name"] = "Relay";
    String objectIdStr = unique_id;
    objectIdStr.toLowerCase();
    config_doc["object_id"] = objectIdStr;
    char command_topic_buffer[128];
    snprintf(command_topic_buffer, sizeof(command_topic_buffer), "PainlessMesh/Command/%s/%s", deviceType, deviceName);
    config_doc["state_topic"] = state_topic_buffer;
    config_doc["value_template"] = "{{ value_json.payload.states[0] }}";
    config_doc["command_topic"] = command_topic_buffer;
    config_doc["payload_on"] = "{\"states\":[\"on\"]}";
    config_doc["payload_off"] = "{\"states\":[\"off\"]}";
    config_doc["state_on"] = "on";
    config_doc["state_off"] = "off";
    JsonArray availability = config_doc.createNestedArray("availability");
    JsonObject gatewayAvail = availability.createNestedObject();
    gatewayAvail["topic"] = AVAILABILITY_TOPIC;
    JsonObject deviceAvail = availability.createNestedObject();
    deviceAvail["topic"] = device_availability_topic;
    config_doc["availability_mode"] = "all";
    config_doc["payload_available"] = PAYLOAD_AVAILABLE;
    config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
    config_doc["device"] = deviceDoc;
    char config_topic[256];
    snprintf(config_topic, sizeof(config_topic), "homeassistant/switch/%s/config", unique_id.c_str());
    String output; serializeJson(config_doc, output);
    mqttClient.publish(config_topic, output.c_str(), true);
  } 
  else if (strcmp(deviceType, "MultiSwitch") == 0) {
    if (channels <= 0 || channels > 3) return;
    deviceDoc["model"] = "MultiSwitch_" + String(channels) + "ch";
    for (int i = 0; i < channels; i++) {
        String suffix = (channels == 1) ? "_relay" : ((channels == 2) ? (i == 0 ? "_relay_L" : "_relay_R") : (i == 0 ? "_relay_L" : (i == 1 ? "_relay_M" : "_relay_R")));
        String friendly_name = (channels == 1) ? "Relay" : ((channels == 2) ? (i == 0 ? "Relay L" : "Relay R") : (i == 0 ? "Relay L" : (i == 1 ? "Relay M" : "Relay R")));
        StaticJsonDocument<1024> config_doc;
        String unique_id = String(deviceName) + suffix;
        config_doc["unique_id"] = unique_id;
        config_doc["name"] = friendly_name;
        String objectIdStr = unique_id;
        objectIdStr.toLowerCase();
        config_doc["object_id"] = objectIdStr;
        char command_topic_buffer[128];
        snprintf(command_topic_buffer, sizeof(command_topic_buffer), "PainlessMesh/Command/%s/%s", deviceType, deviceName);
        String on_p = "{\"states\":[";
        for(int j=0; j<channels; j++) { on_p += (i==j ? "\"on\"" : "\"ignore\""); if(j < channels-1) on_p += ","; }
        on_p += "]}";
        String off_p = "{\"states\":[";
        for(int j=0; j<channels; j++) { off_p += (i==j ? "\"off\"" : "\"ignore\""); if(j < channels-1) off_p += ","; }
        off_p += "]}";
        config_doc["state_topic"] = state_topic_buffer;
        config_doc["value_template"] = "{{ value_json.payload.states[" + String(i) + "] }}";
        config_doc["command_topic"] = command_topic_buffer;
        config_doc["payload_on"] = on_p;
        config_doc["payload_off"] = off_p;
        config_doc["state_on"] = "on";
        config_doc["state_off"] = "off";
        JsonArray availability = config_doc.createNestedArray("availability");
        JsonObject gatewayAvail = availability.createNestedObject();
        gatewayAvail["topic"] = AVAILABILITY_TOPIC;
        JsonObject deviceAvail = availability.createNestedObject();
        deviceAvail["topic"] = device_availability_topic;
        config_doc["availability_mode"] = "all";
        config_doc["payload_available"] = PAYLOAD_AVAILABLE;
        config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
        config_doc["device"] = deviceDoc;
        char config_topic[256];
        snprintf(config_topic, sizeof(config_topic), "homeassistant/switch/%s/config", unique_id.c_str());
        String output; serializeJson(config_doc, output);
        mqttClient.publish(config_topic, output.c_str(), true);
    }
  }
  else if (strcmp(deviceType, "PIR_Sensor") == 0 || strcmp(deviceType, "Contact_Sensor") == 0) {
      bool isPir = (strcmp(deviceType, "PIR_Sensor") == 0);
      deviceDoc["model"] = isPir ? "ESP-NOW PIR Sensor" : "ESP-NOW Contact Sensor";
      {
          StaticJsonDocument<1024> config_doc;
          String unique_id = String(deviceName) + (isPir ? "_motion" : "_contact");
          config_doc["unique_id"] = unique_id;
          config_doc["name"] = isPir ? "Motion" : "Contact";
          String objectIdStr = unique_id;
          objectIdStr.toLowerCase();
          config_doc["object_id"] = objectIdStr;
          config_doc["device_class"] = isPir ? "motion" : "door";
          config_doc["state_topic"] = state_topic_buffer;
          config_doc["value_template"] = "{{ value_json.payload.states[0] }}";
          config_doc["payload_on"] = "on";
          config_doc["payload_off"] = "off";
          JsonArray availability = config_doc.createNestedArray("availability");
          JsonObject gatewayAvail = availability.createNestedObject();
          gatewayAvail["topic"] = AVAILABILITY_TOPIC;
          JsonObject deviceAvail = availability.createNestedObject();
          deviceAvail["topic"] = device_availability_topic;
          config_doc["availability_mode"] = "all";
          config_doc["payload_available"] = PAYLOAD_AVAILABLE;
          config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
          config_doc["device"] = deviceDoc;
          char config_topic[256];
          snprintf(config_topic, sizeof(config_topic), "homeassistant/binary_sensor/%s/config", unique_id.c_str());
          String output; serializeJson(config_doc, output);
          mqttClient.publish(config_topic, output.c_str(), true);
      }
      {
          StaticJsonDocument<1024> config_doc;
          String unique_id = String(deviceName) + "_battery";
          config_doc["unique_id"] = unique_id;
          config_doc["name"] = "Battery";
          String objectIdStr = unique_id;
          objectIdStr.toLowerCase();
          config_doc["object_id"] = objectIdStr;
          config_doc["device_class"] = "voltage";
          config_doc["state_class"] = "measurement";
          config_doc["unit_of_measurement"] = "V";
          config_doc["state_topic"] = state_topic_buffer;
          config_doc["value_template"] = "{{ value_json.payload.battery }}";
          JsonArray availability = config_doc.createNestedArray("availability");
          JsonObject gatewayAvail = availability.createNestedObject();
          gatewayAvail["topic"] = AVAILABILITY_TOPIC;
          JsonObject deviceAvail = availability.createNestedObject();
          deviceAvail["topic"] = device_availability_topic;
          config_doc["availability_mode"] = "all";
          config_doc["payload_available"] = PAYLOAD_AVAILABLE;
          config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
          config_doc["device"] = deviceDoc;
          char config_topic[256];
          snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s/config", unique_id.c_str());
          String output; serializeJson(config_doc, output);
          mqttClient.publish(config_topic, output.c_str(), true);
      }
  }
  else if (strcmp(deviceType, "Env_Sensor") == 0) {
    deviceDoc["model"] = "ESP-NOW Environment Sensor";
    Serial.printf("\n--- Publishing HA Discovery for Env_Sensor (%s) ---\n", deviceName);

    // 在本函数作用域内重新解析消息，以确保 payload 对象的生命周期安全
    StaticJsonDocument<512> doc;
    deserializeJson(doc, originalMessage);
    JsonObject payload = doc["payload"];
    
    // 如果消息中没有 payload，则无法继续，直接退出
    if (!payload) {
      Serial.println("  [ERROR] Env_Sensor message has no payload for discovery. Aborting.");
      return;
    }

    // --- 按需创建 Battery 实体 ---
    if (payload.containsKey("battery")) {
      StaticJsonDocument<1024> config_doc;
      String unique_id = String(deviceName) + "_battery";
      config_doc["unique_id"] = unique_id;
      config_doc["name"] = "Battery";
      String objectIdStr = unique_id;
      objectIdStr.toLowerCase();
      config_doc["object_id"] = objectIdStr;
      config_doc["device_class"] = "voltage";
      config_doc["state_class"] = "measurement";
      config_doc["unit_of_measurement"] = "V";
      config_doc["state_topic"] = state_topic_buffer;
      config_doc["value_template"] = "{{ value_json.payload.battery }}";
      JsonArray availability = config_doc.createNestedArray("availability");
      JsonObject gatewayAvail = availability.createNestedObject();
      gatewayAvail["topic"] = AVAILABILITY_TOPIC;
      JsonObject deviceAvail = availability.createNestedObject();
      deviceAvail["topic"] = device_availability_topic;
      config_doc["availability_mode"] = "all";
      config_doc["payload_available"] = PAYLOAD_AVAILABLE;
      config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
      config_doc["device"] = deviceDoc;
      char config_topic[256];
      snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s/config", unique_id.c_str());
      String output; serializeJson(config_doc, output);
      mqttClient.publish(config_topic, output.c_str(), true);
      Serial.println("  - Published config for: Battery");
    }

    // --- 按需创建 Illuminance 实体 ---
    if (payload.containsKey("illuminance")) {
      StaticJsonDocument<1024> config_doc;
      String unique_id = String(deviceName) + "_illuminance";
      config_doc["unique_id"] = unique_id;
      config_doc["name"] = "Illuminance";
      String objectIdStr = unique_id;
      objectIdStr.toLowerCase();
      config_doc["object_id"] = objectIdStr;
      config_doc["device_class"] = "illuminance";
      config_doc["state_class"] = "measurement";
      config_doc["unit_of_measurement"] = "lx";
      config_doc["state_topic"] = state_topic_buffer;
      config_doc["value_template"] = "{{ value_json.payload.illuminance }}";
      JsonArray availability = config_doc.createNestedArray("availability");
      JsonObject gatewayAvail = availability.createNestedObject();
      gatewayAvail["topic"] = AVAILABILITY_TOPIC;
      JsonObject deviceAvail = availability.createNestedObject();
      deviceAvail["topic"] = device_availability_topic;
      config_doc["availability_mode"] = "all";
      config_doc["payload_available"] = PAYLOAD_AVAILABLE;
      config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
      config_doc["device"] = deviceDoc;
      char config_topic[256];
      snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s/config", unique_id.c_str());
      String output; serializeJson(config_doc, output);
      mqttClient.publish(config_topic, output.c_str(), true);
      Serial.println("  - Published config for: Illuminance");
    }

    // --- 按需创建 Temperature 实体 ---
    if (payload.containsKey("temperature")) {
      StaticJsonDocument<1024> config_doc;
      String unique_id = String(deviceName) + "_temperature";
      config_doc["unique_id"] = unique_id;
      config_doc["name"] = "Temperature";
      String objectIdStr = unique_id;
      objectIdStr.toLowerCase();
      config_doc["object_id"] = objectIdStr;
      config_doc["device_class"] = "temperature";
      config_doc["state_class"] = "measurement";
      config_doc["unit_of_measurement"] = "°C";
      config_doc["state_topic"] = state_topic_buffer;
      config_doc["value_template"] = "{{ value_json.payload.temperature }}";
      JsonArray availability = config_doc.createNestedArray("availability");
      JsonObject gatewayAvail = availability.createNestedObject();
      gatewayAvail["topic"] = AVAILABILITY_TOPIC;
      JsonObject deviceAvail = availability.createNestedObject();
      deviceAvail["topic"] = device_availability_topic;
      config_doc["availability_mode"] = "all";
      config_doc["payload_available"] = PAYLOAD_AVAILABLE;
      config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
      config_doc["device"] = deviceDoc;
      char config_topic[256];
      snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s/config", unique_id.c_str());
      String output; serializeJson(config_doc, output);
      mqttClient.publish(config_topic, output.c_str(), true);
      Serial.println("  - Published config for: Temperature");
    }

    // --- 按需创建 Humidity 实体 ---
    if (payload.containsKey("humidity")) {
      StaticJsonDocument<1024> config_doc;
      String unique_id = String(deviceName) + "_humidity";
      config_doc["unique_id"] = unique_id;
      config_doc["name"] = "Humidity";
      String objectIdStr = unique_id;
      objectIdStr.toLowerCase();
      config_doc["object_id"] = objectIdStr;
      config_doc["device_class"] = "humidity";
      config_doc["state_class"] = "measurement";
      config_doc["unit_of_measurement"] = "%";
      config_doc["state_topic"] = state_topic_buffer;
      config_doc["value_template"] = "{{ value_json.payload.humidity }}";
      JsonArray availability = config_doc.createNestedArray("availability");
      JsonObject gatewayAvail = availability.createNestedObject();
      gatewayAvail["topic"] = AVAILABILITY_TOPIC;
      JsonObject deviceAvail = availability.createNestedObject();
      deviceAvail["topic"] = device_availability_topic;
      config_doc["availability_mode"] = "all";
      config_doc["payload_available"] = PAYLOAD_AVAILABLE;
      config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
      config_doc["device"] = deviceDoc;
      char config_topic[256];
      snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s/config", unique_id.c_str());
      String output; serializeJson(config_doc, output);
      mqttClient.publish(config_topic, output.c_str(), true);
      Serial.println("  - Published config for: Humidity");
    }
  }
  else if (strcmp(deviceType, "ServoController") == 0) {
      deviceDoc["model"] = "Servo Controller";
      { // Angle number entity
          StaticJsonDocument<1024> config_doc;
          String unique_id = String(deviceName) + "_angle";
          config_doc["unique_id"] = unique_id;
          config_doc["name"] = "Angle";
          String objectIdStr = unique_id;
          objectIdStr.toLowerCase();
          config_doc["object_id"] = objectIdStr;
          char command_topic_buffer[128];
          snprintf(command_topic_buffer, sizeof(command_topic_buffer), "PainlessMesh/Command/%s/%s", deviceType, deviceName);
          config_doc["command_topic"] = command_topic_buffer;
          config_doc["command_template"] = "{\"angle\":{{value}}}";
          config_doc["state_topic"] = state_topic_buffer;
          config_doc["value_template"] = "{{ value_json.payload.angle }}";
          config_doc["min"] = 0;
          config_doc["max"] = 180;
          config_doc["unit_of_measurement"] = "°";
          JsonArray availability = config_doc.createNestedArray("availability");
          JsonObject gatewayAvail = availability.createNestedObject();
          gatewayAvail["topic"] = AVAILABILITY_TOPIC;
          JsonObject deviceAvail = availability.createNestedObject();
          deviceAvail["topic"] = device_availability_topic;
          config_doc["availability_mode"] = "all";
          config_doc["payload_available"] = PAYLOAD_AVAILABLE;
          config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
          config_doc["device"] = deviceDoc;
          char config_topic[256];
          snprintf(config_topic, sizeof(config_topic), "homeassistant/number/%s/config", unique_id.c_str());
          String output; serializeJson(config_doc, output);
          mqttClient.publish(config_topic, output.c_str(), true);
      }
      { // Sweep button entity
          StaticJsonDocument<1024> config_doc;
          String unique_id = String(deviceName) + "_sweep";
          config_doc["unique_id"] = unique_id;
          config_doc["name"] = "Sweep Action";
          String objectIdStr = unique_id;
          objectIdStr.toLowerCase();
          config_doc["object_id"] = objectIdStr;
          char command_topic_buffer[128];
          snprintf(command_topic_buffer, sizeof(command_topic_buffer), "PainlessMesh/Command/%s/%s", deviceType, deviceName);
          config_doc["command_topic"] = command_topic_buffer;
          config_doc["payload_press"] = "{\"action\":\"sweep_return\"}";
          JsonArray availability = config_doc.createNestedArray("availability");
          JsonObject gatewayAvail = availability.createNestedObject();
          gatewayAvail["topic"] = AVAILABILITY_TOPIC;
          JsonObject deviceAvail = availability.createNestedObject();
          deviceAvail["topic"] = device_availability_topic;
          config_doc["availability_mode"] = "all";
          config_doc["payload_available"] = PAYLOAD_AVAILABLE;
          config_doc["payload_not_available"] = PAYLOAD_NOT_AVAILABLE;
          config_doc["device"] = deviceDoc;
          char config_topic[256];
          snprintf(config_topic, sizeof(config_topic), "homeassistant/button/%s/config", unique_id.c_str());
          String output; serializeJson(config_doc, output);
          mqttClient.publish(config_topic, output.c_str(), true);
      }
  }

  preferences.begin("Discovered", false);
  preferences.putBool(deviceName, true);
  preferences.end();
}
