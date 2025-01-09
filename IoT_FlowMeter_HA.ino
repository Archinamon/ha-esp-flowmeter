#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <Simpletimer.h>
#include <ACS712.h>

#define DEFAULT_BAUD_RATE 9600

#define SHUNT_PIN      A0 // Shunt meter for water pump
#define BUZZER_PIN     D8 // Buzzer KY-012 pin
#define RELAY_PIN1     D7 // Relay pin for water pump
#define FLOWSENSOR_PIN D5 // Sensor Input

#define EEPROM_SIZE       64 // Memory EEPROM size
#define TOTAL_VOLUME_ADDR  0 // Address of total current volume
#define HIST_VOLUME_ADDR  10 // Address of historycal volume

// Emergency reboot timeout
#define RESTART_LIMIT 60000

// Open/close the relay on NC-channel
#define RELAY_OPEN   1
#define RELAY_CLOSED 0

// Wi-Fi settings
const char* ssid     = "Your_SSID";
const char* password = "Your_passwrd";

// MQTT settings
const int   mqtt_json_buffer_size = 1024;                   // Extended buffer for mostly all of json's
const char* mqtt_server           = "192.168.1.108";        // IP-address of your MQTT-broker (e.g., Home Assistant)
const int   mqtt_port             = 1883;                   // Default port of MQTT-broker
const char* mqtt_user             = "esp_flowmeter_main";   // Username of MQTT-client (if you need it)
const char* mqtt_password         = "8622esphomeflowmeter"; // Pass of MQTT-client (if you need it)

const char* mqtt_topic_flowmeter_op    = "home/flowmeter/main_tank/op";    // FlowMeter operations topic
const char* mqtt_topic_flowmeter_flows = "home/flowmeter/main_tank/flows"; // Flow running topic
const char* mqtt_topic_flowmeter_state = "home/flowmeter/main_tank/state"; // Sync statuses topic
const char* mqtt_topic_flowmeter_pump  = "home/flowmeter/main_tank/pump";  // Water pump switcher topic
const char* mqtt_topic_flowmeter_alarm = "home/flowmeter/main_tank/alarm"; // Emergency topic to publish pump states
const char* mqtt_topic_flowmeter_logs  = "home/flowmeter/main_tank/logs";  // Debug logs topic (be careful to use only in trusted network)

const char* mqtt_topic_broker_status         = "homeassistant/status";                                   // The broker main status
const char* mqtt_topic_flowmeter_discovery   = "homeassistant/sensor/flowmeter/main_tank/config";        // The device discovery topic
const char* mqtt_topic_btn_reboot_discovery  = "homeassistant/button/flowmeter/main_tank_reboot/config"; // Reboot button discovery topic
const char* mqtt_topic_btn_refill_discovery  = "homeassistant/button/flowmeter/main_tank_refill/config"; // Reset on refill button discovery topic
const char* mqtt_topic_switch_pump_discovery = "homeassistant/switch/flowmeter/main_tank_pump/config";   // Pump control switcher discovery topic

// Wi-Fi params & mode
bool wifiConnected = false;
enum OperatingMode {
  NORMAL,
  CONNECTING,
  WAITING_FOR_SETUP,
  PUMP_STUCK
};

// Current status in HomeAssistant
enum FlowMeterStatus {
  OPERATING,
  OFFLINE
};
const char* FlowMeterStatusStrings[] = {
  "operating",
  "offline"
};

enum PumpStatus {
  PUMPING,
  BURNING,
  IDLE
};
const char* PumpStatusStrings[] = {
  "pump_operate",
  "pump_disable",
  "pump_operate"
};

// Home Assistant
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// OTA updates http-server
ESP8266WebServer httpServer(1488);
ESP8266HTTPUpdateServer httpUpdater;

// Current flowmeter state
OperatingMode currentMode = NORMAL;
PumpStatus pumpMode = IDLE;

volatile int currentFlowImpulse;

float historycalVolume = 0.0;
float totalVolume = 0.0, litresPerMinute;
unsigned long currentTime = millis();
unsigned long cloopTime = currentTime;

Simpletimer flowMeterTimer{};
Simpletimer currentMeterTimer{};

ACS712 shuntSensor(SHUNT_PIN, 3.3, 4095, 185);

void setOperationMode(OperatingMode mode) {
  currentMode = mode;
}

bool isCurrentlyOperating(OperatingMode mode) {
  return currentMode == mode;
}

void setPumpMode(PumpStatus mode) {
  pumpMode = mode;
}

bool isCurrentPumpMode(PumpStatus mode) {
  return pumpMode == mode;
}

void resetOperatingMode() {
  setOperationMode(NORMAL);
  setPumpMode(IDLE);

  publishMqttStatus(OPERATING);
  publishMqttSensorValue(totalVolume);
  publishMqttPumpState(IDLE);
}

void resetTotalVolume() {
  totalVolume = 0.0;
  saveVolumeToEEPROM(TOTAL_VOLUME_ADDR, totalVolume);
  publishDebugLogs("Current totalVolume has been set to 0.0");

  delay(1000);
  resetOperatingMode();
}

void IRAM_ATTR flowImpulseInterrupt() {
  currentFlowImpulse++;
}

int counBurningCycle = 0;
void calculateFlowRate() {
  currentTime = millis();

  if (currentTime >= (cloopTime + 1000)) {
    cloopTime = currentTime; // Updates cloopTime
    if (currentFlowImpulse != 0) {
      counBurningCycle = 0; // reset the counter

      // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
      litresPerMinute = (currentFlowImpulse / 7.5); // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour

      Serial.printf("Rate: %.1f L/M", litresPerMinute);
      
      litresPerMinute = litresPerMinute/60;
      totalVolume = totalVolume + litresPerMinute;
      historycalVolume = historycalVolume + litresPerMinute;

      publishMqttSensorValue(totalVolume);
      Serial.printf(" Vol: %.1f L\n", totalVolume);

      saveVolumeToEEPROM(TOTAL_VOLUME_ADDR, totalVolume);
      saveVolumeToEEPROM(HIST_VOLUME_ADDR, historycalVolume);
      currentFlowImpulse = 0; // reset impulse counter
    } else {
      if (isCurrentPumpMode(PUMPING)) {
        // avoid first trigger due to post calculating noises
        if (counBurningCycle++ > 1) {
          publishDebugLogs("Pump is burning! Set the status for the relay operation...");
          setPumpMode(BURNING);
        }
      }
    }
  }
}

void meterCurrentDC() {
  if (isCurrentlyOperating(PUMP_STUCK)) {
    publishMqttPumpAlarm();
    return; // pump should be disabled before...
  }

  int mA = shuntSensor.mA_DC();
  if (mA < 100) {
    // so it's just a noise...

    if (isCurrentPumpMode(PUMPING) || isCurrentPumpMode(BURNING)) {
      resetOperatingMode();
    } // reset all states if pump is running no more

    return;
  }

  publishDebugLogs(
    (String("Current on pump: ") + String(mA)).c_str()
  );

  // this parakeets is regular load of the pump (about 754 parakeets)
  if (mA > 700) {
    if (isCurrentPumpMode(PUMPING)) {
      return; // means we already in 'pumping' state, do not reenter it
    }

    setPumpMode(PUMPING);
    publishMqttPumpState(PUMPING);
    return;
  }

  if (isCurrentPumpMode(BURNING)) {
    setOperationMode(PUMP_STUCK);
    publishMqttStatus(OFFLINE);
    publishMqttPumpState(BURNING);
  }
}

/*
  Buzzer methods
*/

void singleShortBeep() {
  tone(BUZZER_PIN, 2000);
  delay(200);
  noTone(BUZZER_PIN);
  delay(50);
}

void singleLongBeep() {
  tone(BUZZER_PIN, 2000);
  delay(1000);
  noTone(BUZZER_PIN);
}

/*
  Memory EEPROM methods
*/

void saveVolumeToEEPROM(uint addr, float volume) {
  // Converting float to bytearray
  uint8_t* p = (uint8_t*) &volume;
  for (int i = 0; i < sizeof(volume); i++) {
    EEPROM.write(addr + i, p[i]);
  }
  EEPROM.commit();
}

float readVolumeFromEEPROM(uint addr) {
  float volume = 0.0;
  uint8_t* p = (uint8_t*) &volume;
  for (int i = 0; i < sizeof(volume); i++) {
    p[i] = EEPROM.read(addr + i);
  }

  if (isnan(volume) || volume < 0.0 || volume > 1000000.0) {
    Serial.println("Invalid volume value detected in EEPROM. Resetting to 0.0.");
    volume = 0.0;
  }

  return volume;
}

/*
  Setup methods — OTA, memory, watchdog, HA, WiFi and mqtt broker
*/

void setupOTAUpdate() {
  httpUpdater.setup(&httpServer, "/firmware", "arkhiotika", password);
  httpServer.onNotFound(handleNotFound);
  httpServer.on("/restart", []() {
    httpServer.send(200, "text/plain", "controller was restarted");
    delay(300);
    reboot();
  });
  httpServer.begin();
  publishDebugLogs("HTTP Update Server ready");
}

void handleNotFound() {
  httpServer.send(404, "text/plain", "GO FUCK YOURSELF MOTHERFUCKER!");
}

void initializeMemModule() {
  EEPROM.begin(EEPROM_SIZE);

  // Reading last totalVolume from EEPROM
  historycalVolume = readVolumeFromEEPROM(HIST_VOLUME_ADDR);
  totalVolume = readVolumeFromEEPROM(TOTAL_VOLUME_ADDR);
  Serial.printf("Restored totalVolume: %.1f\n", totalVolume);
  Serial.printf("Restored historycalVolume: %.1f\n", historycalVolume);
}

void setupWatchdog() {
  ESP.wdtEnable(RESTART_LIMIT);
  publishDebugLogs("Emergency restart watchdog has been attached");
}

void setupHomeAssistant() {
  if (isCurrentlyOperating(WAITING_FOR_SETUP)) {
    setOperationMode(CONNECTING);
  }

  if (!isCurrentlyOperating(CONNECTING)) {
    publishDebugLogs("ERROR! This method should be called only in Connecting mode!");
    return;
  }

  WiFi.mode(WIFI_STA);

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(mqtt_json_buffer_size);
  mqttClient.setCallback(mqttCallback);

  taskEstablishWiFiConnection();
  taskSubscribeToMQTTBroker();

  startMqttDiscovery();
  delay(3000);
  resetOperatingMode();
}

void taskEstablishWiFiConnection() {
  if (!isCurrentlyOperating(CONNECTING)) {
    publishDebugLogs("Wi-Fi state: connected!");
    return;
  }

  Serial.println();
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected!");
    Serial.print("Current IP: ");
    Serial.println(WiFi.localIP());

    wifiConnected = true;

    setupOTAUpdate();
  } else {
    publishDebugLogs("\nCoudn't connect to Wi-Fi. Trying again.");
    delay(2000);
  }
}

void taskSubscribeToMQTTBroker() {
  while (!wifiConnected) {
    publishDebugLogs("Awaiting Wi-FI...");
    delay(1000);
    return;
  }

  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP8266FlowMeterClient", mqtt_user, mqtt_password)) {
      publishMqttStatus(OFFLINE);
      publishDebugLogs("Connection established!");

      mqttClient.subscribe(mqtt_topic_flowmeter_op);
      mqttClient.subscribe(mqtt_topic_broker_status);
      mqttClient.subscribe(mqtt_topic_flowmeter_pump);
    } else {
      Serial.print("Error, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Trying again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char) payload[i];
  }

  String tpc = String(topic);

  // Operating the FlowMeter
  if (tpc == mqtt_topic_flowmeter_op ||
    tpc == mqtt_topic_flowmeter_pump
  ) {
    mqttProceedOnTopicMessageReceived(message);
  }
  
  if (tpc == mqtt_topic_broker_status) {
    mqttProceedOnDeviceStatusMessageReceived(message);
  }
}

void mqttMainLooper() {
  if (wifiConnected && mqttClient.connected()) {
    mqttClient.loop();
    return;
  }
}

void mqttProceedOnTopicMessageReceived(String msg) {
  publishDebugLogs(String("Got message: " + msg).c_str());

  if (msg == "restart") {
    reboot();
  }

  if (msg == "refill") {
    resetTotalVolume();
  }

  if (msg == "pump_operate") {
    if (digitalRead(RELAY_PIN1) != RELAY_OPEN) {
      digitalWrite(RELAY_PIN1, RELAY_OPEN);
      singleLongBeep();
    }
  }

  if (msg == "pump_disable") {
    if (digitalRead(RELAY_PIN1) != RELAY_CLOSED) {
      digitalWrite(RELAY_PIN1, RELAY_CLOSED);
      singleLongBeep();
    }
  }
}

void mqttProceedOnDeviceStatusMessageReceived(String msg) {
  publishDebugLogs(String("Got message: " + msg).c_str());

  if (msg == "online") {
    startMqttDiscovery();
    delay(3000); // operation timeout for proper registering the device on HA broker server
    resetOperatingMode();
  }
}

/*
  Discovery data
*/

JsonObject createDeviceInfo(JsonDocument& doc) {
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "esp8266mod_arch_sketch002";
  device["manufacturer"] = "Arduino";
  device["model"] = "Node MCU";
  device["name"] = "Custom FlowMeter sensor";
  return device;
}

JsonObject createAvailabilityInfo(JsonDocument& doc) {
  JsonObject availability = doc.createNestedObject("availability");
  availability["topic"] = mqtt_topic_flowmeter_state;
  availability["payload_available"] = FlowMeterStatusStrings[OPERATING];
  availability["payload_not_available"] = FlowMeterStatusStrings[OFFLINE];
  return availability;
}

void startMqttDiscovery() {
  StaticJsonDocument<512> flowmeterDoc;
  flowmeterDoc["unique_id"] = "flowmeter_main";
  flowmeterDoc["object_id"] = "flowmeter_main";
  flowmeterDoc["name"] = "Flow Meter";
  flowmeterDoc["platform"] = "sensor";
  flowmeterDoc["state_topic"] = mqtt_topic_flowmeter_flows;
  flowmeterDoc["suggested_display_precision"] = "1";
  flowmeterDoc["device_class"] = "water";
  flowmeterDoc["state_class"] = "measurement";
  flowmeterDoc["qos"] = 2;
  createAvailabilityInfo(flowmeterDoc);
  createDeviceInfo(flowmeterDoc);

  // The device data itself
  char flowmeterPayload[512];
  serializeJson(flowmeterDoc, flowmeterPayload);
  mqttClient.publish(mqtt_topic_flowmeter_discovery, flowmeterPayload);

  // Reboot buton
  StaticJsonDocument<512> rebootDoc;
  rebootDoc["unique_id"] = "flowmeter_main_btn_reboot";
  rebootDoc["object_id"] = "flowmeter_main_btn_reboot";
  rebootDoc["name"] = "Reboot NodeMCU";
  rebootDoc["command_topic"] = mqtt_topic_flowmeter_op;
  rebootDoc["payload_press"] = "restart";
  rebootDoc["entity_category"] = "config";
  rebootDoc["device_class"] = "restart";
  createDeviceInfo(rebootDoc);

  // Send the Reboot button data
  char rebootPayload[512];
  serializeJson(rebootDoc, rebootPayload);
  mqttClient.publish(mqtt_topic_btn_reboot_discovery, rebootPayload);

  // Refill button
  StaticJsonDocument<512> refillDoc;
  refillDoc["unique_id"] = "flowmeter_main_btn_refill";
  refillDoc["object_id"] = "flowmeter_main_btn_refill";
  refillDoc["name"] = "Reset the volume";
  refillDoc["command_topic"] = mqtt_topic_flowmeter_op;
  refillDoc["payload_press"] = "refill";
  refillDoc["entity_category"] = "config";
  refillDoc["device_class"] = "identify";
  createDeviceInfo(refillDoc);

  // Send the Refill button data
  char refillPayload[512];
  serializeJson(refillDoc, refillPayload);
  mqttClient.publish(mqtt_topic_btn_refill_discovery, refillPayload);

  // Pump switcher
  StaticJsonDocument<512> pumpSwitchDoc;
  pumpSwitchDoc["unique_id"] = "flowmeter_main_switch_pump";
  pumpSwitchDoc["object_id"] = "flowmeter_main_switch_pump";
  pumpSwitchDoc["name"] = "Water pump switcher";
  pumpSwitchDoc["state_topic"] = mqtt_topic_flowmeter_pump;
  pumpSwitchDoc["command_topic"] = mqtt_topic_flowmeter_pump;
  pumpSwitchDoc["payload_on"] = PumpStatusStrings[PUMPING];
  pumpSwitchDoc["payload_off"] = PumpStatusStrings[BURNING];
  pumpSwitchDoc["state_on"] = PumpStatusStrings[PUMPING];
  pumpSwitchDoc["state_off"] = PumpStatusStrings[BURNING];
  pumpSwitchDoc["entity_category"] = "config";
  pumpSwitchDoc["optimistic"] = "true";
  createDeviceInfo(pumpSwitchDoc);

  // Send the Pump switcher data
  char pumpSwitchPayload[512];
  serializeJson(pumpSwitchDoc, pumpSwitchPayload);
  mqttClient.publish(mqtt_topic_switch_pump_discovery, pumpSwitchPayload);
  publishDebugLogs("[MQTT] The discovery config has been sent!");
}

/*
  Logs and debug printouts
*/

void publishDebugLogs(const char* message) {
  Serial.println(message);
  mqttClient.publish(mqtt_topic_flowmeter_logs, message);
}

void publishMqttPumpState(PumpStatus mode) {
  const char* pstate = PumpStatusStrings[mode];

  if (mode != IDLE) {
    mqttClient.publish(mqtt_topic_flowmeter_pump, pstate);
    publishDebugLogs((String("The pump status is: ") + pstate).c_str());
  }
}

void publishMqttStatus(FlowMeterStatus status) {
  mqttClient.publish(mqtt_topic_flowmeter_state, FlowMeterStatusStrings[status]);

  publishDebugLogs("The status has been sent: ");
  publishDebugLogs(FlowMeterStatusStrings[status]);
}

void publishMqttSensorValue(float value) {
  mqttClient.publish(mqtt_topic_flowmeter_flows, String(value).c_str(), true);
}

void publishMqttPumpAlarm() {
  // need this to trigger an emergency actions in HA...
  mqttClient.publish(mqtt_topic_flowmeter_alarm, "alarm");
}

/*
  Main Node MCU callbacks
*/

void setup() {
  setOperationMode(WAITING_FOR_SETUP);

  Serial.begin(9600);
  pinMode(FLOWSENSOR_PIN, INPUT_PULLUP);
  digitalWrite(FLOWSENSOR_PIN, HIGH); // Optional Internal Pull-Up
  pinMode(RELAY_PIN1, OUTPUT);
  digitalWrite(RELAY_PIN1, RELAY_OPEN);

  initializeMemModule();

  attachInterrupt(digitalPinToInterrupt(FLOWSENSOR_PIN), flowImpulseInterrupt, RISING);
  
  flowMeterTimer.register_callback(calculateFlowRate);
  currentMeterTimer.register_callback(meterCurrentDC);

  setupWatchdog();
  setupHomeAssistant();

  shuntSensor.autoMidPoint();

  singleShortBeep();
}

void loop () {
  if (WiFi.status() != WL_CONNECTED) {
    if (isCurrentlyOperating(CONNECTING)) {
      return;
    }

    wifiConnected = false;
    setOperationMode(CONNECTING);
    publishDebugLogs("Connection lost! Reconnecting...");

    setupHomeAssistant();
  }

  httpServer.handleClient();

  flowMeterTimer.run(1000);
  currentMeterTimer.run(3000);

  mqttMainLooper();

  // update watchdog
  ESP.wdtFeed();
}

void reboot() {
  singleLongBeep();
  publishMqttStatus(OFFLINE);
  ESP.restart();
}
