/*
  ESP32-S3 Combined Master Firmware (.ino format for Arduino IDE)
  
  Daftar Library yang wajib diinstal di Arduino IDE:
  1. ArduinoJson (oleh Benoit Blanchon)
  2. PubSubClient (oleh Nick O'Leary)
  3. RTClib (oleh Adafruit)
  4. ModbusMaster (oleh Doc Walker / 4-20ma)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <httpUpdate.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <ModbusMaster.h>

// ==========================================
// 1. PIN DEFINITIONS (ESP32-S3 N16R8)
// ==========================================
#define PIN_BUZZER        7   // Buzzer output (GPIO 7)
#define PIN_LED_DATA      2   // LED Data (GPIO 2)

// IP5306 Battery LED pins (Active HIGH after transistor level shifter)
#define PIN_BAT_LED1      19
#define PIN_BAT_LED2      20
#define PIN_BAT_LED3      3

// RS485 Pins for Sensor Box communication
#define RS485_RX          5   // RX pin (GPIO 5 / RO)
#define RS485_TX          6   // TX pin (GPIO 6 / DI)
#define RS485_DE_RE       4   // DE/RE pin for half-duplex control (GPIO 4 / DE)

// SPI Pins for SD Card
#define PIN_SD_CS         10  // SD Card Chip Select (GPIO 10)
#define PIN_SD_MOSI       11  // SD Card MOSI (GPIO 11)
#define PIN_SD_MISO       13  // SD Card MISO (GPIO 13)
#define PIN_SD_SCK        12  // SD Card SCK (GPIO 12)

// Nextion UART Pins (Serial1)
#define NEXTION_RX        18  // Connected to Nextion TX (GPIO 18)
#define NEXTION_TX        17  // Connected to Nextion RX (GPIO 17)

// I2C Pins for RTC and IP5306
#define PIN_I2C_SDA       8   // SDA pin (GPIO 8)
#define PIN_I2C_SCL       9   // SCL pin (GPIO 9)
#define PIN_IP5306_INT    47  // Interrupt pin (GPIO 47)

// Button pressed state (LOW for active LOW / pull-up, HIGH for active HIGH / pull-down)
#define BUTTON_PRESSED_STATE LOW 

// ==========================================
// RTC DS3231 & IP5306 PMIC I2C UTILITIES
// ==========================================
struct SystemDateTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

RTC_DS3231 rtc;

// IP5306 I2C Battery capacity & charging read
bool detectIP5306() {
  Wire.beginTransmission(0x75);
  return (Wire.endTransmission() == 0);
}

int8_t readIP5306BatteryLevel() {
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(0x75, 1) == 1) {
    uint8_t reg = Wire.read();
    switch (reg & 0xF0) {
      case 0xE0: return 25;
      case 0xC0: return 50;
      case 0x80: return 75;
      case 0x00: return 100;
      default: return 0;
    }
  }
  return -1;
}

int8_t readIP5306IsCharging() {
  Wire.beginTransmission(0x75);
  Wire.write(0x70);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(0x75, 1) == 1) {
    uint8_t reg = Wire.read();
    return (reg & 0x08) ? 1 : 0; // Bit 3: 1=charging, 0=not charging
  }
  return -1;
}

// ==========================================
// 2. CONSTANTS & SYSTEM VARIABLES
// ==========================================
WebServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial nextionSerial(1); // Use Hardware UART 1 for Nextion

ModbusMaster node;

// Spinlock for thread-safe cross-core flow variable access
portMUX_TYPE flowMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t modbusTaskHandle = NULL;

void requestSensorData();

// FreeRTOS Task for Modbus RTU Communication on Core 0
void vModbusTask(void *pvParameters) {
  Serial.println("[RTOS] Modbus RTU Task started on Core 0. 🚀");
  for (;;) {
    requestSensorData();
    vTaskDelay(pdMS_TO_TICKS(50)); // Poll every 50ms untuk transmisi yang lebih cepat
  }
}

void preTransmission() { 
  digitalWrite(RS485_DE_RE, HIGH); 
}

void postTransmission() {
  Serial2.flush(); // Tunggu hingga seluruh byte TX selesai dikirim sebelum DE_RE di-LOW-kan
  delayMicroseconds(100); // Jeda 100us agar bit terakhir selesai ditransmisikan pada 115200 bps
  digitalWrite(RS485_DE_RE, LOW);
}

const char *mqtt_server = "mqtt.clowsens.cloud";
const int mqtt_port = 1883;

// GitHub Repository Settings for OTA updates
const char* gh_owner = "Hahangus22"; // Ganti dengan username GitHub Anda
const char* gh_repo = "Clowsens_Fimware"; // Ganti dengan nama repository GitHub Anda

// Calibration & Settings (Shared)
int deviceID = 1;               // Default
String wifi_ssid = "";
String wifi_pass = "";
String hostname = "Clowsens_" + String(deviceID); // Nama device yang akan muncul
float CALIBRATION_FACTOR = 60;  // Default 60 (stored as puluhan in WebServer/EEPROM, divided by 100.0 in calculations)
float ratio1 = 48;              // Min Ratio Default (corresponds to 0.48)
float ratio2 = 49;              // Level 2 Threshold (0.49)
float ratio3 = 50;              // Level 3 Threshold (0.50)
float ratio4 = 52;              // Max Ratio / Level 4 Threshold (0.52)

// Sensor & Logic Variables
volatile float flowrate = 0.0;
volatile unsigned int pulse = 0;
unsigned int counter_detik = 0;
unsigned char status_alarm = 0;
unsigned char status_buzzer = 0;

unsigned long startMillis, startMillis1, startMillis2;
const unsigned long period = 1000;  // 1 second flow calculation period
#define waktu 1
#define alarm_fast 200
#define alarm_slow 500

// Blood color mapping variables
float red_ratio = 0.0;
float mapped_percent = 0.0;
int level = 0;
unsigned char flow_alarm_status = 0;  // 0=normal, 1=no_flow, 2=low_flow
unsigned char blood_alarm_status = 0; // 0=normal, 1=blood_detected
bool buzzer_active = true;            // Master buzzer state (Nextion controlled)

// Flow sensor variables
unsigned long lastSecondTime = 0;
unsigned int pulse_per_second = 0;
volatile unsigned int pulse_buffer = 0;
float flow_per_second = 0.0;
float flow_per_minute = 0.0;
float volume_total = 0.0;
float volume_session = 0.0;

const float PULSE_TO_ML = 0.01;      // 1 pulse = 0.01 ml
volatile unsigned long lastPulseMicros = 0;
bool pernahAdaFlow = false;

// Hardware Status flags
bool tcs_ok = false;
bool flow_ok = false;
bool mqtt_ok = false;
bool sd_ok = false;
bool short_detected = false; // Short circuit detection flag from slave node
uint16_t slave_error_code = 0; // Detailed error status code from slave node
bool ip5306_detected = false;
SystemDateTime currentDT = {2026, 7, 11, 20, 0, 0};
bool rtc_ok = false;

// AT Command Debug Control (Default MATI/INVISIBLE: AT+DEBUG=0)
bool debugEnabled = false; 

#define debugPrint(...)   do { if (debugEnabled) Serial.print(__VA_ARGS__); } while(0)
#define debugPrintln(...) do { if (debugEnabled) Serial.println(__VA_ARGS__); } while(0)
#define debugPrintf(...)  do { if (debugEnabled) Serial.printf(__VA_ARGS__); } while(0)

// Battery monitoring configuration & variables
#define TEST_RANDOM_BATTERY false // Set false to use actual battery LED reading
const int pinADC = 1; // GPIO 1 (unused but kept for code compatibility)
float teganganHalus = 3.7; // Nilai awal standar
const float ALPHA = 0.05;   // Semakin kecil (misal 0.01), semakin lambat & stabil filternya
int persenLayar = 100;
bool isCharging = false;    // Menandakan status baterai saat ini sedang charging atau discharging
unsigned long waktuTerakhirUpdate = 0;
unsigned long waktuTerakhirChargeAnimate = 0;

// Variables for battery LED pin sampling
int led1Samples[20] = {0};
int led2Samples[20] = {0};
int led3Samples[20] = {0};
int sampleIdx = 0;
unsigned long lastSampleTime = 0;

// SD card and scheduling
int recordNo = 1;
unsigned long lastSD = 0;
const unsigned long intervalSD = 15000;   // 5 seconds SD logging
unsigned long lastMQTT = 0;
const unsigned long mqttInterval = 15000; // 5 seconds MQTT publishing
bool ntpSynced = false;                   // Track NTP sync status (true = calibrated, false = needs sync)

// ==========================================
// 4. NVS (PREFERENCES) LOAD / SAVE FUNCTIONS
// ==========================================
Preferences preferences;

void loadWiFi() {
  preferences.begin("wifi", true);
  wifi_ssid = preferences.getString("ssid", "");
  wifi_pass = preferences.getString("pass", "");
  preferences.end();
  Serial.printf("Loaded WiFi credentials -> SSID: %s\n", wifi_ssid.c_str());
}

void saveWiFi(String ssid, String pass) {
  preferences.begin("wifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("pass", pass);
  preferences.end();
  Serial.println("WiFi Credentials saved to NVS.");
}

void saveCalibration() {
  preferences.begin("calib", false);
  preferences.putFloat("cal", CALIBRATION_FACTOR);
  preferences.putFloat("r1", ratio1);
  preferences.putFloat("r2", ratio2);
  preferences.putFloat("r3", ratio3);
  preferences.putFloat("r4", ratio4);
  preferences.end();
  Serial.println("Calibration saved to NVS.");
}

void loadCalibration() {
  preferences.begin("calib", true);
  CALIBRATION_FACTOR = preferences.getFloat("cal", 60.0);
  ratio1 = preferences.getFloat("r1", 48.0);
  ratio2 = preferences.getFloat("r2", 49.0);
  ratio3 = preferences.getFloat("r3", 50.0);
  ratio4 = preferences.getFloat("r4", 52.0);
  preferences.end();

  if (isnan(CALIBRATION_FACTOR) || CALIBRATION_FACTOR < 1.0) CALIBRATION_FACTOR = 60.0;

  bool needSave = false;
  if (isnan(ratio1) || ratio1 < 1.0 || ratio1 > 100.0) { ratio1 = 48.0; needSave = true; }
  if (isnan(ratio2) || ratio2 < 1.0 || ratio2 > 100.0) { ratio2 = 49.0; needSave = true; }
  if (isnan(ratio3) || ratio3 < 1.0 || ratio3 > 100.0) { ratio3 = 50.0; needSave = true; }
  if (isnan(ratio4) || ratio4 < 1.0 || ratio4 > 100.0) { ratio4 = 52.0; needSave = true; }

  if (needSave) {
    saveCalibration();
  }

  Serial.printf("Loaded calibration from NVS -> ID:%d, Factor:%.2f, R1:%.2f, R2:%.2f, R3:%.2f, R4:%.2f\n", 
                deviceID, CALIBRATION_FACTOR, ratio1, ratio2, ratio3, ratio4);
}

// ==========================================
// 5. COLOR & FLOW SENSOR CALCULATIONS
// ==========================================
void hitung_flow_per_detik() {
  unsigned long currentTime = millis();
  if (currentTime - lastSecondTime >= 1000) {
    lastSecondTime = currentTime;
    
    taskENTER_CRITICAL(&flowMux);
    pulse_per_second = pulse_buffer;
    pulse_buffer = 0;
    taskEXIT_CRITICAL(&flowMux);
 
    flow_per_second = pulse_per_second * PULSE_TO_ML;
    volume_session += flow_per_second;
    volume_total = volume_session;
    flow_per_minute = flow_per_second * 60.0;
    flowrate = flow_per_minute;
  }
}

void requestSensorData() {
  // Read 3 holding registers starting from register 0
  uint8_t result = node.readHoldingRegisters(0, 3);
  
  bool success = false;
  uint16_t pulses_total = 0;
  uint16_t color_val = 0;
  uint16_t short_val = 0;

  if (result == node.ku8MBSuccess) {
    success = true;
    tcs_ok = true;
    
    pulses_total = node.getResponseBuffer(0);
    color_val = node.getResponseBuffer(1);
    short_val = node.getResponseBuffer(2);

    slave_error_code = short_val;
    short_detected = (slave_error_code == 1);

    // Update raw pulses directly (since Slave resets it after read)
    if (pulses_total > 0) {
      taskENTER_CRITICAL(&flowMux);
      pulse += pulses_total;
      pulse_buffer += pulses_total;
      lastPulseMicros = micros();
      taskEXIT_CRITICAL(&flowMux);
    }

    // Run color leveling
    if (color_val > 0) {
      red_ratio = (float)color_val / 10000.0;
      
      float min_ratio = ratio1 / 100.0;
      float max_ratio = ratio4 / 100.0;
      
      mapped_percent = ((red_ratio - min_ratio) / (max_ratio - min_ratio)) * 100.0;
      if (mapped_percent < 0) mapped_percent = 0;
      if (mapped_percent > 100) mapped_percent = 100;

      float r2 = ratio2 / 100.0;
      float r3 = ratio3 / 100.0;
      float r4 = ratio4 / 100.0;

      if (red_ratio < r2) level = 1;
      else if (red_ratio < r3) level = 2;
      else if (red_ratio < (r3 + 0.01)) level = 3;
      else if (red_ratio < r4) level = 4;
      else level = 5;

      blood_alarm_status = level;
    }
  } else {
    tcs_ok = false; // Sensor box offline
  }

  // Rate-limited USB Serial Debug Printing (every 1 second)
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint >= 1000) {
    lastDebugPrint = millis();
    if (success) {
      taskENTER_CRITICAL(&flowMux);
      unsigned int current_pps = pulse_per_second;
      unsigned int current_pulse = pulse;
      taskEXIT_CRITICAL(&flowMux);
      
      debugPrintf("[RS485 RX] OK | Poll Pulses: %u | ESP32 Pulses/sec: %u | Accumulated Pulses (Min): %u | Vol: %.2f ml | Color: %u | Short Det: %d\n", 
                    pulses_total, current_pps, current_pulse, volume_session, color_val, short_detected);
    } else {
      debugPrintf("[RS485 RX] FAIL | Modbus error: 0x%02X ❌\n", result);
    }
  }
}

void reset_volume_session() {
  volume_session = 0.0;
  volume_total = 0.0;
}

// Fungsi Interpolasi LiPo (Kurva tidak linier)
int hitungPersenLiPo(float v) {
  int p = 0;
  if (v >= 4.15) p = 100;
  else if (v >= 4.00) p = 85 + (v - 4.00) * 100; 
  else if (v >= 3.80) p = 40 + (v - 3.80) * 225; 
  else if (v >= 3.70) p = 15 + (v - 3.70) * 250; 
  else if (v >= 3.50) p = 0 + (v - 3.50) * 75;   
  else p = 0;
  
  if (p > 100) p = 100;
  if (p < 0) p = 0;
  return p;
}

// Fungsi Interpolasi LiPo saat Charging (Kurva bergeser karena tegangan pengisian lebih tinggi)
int hitungPersenCharge(float v) {
  int p = 0;
  // Pengisian biasanya sampai 4.2V. Kita map tegangan pengisian 3.6V - 4.2V secara linier/kurva tipis
  if (v >= 4.20) p = 100;
  else if (v >= 4.10) p = 90 + (v - 4.10) * 100; // 4.1V ke 4.2V -> 90% ke 100%
  else if (v >= 3.90) p = 50 + (v - 3.90) * 200; // 3.9V ke 4.1V -> 50% ke 90%
  else if (v >= 3.70) p = 15 + (v - 3.70) * 175; // 3.7V ke 3.9V -> 15% ke 50%
  else if (v >= 3.60) p = 0 + (v - 3.60) * 150;  // 3.6V ke 3.7V -> 0% ke 15%
  else p = 0;

  if (p > 100) p = 100;
  if (p < 0) p = 0;
  return p;
}

// ==========================================
// 6. NEXTION COMMUNICATION
// ==========================================
void sendEnd() {
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void nextionSetVal(const char* id, int v) {
  nextionSerial.print(id);
  nextionSerial.print(".val=");
  nextionSerial.print(v);
  sendEnd();
  // debugPrintf("[Nextion TX] %s.val=%d\n", id, v);
}

void nextionSetTxt(const char* id, const char* txt) {
  nextionSerial.print(id);
  nextionSerial.print(".txt=\"");
  nextionSerial.print(txt);
  nextionSerial.print("\"");
  sendEnd();
  // debugPrintf("[Nextion TX] %s.txt=\"%s\"\n", id, txt);
}

void nextionSetVis(const char* id, bool visible) {
  nextionSerial.print("vis ");
  nextionSerial.print(id);
  nextionSerial.print(",");
  nextionSerial.print(visible ? "1" : "0");
  sendEnd();
}

  // =========================================================================
  // SISTEM KODE ERROR CLOWSENS (00: Normal, 01-10: Error Code)
  // =========================================================================
  // 00: Normal / No Error
  // 01: SD Card Error / Gagal Write
  // 02: RTC DS3231 Error / Jam Disconnect
  // 03: Sensor Warna TCS34725 Error (ESP8266 Slave)
  // 04: Sensor Flow Urin Error (ESP8266 Slave)
  // 05: Modbus RS485 Communication Timeout / Slave Offline
  // 06: Short Circuit Terdeteksi di Slave Node
  // 07: Power Slave Lost / Putus Daya Node
  // 08: WiFi Offline
  // 09: MQTT Broker Offline
  // 10: Baterai Error (Tegangan Baterai LiPo < 3.4V di GPIO 1)
  // =========================================================================

const char* getActiveErrorCode() {
  if (!sd_ok) {
    return "01"; // Gagal Write SD Card / SD Card Offline
  } else if (!rtc_ok) {
    return "02"; // RTC DS3231 Offline
  } else if (tcs_ok && slave_error_code == 2) {
    return "03"; // Sensor Warna TCS34725 Fail
  } else if (tcs_ok && slave_error_code == 3) {
    return "04"; // Sensor Flow Fail
  } else if (!tcs_ok) {
    return "05"; // Modbus RS485 Timeout / Slave Offline / Power Lost
  } else if (short_detected || slave_error_code == 1) {
    return "06"; // Short Circuit Terdeteksi (ADC A0 dari Slave)
  } else if (WiFi.status() != WL_CONNECTED) {
    return "08"; // WiFi Offline
  } else if (!mqtt_ok) {
    return "09"; // MQTT Broker Offline
  } else if (teganganHalus < 3.40 || persenLayar <= 5) {
    return "10"; // Baterai Error (Tegangan LiPo < 3.4V di GPIO 1 / ADC)
  }
  return "00"; // Normal / No Error
}

void nextionUpdateAll() {
  nextionSetVal("Flow",   (int)(flowrate * 100.0));
  nextionSetVal("Vol",    (int)(volume_session * 100.0));
  nextionSetVal("Pulse",  (int)pulse_per_second);
  nextionSetVal("Device", deviceID);
  nextionSetVal("Ratio",  (int)mapped_percent);
  nextionSetVal("Hema",   (int)level);
  nextionSetVal("Bat",    persenLayar);
  
  // Kirim data tegangan baterai ke Nextion (baik sebagai nilai integer * 100 maupun teks string)
  nextionSetVal("Volt",   (int)(teganganHalus * 100.0)); 
  char voltStr[16];
  sprintf(voltStr, "%.2f V", teganganHalus);
  nextionSetTxt("tVolt",  voltStr);

  if (rtc_ok) {
    nextionSetVal("Times", currentDT.hour * 3600 + currentDT.minute * 60 + currentDT.second);
  } else {
    nextionSetVal("Times", (int)(millis() / 1000));
  }

  nextionSetTxt("Flow",   flow_ok ? "Ok" : "No");
  nextionSetTxt("TCS",    tcs_ok ? "Ok" : "No");
  nextionSetTxt("SD",     sd_ok ? "Ok" : "No");
  nextionSetTxt("Cloud",  mqtt_ok ? "Ok" : "No");
  nextionSetTxt("Short",  short_detected ? "Err" : "Ok");

  String ipStr = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "0.0.0.0";
  String ssidStr = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "-";
  nextionSetTxt("IP",     ipStr.c_str());
  nextionSetTxt("SSID",   ssidStr.c_str());

  const char* activeErrCode = getActiveErrorCode();

  if (strcmp(activeErrCode, "00") != 0) {
    nextionSetTxt("Err1", activeErrCode);
    nextionSetVis("Err", true);
    nextionSetVis("Err1", true);
  } else {
    // Kondisi Normal: Sembunyikan gambar segitiga & angka kode error
    nextionSetVis("Err", false);
    nextionSetVis("Err1", false);
  }
}

void checkNextionInput() {
  if (nextionSerial.peek() == 0xAA) return; // Leave for nextionReadCalibration

  static String bufNx = "";
  static unsigned long lastRxTime = 0;

  while (nextionSerial.available()) {
    if (nextionSerial.peek() == 0xAA) return;

    char c = nextionSerial.read();

    if (millis() - lastRxTime > 2000 && bufNx.length() > 0) {
      bufNx = "";
    }
    lastRxTime = millis();

    // Clean noise: shift buffer left until it starts with 'B' or a digit
    while (bufNx.length() > 0) {
      char firstChar = bufNx.charAt(0);
      if (firstChar != 'B' && !isDigit(firstChar)) {
        bufNx.remove(0, 1);
      } else {
        break;
      }
    }

    if (c >= 32 && c <= 126) {
      bufNx += c;
    }

    // Command Parser
    if (bufNx == "BUZ:1") {
      buzzer_active = true;
      Serial.println("Buzzer ENABLED by Nextion");
      bufNx = "";
      continue;
    }
    if (bufNx == "BUZ:0") {
      buzzer_active = false;
      Serial.println("Buzzer DISABLED by Nextion");
      digitalWrite(PIN_BUZZER, LOW);
      bufNx = "";
      continue;
    }

    // 10-Digit calibration text input
    if (bufNx.length() >= 10 && isDigit(bufNx.charAt(0))) {
      bool allDigit = true;
      for (int i = 0; i < 10; i++) {
        if (!isDigit(bufNx.charAt(i))) {
          allDigit = false;
          break;
        }
      }

      if (allDigit) {
        float v1 = bufNx.substring(0, 2).toInt();
        float v2 = bufNx.substring(2, 4).toInt();
        float v3 = bufNx.substring(4, 6).toInt();
        float v4 = bufNx.substring(6, 8).toInt();
        float v5 = bufNx.substring(8, 10).toInt();

        if (v1 > 0) CALIBRATION_FACTOR = v1;
        if (v2 > 0) ratio1 = v2;
        if (v3 > 0) ratio2 = v3;
        if (v4 > 0) ratio3 = v4;
        if (v5 > 0) ratio4 = v5;

        saveCalibration(); // Directly save new settings to EEPROM!
        bufNx = "";
      }
    }

    if (bufNx.length() > 20) bufNx = "";
  }
}

void processATCommand(String input, Stream* out1, Stream* out2) {
  input.trim();
  input.toUpperCase();

  if (input == "AT") {
    out1->println("OK");
    out2->println("OK");
  } 
  else if (input == "AT+DEBUG=1" || input == "AT+DEBUG=ON") {
    debugEnabled = true;
    preferences.begin("sys-debug", false);
    preferences.putBool("debug", true);
    preferences.end();
    out1->println("OK");
    out1->println("[SYSTEM] Debug Serial BERHASIL DIHIDUPKAN! ✅");
    out2->println("OK");
    out2->println("[SYSTEM] Debug Serial BERHASIL DIHIDUPKAN! ✅");
  } 
  else if (input == "AT+DEBUG=0" || input == "AT+DEBUG=OFF") {
    out1->println("[SYSTEM] Debug Serial BERHASIL DIMATIKAN! 🔕");
    out1->println("OK");
    out2->println("[SYSTEM] Debug Serial BERHASIL DIMATIKAN! 🔕");
    out2->println("OK");
    debugEnabled = false;
    preferences.begin("sys-debug", false);
    preferences.putBool("debug", false);
    preferences.end();
  } 
  else if (input == "AT+DEBUG?") {
    out1->print("+DEBUG: ");
    out1->println(debugEnabled ? "1" : "0");
    out1->println("OK");
    out2->print("+DEBUG: ");
    out2->println(debugEnabled ? "1" : "0");
    out2->println("OK");
  }
}

void checkATCommand() {
  static String inputSerial = "";

  // 1. Read from Serial (Native USB or UART0)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (inputSerial.length() > 0) {
        #if ARDUINO_USB_CDC_ON_BOOT
        processATCommand(inputSerial, &Serial, &Serial0);
        #else
        processATCommand(inputSerial, &Serial, &Serial);
        #endif
        inputSerial = "";
      }
    } else {
      inputSerial += c;
    }
  }

  // 2. Read from Serial0 (UART CP2102) - Hanya jika USB CDC On Boot diaktifkan
  #if ARDUINO_USB_CDC_ON_BOOT
  static String inputSerial0 = "";
  while (Serial0.available()) {
    char c = Serial0.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (inputSerial0.length() > 0) {
        processATCommand(inputSerial0, &Serial, &Serial0);
        inputSerial0 = "";
      }
    } else {
      inputSerial0 += c;
    }
  }
  #endif
}

void nextionReadCalibration() {
  if (nextionSerial.peek() != 0xAA) return;

  if (nextionSerial.available() >= 21) {
    nextionSerial.read(); // Consume header 0xAA

    long vals[5];
    for (int i = 0; i < 5; i++) {
      uint8_t b0 = nextionSerial.read();
      uint8_t b1 = nextionSerial.read();
      uint8_t b2 = nextionSerial.read();
      uint8_t b3 = nextionSerial.read();
      vals[i] = (long)b0 | ((long)b1 << 8) | ((long)b2 << 16) | ((long)b3 << 24);
    }

    long n1_val = vals[0];
    long n2_val = vals[1];
    long n3_val = vals[2];
    long n4_val = vals[3];
    long n5_val = vals[4];

    if (n1_val > 0) CALIBRATION_FACTOR = n1_val;
    if (n2_val > 0) ratio1 = n2_val;
    if (n3_val > 0) ratio2 = n3_val;
    if (n4_val > 0) ratio3 = n4_val;
    if (n5_val > 0) ratio4 = n5_val;

    saveCalibration(); // Persist Nextion-set calibration parameters immediately!
    
    Serial.println("✓ Calibration received & stored from Nextion binary block:");
    Serial.printf("  CAL FACTOR = %.2f\n  Ratio 1    = %.2f\n  Ratio 2    = %.2f\n  Ratio 3    = %.2f\n  Ratio 4    = %.2f\n", 
                  CALIBRATION_FACTOR, ratio1, ratio2, ratio3, ratio4);
  }
}

// ==========================================
// 7. WEBSERVER ROUTE HANDLERS
// ==========================================
void handleRoot() {
  String page = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Clowsens Setup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin:0; background:#0f172a; color:#f8fafc; }
      .navbar { background:#1e293b; overflow:hidden; text-align:center; padding:14px 0; border-bottom: 2px solid #334155; }
      .navbar a { color:#cbd5e1; text-decoration:none; padding:14px 20px; font-weight:bold; display:inline-block; transition:0.3s; }
      .navbar a:hover { color:#38bdf8; background:#334155; border-radius:5px; }
      .content { max-width: 600px; margin: 40px auto; padding: 20px; background: #1e293b; border-radius: 12px; border: 1px solid #334155; box-shadow: 0 4px 6px -1px rgb(0 0 0 / 0.1); text-align:center; }
      h2 { color:#38bdf8; margin-top:0; }
      p { color:#94a3b8; line-height: 1.6; }
    </style>
  </head>
  <body>
    <div class="navbar">
      <a href="/">Profil Clowsens</a>
      <a href="/wifi">Setup WiFi</a>
      <a href="/calibration">Kalibrasi</a>
    </div>
    <div class="content">
      <h2>Profil Singkat Clowsens</h2>
      <p>Clowsens adalah sistem sensor cerdas untuk pengelolaan data berbasis IoT. 
      Anda dapat melakukan konfigurasi WiFi and kalibrasi alat langsung melalui halaman ini.</p>
    </div>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", page);
}

void handleWiFi() {
  if (server.method() == HTTP_POST) {
    wifi_ssid = server.arg("ssid");
    wifi_pass = server.arg("pass");
    saveWiFi(wifi_ssid, wifi_pass);
    server.send(200, "text/html", "<body style='background:#0f172a;color:#f8fafc;font-family:sans-serif;text-align:center;padding:50px;'><h2>WiFi disimpan! ESP32-S3 restart...</h2></body>");
    delay(1000);
    ESP.restart();
  } else {
    int n = WiFi.scanNetworks();
    String options = "";
    for (int i = 0; i < n; i++) {
      options += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
    }

    String page = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Setup WiFi - Clowsens</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { font-family: 'Segoe UI', sans-serif; background:#0f172a; color:#f8fafc; margin:0; }
        .navbar { background:#1e293b; overflow:hidden; text-align:center; padding:14px 0; border-bottom: 2px solid #334155; }
        .navbar a { color:#cbd5e1; text-decoration:none; padding:14px 20px; font-weight:bold; display:inline-block; }
        .navbar a:hover { color:#38bdf8; background:#334155; border-radius:5px; }
        .content { max-width: 500px; margin: 40px auto; padding: 30px; background: #1e293b; border-radius: 12px; border: 1px solid #334155; text-align:center; }
        h2 { color:#38bdf8; }
        input, select { padding:10px; margin:10px 0; width:90%; border-radius:6px; border:1px solid #475569; background:#0f172a; color:#f8fafc; }
        input[type=submit] { background:#0284c7; color:white; border:none; cursor:pointer; font-weight:bold; transition: 0.2s; }
        input[type=submit]:hover { background:#0369a1; }
      </style>
    </head>
    <body>
      <div class="navbar">
        <a href="/">Profil Clowsens</a>
        <a href="/wifi">Setup WiFi</a>
        <a href="/calibration">Kalibrasi</a>
      </div>
      <div class="content">
        <h2>Setup WiFi</h2>
        <form method="POST">
          <label style="display:block; text-align:left; margin-left:5%;">Pilih WiFi:</label>
          <select name="ssid">)rawliteral"
                  + options + R"rawliteral(</select><br>
          <label style="display:block; text-align:left; margin-left:5%;">Password:</label>
          <input type="password" name="pass" placeholder="Masukkan password"><br>
          <input type="submit" value="Simpan WiFi">
        </form>
      </div>
    </body>
    </html>
    )rawliteral";

    server.send(200, "text/html", page);
  }
}

void handleCalibration() {
  if (server.method() == HTTP_POST) {
    deviceID = server.arg("devid").toInt();
    CALIBRATION_FACTOR = server.arg("flow").toFloat();
    ratio1 = server.arg("r1").toFloat();
    ratio2 = server.arg("r2").toFloat();
    ratio3 = server.arg("r3").toFloat();
    ratio4 = server.arg("r4").toFloat();

    saveCalibration();

    server.send(200, "text/html",
                R"rawliteral(<body style="background:#0f172a;color:#f8fafc;font-family:sans-serif;text-align:center;padding:50px;">
                <h2>Kalibrasi Disimpan!</h2>
                <p><a href="/calibration" style="color:#38bdf8;text-decoration:none;font-weight:bold;">Kembali ke halaman Kalibrasi</a></p></body>)rawliteral");
  } else {
    String page = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Kalibrasi - Clowsens</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { font-family: 'Segoe UI', sans-serif; background:#0f172a; color:#f8fafc; margin:0; }
        .navbar { background:#1e293b; overflow:hidden; text-align:center; padding:14px 0; border-bottom: 2px solid #334155; }
        .navbar a { color:#cbd5e1; text-decoration:none; padding:14px 20px; font-weight:bold; display:inline-block; }
        .navbar a:hover { color:#38bdf8; background:#334155; border-radius:5px; }
        .content { max-width: 500px; margin: 40px auto; padding: 30px; background: #1e293b; border-radius: 12px; border: 1px solid #334155; text-align:center; }
        h2 { color:#38bdf8; margin-top:0; }
        h3 { color: #94a3b8; border-bottom:1px solid #334155; padding-bottom:5px; text-align:left; }
        input { padding:10px; margin:5px 0; width:90%; border-radius:6px; border:1px solid #475569; background:#0f172a; color:#f8fafc; }
        label { display:block; text-align:left; margin-left:5%; color:#cbd5e1; font-weight:500; margin-top:10px; }
        input[type=submit] { background:#0284c7; color:white; border:none; cursor:pointer; font-weight:bold; margin-top:20px; transition:0.2s; }
        input[type=submit]:hover { background:#0369a1; }
      </style>
    </head>
    <body>
      <div class="navbar">
        <a href="/">Profil Clowsens</a>
        <a href="/wifi">Setup WiFi</a>
        <a href="/calibration">Kalibrasi</a>
      </div>
      <div class="content">
        <h2>Setup Device & Kalibrasi</h2>
        <form method="POST">
          <h3>Device Configuration</h3>
          <label>Device ID:</label>
          <input type="number" name="devid" value=")rawliteral" + String(deviceID) + R"rawliteral(" required>
          
          <h3>Sensor Calibration</h3>
          <label>Flow Factor:</label>
          <input name="flow" value=")rawliteral" + String(CALIBRATION_FACTOR, 2) + R"rawliteral(" step="0.01">
          <label>Ratio 1 (Min):</label>
          <input name="r1" value=")rawliteral" + String(ratio1, 2) + R"rawliteral(" step="0.01">
          <label>Ratio 2:</label>
          <input name="r2" value=")rawliteral" + String(ratio2, 2) + R"rawliteral(" step="0.01">
          <label>Ratio 3:</label>
          <input name="r3" value=")rawliteral" + String(ratio3, 2) + R"rawliteral(" step="0.01">
          <label>Ratio 4 (Max):</label>
          <input name="r4" value=")rawliteral" + String(ratio4, 2) + R"rawliteral(" step="0.01">
          
          <input type="submit" value="SIMPAN PERUBAHAN">
        </form>
      </div>
    </body>
    </html>
    )rawliteral";

    server.send(200, "text/html", page);
  }
}

// ==========================================
// 8. DATA LOGGING (SD CARD)
// ==========================================
void saveToSD() {
  File dataFile = SD.open("/Clowsens.csv", FILE_WRITE);
  if (dataFile) {
    bool isNewFile = (dataFile.size() == 0);
    if (isNewFile) {
      dataFile.println("No,Timestamp,Flow/sec,Flow/min,Flowrate,Vol_session,Vol_total,Alarm_flow,Alarm_comb,Ratio,Level,Bat_V,Bat_Pct,Is_Charging,Short_Det");
    }

    dataFile.print(recordNo++); dataFile.print(",");
    
    // Tulis waktu RTC
    if (rtc_ok) {
      char tBuf[30];
      sprintf(tBuf, "%04d-%02d-%02d %02d:%02d:%02d",
              currentDT.year, currentDT.month, currentDT.day,
              currentDT.hour, currentDT.minute, currentDT.second);
      dataFile.print(tBuf);
    } else {
      dataFile.print("N/A");
    }
    dataFile.print(",");
    
    dataFile.print(flow_per_second); dataFile.print(",");
    dataFile.print(flow_per_minute); dataFile.print(",");
    dataFile.print(flowrate); dataFile.print(",");
    dataFile.print(volume_session); dataFile.print(",");
    dataFile.print(volume_total); dataFile.print(",");
    dataFile.print(flow_alarm_status); dataFile.print(",");
    dataFile.print(status_alarm); dataFile.print(",");
    dataFile.print(red_ratio); dataFile.print(",");
    dataFile.print(level); dataFile.print(",");
    dataFile.print(teganganHalus, 2); dataFile.print(",");
    dataFile.print(persenLayar); dataFile.print(",");
    dataFile.print(isCharging ? 1 : 0); dataFile.print(",");
    dataFile.println(short_detected ? 1 : 0);

    dataFile.close();
    debugPrintln("Data CSV disimpan ke SD Card ✅");
    sd_ok = true;
  } else {
    debugPrintln("Gagal menulis ke SD Card! ❌");
    sd_ok = false;
  }
}

// ==========================================
// 9. MQTT CLIENT INTEGRATION
// ==========================================
// Helper function to dynamically fetch the download URL of the latest .bin release asset from GitHub API
String getLatestReleaseUrl(const char* owner, const char* repo) {
  WiFiClientSecure clientSecure;
  clientSecure.setInsecure(); // Skip certificate check for simplicity
  clientSecure.setTimeout(10000); // 10 seconds connection timeout
  
  HTTPClient http;
  String apiPath = "https://api.github.com/repos/" + String(owner) + "/" + String(repo) + "/releases/latest";
  
  Serial.printf("[OTA] Fetching latest release info from API: %s\n", apiPath.c_str());
  
  http.begin(clientSecure, apiPath);
  http.addHeader("User-Agent", "ESP32-S3-OTA"); // Github API strictly requires User-Agent header
  
  int httpCode = http.GET();
  String binUrl = "";
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    
    // Filter JSON to only keep 'assets' -> 'name' and 'browser_download_url' to conserve heap memory
    StaticJsonDocument<256> filter;
    filter["assets"][0]["name"] = true;
    filter["assets"][0]["browser_download_url"] = true;
    
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, payload, DeserializationOption::Filter(filter));
    
    if (!error) {
      JsonArray assets = doc["assets"].as<JsonArray>();
      for (JsonObject asset : assets) {
        String name = asset["name"].as<String>();
        // Find the first asset ending with .bin
        if (name.endsWith(".bin")) {
          binUrl = asset["browser_download_url"].as<String>();
          break;
        }
      }
    } else {
      Serial.printf("[OTA] JSON parse error: %s\n", error.c_str());
    }
  } else {
    Serial.printf("[OTA] GitHub API request failed. HTTP Code: %d\n", httpCode);
  }
  
  http.end();
  return binUrl;
}

// OTA Update execution function
void performOTAUpdate(const char* binUrl) {
  Serial.printf("[OTA] Starting update from URL: %s\n", binUrl);
  digitalWrite(PIN_BUZZER, LOW); // Mute buzzer immediately during update
  
  WiFiClientSecure clientSecure;
  clientSecure.setInsecure(); // Skip TLS certificate verification for easier GitHub HTTPS access
  clientSecure.setTimeout(15000); // 15 seconds download timeout
  
  // Set progress report callback
  httpUpdate.onProgress([](int cur, int total) {
    static int lastPercent = -1;
    int percent = (cur * 100) / total;
    if (percent % 10 == 0 && percent != lastPercent) {
      lastPercent = percent;
      Serial.printf("[OTA] Progress: %d%%\n", percent);
    }
  });

  t_httpUpdate_return ret = httpUpdate.update(clientSecure, binUrl);
  
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("[OTA] HTTP Update failed! Error (%d): %s\n", 
                    httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA] No updates available.");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("[OTA] Update successful! Rebooting board... 🔄");
      delay(1000);
      ESP.restart();
      break;
  }
}

// MQTT Message Receiver Callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char message[length + 1];
  for (unsigned int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';

  Serial.printf("[MQTT RX] Topic: %s | Payload: %s\n", topic, message);

  if (strcmp(topic, "Clowsens_Update") == 0) {
    String url = String(message);
    url.trim();

    // Check if payload contains a direct URL, otherwise fetch latest release dynamically from GitHub API
    if (url.startsWith("http://") || url.startsWith("https://")) {
      Serial.println("[OTA] Valid URL in payload. Triggering update...");
      performOTAUpdate(url.c_str());
    } else {
      Serial.println("[OTA] Requesting latest release download URL from GitHub API...");
      String latestReleaseUrl = getLatestReleaseUrl(gh_owner, gh_repo);
      
      if (latestReleaseUrl.length() > 0 && (latestReleaseUrl.startsWith("http://") || latestReleaseUrl.startsWith("https://"))) {
        Serial.printf("[OTA] Found latest binary URL: %s\n", latestReleaseUrl.c_str());
        performOTAUpdate(latestReleaseUrl.c_str());
      } else {
        Serial.println("[OTA] Failed to resolve latest release URL from GitHub API! ❌");
      }
    }
  }
}

void reconnectMQTT() {
  if (!client.connected()) {
    String clientId = "Clowsens_" + String(deviceID);
    if (client.connect(clientId.c_str())) {
      debugPrintln("MQTT Connected: " + clientId + " ✅");
      
      // Subscribe to Clowsens_Update topic for OTA firmware updates
      client.subscribe("Clowsens_Update");
      debugPrintln("Subscribed to MQTT Clowsens_Update topic successfully.");
      
      mqtt_ok = true;
    } else {
      debugPrint("MQTT connection failed, state=");
      debugPrint(client.state());
      debugPrintln(". Will try again next cycle.");
      mqtt_ok = false;
    }
  }
}

void publishMQTT() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    
    if (client.connected()) {
      StaticJsonDocument<768> doc;
      doc["device"] = String(deviceID);
      
      JsonObject data = doc.createNestedObject("data");
      data["device_id"] = deviceID;
      
      // Timestamp dari RTC DS3231
      if (rtc_ok) {
        char tBuf[30];
        sprintf(tBuf, "%04d-%02d-%02d %02d:%02d:%02d",
                currentDT.year, currentDT.month, currentDT.day,
                currentDT.hour, currentDT.minute, currentDT.second);
        data["timestamp"] = String(tBuf);
      } else {
        data["timestamp"] = "N/A";
      }

      JsonObject flow = data.createNestedObject("flow_data");
      flow["flow_per_second"] = flow_per_second;
      flow["flow_per_minute"] = flow_per_minute;
      flow["flowrate"] = flowrate;
      flow["volume_session"] = volume_session;
      flow["volume_total"] = volume_total;
      
      JsonObject alarm = data.createNestedObject("alarms");
      alarm["flow"] = flow_alarm_status;
      alarm["combined"] = status_alarm;
      alarm["short_detected"] = short_detected;
      
      JsonObject darah = data.createNestedObject("darah");
      darah["ratio"] = red_ratio;
      darah["leveling"] = level;
      
      JsonObject battery = data.createNestedObject("battery");
      battery["voltage"] = teganganHalus;
      battery["percentage"] = persenLayar;
      battery["is_charging"] = isCharging;
      
      // Kode Error Sistem Clowsens ("00" = Normal, "01"-"09" = Error Code)
      data["error_code"] = getActiveErrorCode();
      
      String payload;
      serializeJson(doc, payload);
      
      String topic = "Clowsens/Data/" + String(deviceID);
      if (client.publish(topic.c_str(), payload.c_str())) {
        debugPrintln("MQTT Published successfully: " + topic);
        mqtt_ok = true;
      } else {
        debugPrintln("MQTT Publish failed! ❌");
        mqtt_ok = false;
      }
    }
  }
}

// FreeRTOS Task for MQTT communication on Core 0 (Thread-safe background task)
void vMqttTask(void *pvParameters) {
  Serial.println("[RTOS] MQTT Task started on Core 0. 🌐");
  unsigned long lastPub = 0;
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnectMQTT();
      }
      
      if (client.connected()) {
        client.loop();
        
        if (millis() - lastPub >= mqttInterval) {
          lastPub = millis();
          publishMQTT();
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // check every 100ms
  }
}

// ==========================================
// 10. SETUP FUNCTION
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Load saved AT Debug mode (Default MATI/INVISIBLE = false)
  preferences.begin("sys-debug", true);
  debugEnabled = preferences.getBool("debug", false);
  preferences.end();

  // Nextion Serial init on HW UART1 (18=RX, 17=TX) at 9600 baud
  nextionSerial.begin(9600, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  
  pinMode(PIN_BAT_LED1, INPUT_PULLUP);
  pinMode(PIN_BAT_LED2, INPUT_PULLUP);
  pinMode(PIN_BAT_LED3, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Initialize LED Data pin
  pinMode(PIN_LED_DATA, OUTPUT);
  digitalWrite(PIN_LED_DATA, LOW);

  // Initialize I2C for RTC & IP5306
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  pinMode(PIN_IP5306_INT, INPUT_PULLUP);

  // Detect IP5306 PMIC
  ip5306_detected = detectIP5306();
  if (ip5306_detected) {
    Serial.println("IP5306 PMIC detected on I2C bus! 🔋");
  } else {
    Serial.println("IP5306 PMIC not found. Using analog ADC voltage divider. ⚡");
  }

  // Initialize and check RTC DS3231 using RTClib
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC DS3231! ❌");
    rtc_ok = false;
  } else {
    rtc_ok = true;
    if (rtc.lostPower() || rtc.now().year() < 2026) {
      Serial.println("RTC lost power or invalid year! Syncing to compile time...");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    // Read initial time
    DateTime now = rtc.now();
    currentDT.year = now.year();
    currentDT.month = now.month();
    currentDT.day = now.day();
    currentDT.hour = now.hour();
    currentDT.minute = now.minute();
    currentDT.second = now.second();
    
    Serial.printf("RTC DS3231 connected. Current Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                  currentDT.year, currentDT.month, currentDT.day,
                  currentDT.hour, currentDT.minute, currentDT.second);
  }

  loadWiFi();
  loadCalibration();

  // Baca awal banget buat set basis tegangan dari pin LED IP5306 (Active HIGH setelah transistor inverter)
  bool initL1 = (digitalRead(PIN_BAT_LED1) == HIGH);
  bool initL2 = (digitalRead(PIN_BAT_LED2) == HIGH);
  bool initL3 = (digitalRead(PIN_BAT_LED3) == HIGH);

  if (initL3) {
    persenLayar = 100;
    teganganHalus = 4.2;
  } else if (initL2) {
    persenLayar = 50;
    teganganHalus = 3.8;
  } else if (initL1) {
    persenLayar = 25;
    teganganHalus = 3.6;
  } else {
    persenLayar = 0;
    teganganHalus = 3.4;
  }
  isCharging = false;

  // Initialize RS485 Serial2 (115200 bps) for Sensor Box Communication
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW); // Start in Receive Mode
  Serial2.begin(115200, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_RX, INPUT_PULLUP); // Enable pull-up on RX pin to suppress line noise
  
  node.begin(0x10, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("RS485 Serial2 and ModbusMaster Initialized. ✅");
  tcs_ok = false;

  // Initialize SPI & SD Card
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD card initialization failed! ❌");
    sd_ok = false;
  } else {
    Serial.println("SD card initialized successfully! ✅");
    sd_ok = true;
  }

  // Flow Sensor initial state
  pulse = 0;

  // Timers & counters initialization
  startMillis = millis();
  lastSecondTime = millis();
  startMillis1 = millis();
  startMillis2 = millis();
  lastSD = millis();
  lastMQTT = millis();
  
  counter_detik = 0;
  status_buzzer = 0;
  status_alarm = 0;
  flow_alarm_status = 0;
  volume_session = 0.0;
  volume_total = 0.0;

  // AP Mode force trigger disabled (forceSetup always false)
  bool forceSetup = false;

  if (!forceSetup) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    
    unsigned long startConn = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startConn < 10000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi Connected. IP ESP: ");
      Serial.println(WiFi.localIP().toString());
      configTime(7 * 3600, 0, "id.pool.ntp.org", "pool.ntp.org");
      Serial.println("NTP Configured for Western Indonesia Time (GMT+7).");
    } else {
      Serial.println("WiFi connection timeout. Reverting to Access Point Mode.");
    }
  }

  if (WiFi.status() != WL_CONNECTED || forceSetup) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Clowsens_Setup");
    Serial.println("ESP32-S3 configured as Access Point: Clowsens_Setup");
  }

  // Web Server Route registrations
  server.on("/", handleRoot);
  server.on("/wifi", handleWiFi);
  server.on("/calibration", handleCalibration);
  server.begin();

  // MQTT Server registration
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  // Create Modbus polling task on Core 0 (Wi-Fi and TCP/IP Core)
  xTaskCreatePinnedToCore(
    vModbusTask,
    "ModbusTask",
    4096,
    NULL,
    2, // Priority 2 (above normal priority 1)
    &modbusTaskHandle,
    0  // Pinned to Core 0
  );

  // Create MQTT communication task on Core 0
  xTaskCreatePinnedToCore(
    vMqttTask,
    "MqttTask",
    8192,
    NULL,
    1, // Priority 1 (normal)
    NULL,
    0  // Pinned to Core 0
  );

  Serial.println("Clowsens Combined Firmware Initialized successfully.");
}

// ==========================================
// 11. MAIN LOOP
// ==========================================
void loop() {
  // Sampling battery LED pins every 50ms (Active HIGH setelah transistor inverter)
  if (millis() - lastSampleTime >= 50) {
    lastSampleTime = millis();
    led1Samples[sampleIdx] = digitalRead(PIN_BAT_LED1);
    led2Samples[sampleIdx] = digitalRead(PIN_BAT_LED2);
    led3Samples[sampleIdx] = digitalRead(PIN_BAT_LED3);
    sampleIdx = (sampleIdx + 1) % 20;
  }

  // Sync RTC with NTP time if connected and not yet synced
  if (!ntpSynced && WiFi.status() == WL_CONNECTED) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10)) {
      rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, 
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
      ntpSynced = true;
      Serial.println("RTC successfully synced with NTP Server! ⏰");
    }
  }

  checkATCommand(); // Process AT+DEBUG commands from USB Serial
  server.handleClient();
  
  // Read inputs from Nextion Screen
  nextionReadCalibration();
  checkNextionInput();

  // Run flow calculations per second
  hitung_flow_per_detik();

  // Poll sensor data handled by vModbusTask on Core 0

  // Read DS3231 RTC Time using RTClib (Setiap 1 detik saja untuk mengurangi kepadatan I2C)
  static unsigned long lastRTCRead = 0;
  if (millis() - lastRTCRead >= 1000) {
    lastRTCRead = millis();

    // Cek koneksi I2C DS3231 secara real-time (Address 0x68)
    Wire.beginTransmission(0x68);
    if (Wire.endTransmission() == 0) {
      rtc_ok = true;
      DateTime now = rtc.now();
      currentDT.year = now.year();
      currentDT.month = now.month();
      currentDT.day = now.day();
      currentDT.hour = now.hour();
      currentDT.minute = now.minute();
      currentDT.second = now.second();
      
      // Deteksi pergantian hari untuk re-kalibrasi NTP ulang
      static int lastDay = -1;
      if (lastDay != -1 && currentDT.day != lastDay) {
        ntpSynced = false; // Set ke false agar loop melakukan sinkronisasi NTP lagi
        Serial.println("[SYSTEM] Hari baru terdeteksi! Menjadwalkan kalibrasi ulang waktu via NTP... ⏰");
      }
      lastDay = currentDT.day;
      
      debugPrintf("[RTC Time] %04d-%02d-%02d %02d:%02d:%02d\n",
                    currentDT.year, currentDT.month, currentDT.day,
                    currentDT.hour, currentDT.minute, currentDT.second);
    } else {
      rtc_ok = false;
      debugPrintln("[RTC] DS3231 Disconnected / Unplugged! ❌");
    }
  }

  // Battery monitoring logic (Membaca pin LED IP5306 dengan filter sampling) - Dievaluasi setiap 5 detik
  static unsigned long lastBatteryRead = 0;
  if (millis() - lastBatteryRead >= 5000) {
    lastBatteryRead = millis();
    
    // Hitung jumlah pembacaan HIGH (LED ON setelah level shifter NPN) dari buffer sampling
    int l1HighCount = 0;
    int l2HighCount = 0;
    int l3HighCount = 0;
    
    for (int i = 0; i < 20; i++) {
      if (led1Samples[i] == HIGH) l1HighCount++;
      if (led2Samples[i] == HIGH) l2HighCount++;
      if (led3Samples[i] == HIGH) l3HighCount++;
    }
    
    // Status LED: 0 = Solid OFF, 1 = Solid ON, 2 = Blinking (Kedip-kedip saat charging)
    int s1 = 0;
    int s2 = 0;
    int s3 = 0;
    
    if (l1HighCount >= 18) s1 = 1;
    else if (l1HighCount >= 2) s1 = 2;
    
    if (l2HighCount >= 18) s2 = 1;
    else if (l2HighCount >= 2) s2 = 2;
    
    if (l3HighCount >= 18) s3 = 1;
    else if (l3HighCount >= 2) s3 = 2;
    
    // Deteksi status charging: jika ada salah satu LED yang berkedip
    isCharging = (s1 == 2 || s2 == 2 || s3 == 2);
    
    // Map persentase layar & tegangan berdasarkan LED tertinggi yang aktif
    if (s3 == 1 || s3 == 2) {
      if (s3 == 1) {
        persenLayar = 100;
        teganganHalus = 4.2;
      } else {
        persenLayar = 75; // Blinking di LED 3 menandakan sedang charging menuju 75%
        teganganHalus = 4.0;
      }
    } else if (s2 == 1 || s2 == 2) {
      persenLayar = 50;
      teganganHalus = 3.8;
    } else if (s1 == 1 || s1 == 2) {
      persenLayar = 25;
      teganganHalus = 3.6;
    } else {
      persenLayar = 0;
      teganganHalus = 3.4;
    }
    
    debugPrintf("[BATTERY LED] Samples HIGH - L1: %d/20, L2: %d/20, L3: %d/20 | States - S1: %d, S2: %d, S3: %d | Pct: %d, Charging: %d\n",
                l1HighCount, l2HighCount, l3HighCount, s1, s2, s3, persenLayar, isCharging);
  }

  // 4. Logika Update Layar Nextion Baterai (Tiap 3 detik untuk test data random, atau 10 detik normal)
  #if TEST_RANDOM_BATTERY
  const unsigned long intervalUpdateBat = 3000;
  #else
  const unsigned long intervalUpdateBat = 10000;
  #endif

  if (millis() - waktuTerakhirUpdate >= intervalUpdateBat) {
    waktuTerakhirUpdate = millis();
    
  #if TEST_RANDOM_BATTERY
    // Mode Uji Coba: Mengirim nilai 25, 50, 75, 100 secara acak ke Nextion (object Bat)
    static const int testBatVals[] = {25, 50, 75, 100};
    persenLayar = testBatVals[random(0, 4)];
    debugPrintf("[TEST BATTERY] Sending Bat.val = %d\n", persenLayar);
  #endif

    if (isCharging) {
      nextionSetTxt("t1", "Isi Daya...");
    } else {
      nextionSetTxt("t1", "");
    }

    // Kirim nilai baterai ke komponen 'Bat' di Nextion (objname: Bat)
    nextionSetVal("Bat", persenLayar);
  }

  // Average flow rate computation (Once every minute)
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis >= period) {
    startMillis = currentMillis;
    
    taskENTER_CRITICAL(&flowMux);
    unsigned int temp_pulse = pulse;
    pulse = 0;
    taskEXIT_CRITICAL(&flowMux);

    // Multiply average pulse by calibration factor (stored in EEPROM, divided by 100)
    flowrate = temp_pulse * (CALIBRATION_FACTOR / 100.0);

    // Evaluate flow alarms
    if (flowrate == 0) {
      counter_detik++;
      if (counter_detik >= waktu) {
        flow_alarm_status = 1; // No Flow
      }
    } else if (flowrate <= 3.0) {
      counter_detik++;
      if (counter_detik >= waktu) {
        flow_alarm_status = 2; // Low Flow
      }
    } else {
      counter_detik = 0;
      flow_alarm_status = 0; // Normal Flow
    }
  }

  // Set flow sensor hardware status (Active if pulses received in last 5 seconds)
  if (micros() - lastPulseMicros > 5000000) {
    flow_ok = false;
  } else {
    flow_ok = true;
    pernahAdaFlow = true;
  }

  // Determine combined alarm state
  if (level > 1) {
    status_alarm = 3; // Blood detected (Highest priority)
  } else {
    status_alarm = flow_alarm_status; // 0 = normal, 1 = no flow, 2 = low flow
  }

  // Write Data to SD Card (Every 5 seconds)
  if (millis() - lastSD >= intervalSD) {
    lastSD = millis();
    saveToSD();
  }

  // Update Nextion Display (Every 1000ms / 1 second)
  static unsigned long lastNx = 0;
  if (millis() - lastNx >= 1000) {
    lastNx = millis();
    nextionUpdateAll();
  }

  // Buzzer Alarm Management (Hanya aktif untuk Hematuria > 3)
  if (level > 3) {
    unsigned long alarm_interval = (level == 4) ? alarm_slow : alarm_fast;
    unsigned long currentMillisBuz = millis();
    if (currentMillisBuz - startMillis1 >= alarm_interval) {
      startMillis1 = currentMillisBuz;
      status_buzzer = !status_buzzer;
      
      if (buzzer_active) {
        digitalWrite(PIN_BUZZER, status_buzzer);
      } else {
        digitalWrite(PIN_BUZZER, LOW);
      }
    }
  } else {
    // Normal state atau level <= 3 -> Matikan buzzer
    digitalWrite(PIN_BUZZER, LOW);
  }

  // Toggle LED Data to indicate the loop has completed successfully
  digitalWrite(PIN_LED_DATA, !digitalRead(PIN_LED_DATA));

  // Yield to allow background tasks (like WiFi / TCP stack) to run smoothly
  delay(10);
}
