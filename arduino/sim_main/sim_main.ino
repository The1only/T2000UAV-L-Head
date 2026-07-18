/**
 * @file imu_device.cpp
 * @brief ESP32 Telnet bridge + SSDP (UPnP) device discovery + optional IMU simulator.
 *
 * This firmware exposes the ESP32-S3 as:
 *  - A Telnet-based IMU/radar/transponder data source (WIT protocol packets)
 *  - A UPnP/SSDP-discoverable network sensor
 *  - A simulator (if SIMULATE enabled)
 *  - We use "Adafruit Feather ESP32-S3 2MB PSRAM"
 *
 * It also provides:
 *  - USB serial command interface for setting Wi-Fi credentials
 *  - Persistent Wi-Fi storage using NVS (Preferences)
 *  - SSDP responder for discovery by Qt, Android, macOS, and iOS hosts
 *
 * Main components:
 *  - @ref handleSsdpReceive() : Responds to SSDP M-SEARCH discovery
 *  - @ref sendPacket()        : Sends binary IMU packets to Telnet clients
 *  - @ref handleSerialConfig(): Interactive CLI for WiFi setup
 *  - @ref connectWiFi()       : Multi-AP WiFi connection logic
 */

#include <math.h>
#include <Preferences.h>
#include <WiFiMulti.h>
#include <WiFiUDP.h>
#include <WiFi.h>
#include <Adafruit_BMP280.h>
#include "esp_task_wdt.h"
#define SIM true;
#define USE_WIFI 

#define SENSOR_RADAR 1
#define SENSOR_IMU 2
#define SENSOR_TRANSPONDER 3
#define SENSOR_ALTIMETER 4
#define SENSOR_AIRSPEED 5
#define SENSOR_ANGLE 6

#define SENSOR_V SENSOR_AIRSPEED  // set this manually or with build flags
#define EXTRA_SENSOR_V SENSOR_ANGLE


#if SENSOR_V == SENSOR_RADAR
#define SENSOR "RADAR"  //              ///< Sensor identifier reported in SSDP USN
#define ESP32S3

#elif SENSOR_V == SENSOR_IMU
#define SENSOR "IMU"  //              ///< Sensor identifier reported in SSDP USN
#define ESP32C3

#elif SENSOR_V == SENSOR_TRANSPONDER
#define SENSOR "T2000U"  //              ///< Sensor identifier reported in SSDP USN
#define SIM true;
#define ESP32S3

#elif SENSOR_V == SENSOR_ALTIMETER
#define SENSOR "ALTIMETER"  //              ///< Sensor identifier reported in SSDP USN
#define ESP32S3

#elif SENSOR_V == SENSOR_AIRSPEED
#define SENSOR "AIRSPEED"  //              ///< Sensor identifier reported in SSDP USN
#define ESP32S3
#endif

#ifdef ESP32C3
#define USE_OLED 1
#elif defined(ESP32S3)
#define USE_FastLED 1
#endif

#define SENSOR_TYPE "Airplane-device"  ///< SSDP search target (ST)

#define MAX_SRV_CLIENTS 4  ///< Maximum simultaneous Telnet clients
#define USE_SERIAL true    ///< Use UART bridge instead of IMU simulation

// --- Put these helpers somewhere (top of file) ---
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef USE_FastLED
#include <FastLED.h>
#define NUM_LEDS 1
#define DATA_PIN 48
CRGB leds[NUM_LEDS];
#endif

#ifdef USE_OLED
#include <U8g2lib.h>
#define SDA_PIN_D 5
#define SCL_PIN_D 6
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);  // EastRising 0.42" OLED
#endif

#define WDT_TIMEOUT 5  // seconds

bool SIMULATE = SIM;
bool SerialStat = false;

int sensorValue = 49999;  // variable to store the value coming from the angle sensor
int sensorPin = A0;   // select the input pin for the potentiometer

// -----------------------------------------------------------------------------
// Timing / LED
// -----------------------------------------------------------------------------
static const uint16_t PERIOD_MS = 20;  ///< Loop delay when simulating IMU (~50 Hz)

// -----------------------------------------------------------------------------
// Wi-Fi Credential Storage (NVS)
// -----------------------------------------------------------------------------

Preferences prefs;  ///< ESP32 NVS storage instance

const char *PREF_NAMESPACE = "wifi";  ///< Namespace in NVS for Wi-Fi settings
const char *KEY_SSID = "ssid";        ///< Key for stored SSID
const char *KEY_PASS = "pass";        ///< Key for stored password

bool hasStoredWifi = false;  ///< True if credentials exist in NVS
String storedSsid;           ///< Stored SSID value
String storedPass;           ///< Stored password value

String serialLine;  ///< Input buffer for USB-serial commands

// -----------------------------------------------------------------------------
// WiFi / Telnet Server
// -----------------------------------------------------------------------------

WiFiMulti wifiMulti;  ///< Manages multiple fallback WiFi APs

// Hardcoded fallback Wi-Fi APs (optional)
const char *ssid0 = "x9Tek_printer";
const char *pass0 = "xdidiinne";

const char *ssid1 = "xHvattum";
const char *pass1 = "xJordvarme@2023@";

const char *ssid2 = "xAltibox177449";
const char *pass2 = "xHheX9Xac";

const char *ssid3 = "xAeros2";
const char *pass3 = "xTerjenilsen1";

const char *ssid4 = "xBakeriet";
const char *pass4 = "xkaffe og boller";

WiFiServer server(23);                      ///< Telnet server
WiFiClient serverClients[MAX_SRV_CLIENTS];  ///< Connected clients array

// -----------------------------------------------------------------------------
// SSDP / UPnP Discovery
// -----------------------------------------------------------------------------

/**
 * @brief SSDP multicast address as defined by UPnP (239.255.255.250:1900)
 */
const IPAddress SSDP_MCAST_ADDR("255.255.255.255");

/**
 * @brief Port for UPnP discovery (1900)
 */
const uint16_t SSDP_PORT = 4210;

/**
 * @brief SSDP Search Target (ST) string (e.g. "imu-device", "radar-device")
 */
const char *SSDP_ST = SENSOR_TYPE;

WiFiUDP ssdpUdp;  ///< UDP socket for SSDP traffic

bool active = true;  //false;

// -----------------------------------------------------------------------------
// Setup / Loop
// -----------------------------------------------------------------------------

/**
 * @brief Arduino setup. Initializes Wi-Fi, SSDP, Telnet, USART (optional).
 */
void setup() {
  Serial.begin(115200);

  uint32_t start = millis();
  while (!Serial && millis() - start < 5000) {
    delay(10);
  }

  Serial.print("v1.0 -> ");
  Serial.println(SENSOR);

  // Set up LED...
#ifdef USE_FastLED
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
#endif

#ifdef USE_OLED
  Wire.begin(SDA_PIN_D, SCL_PIN_D);
  u8g2.begin();

  u8g2.clearBuffer();                  // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font

  //  Serial.print("v1.0 -> ");
  //  Serial.println(SENSOR);

  u8g2.drawStr(0, 10, "Sensor v1.0a");  // write something to the internal memory
  u8g2.drawStr(0, 20, SENSOR);          // write something to the internal memory
  u8g2.sendBuffer();                    // transfer internal memory to the display
#endif

  // Initialize Task Watchdog
  //  esp_task_wdt_config_t wdt_config = {
  //    .timeout_ms = WDT_TIMEOUT * 1000,
  //    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // watch both cores
  //    .trigger_panic = true                             // reset on timeout
  //  };
  // esp_task_wdt_init(&wdt_config);
  // Add loop() task to watchdog
  // esp_task_wdt_add(NULL);

#ifdef USE_WIFI
  loadStoredCredentials();
  addAccessPoints();

  Serial.println("Hitt a key to stop booting...");
  for (int i = 0; i < 5000; i++) {
    if (Serial.available()) {
      Serial.println("Entering Config mode...");
      while (handleSerialConfig() == false);
    }
    if (SerialStat == true) break;
    delay(1);
  }

  if (SerialStat == false) {
    // --- optional one-time scan to verify SSID exists ---
    do {

      if (Serial.available()) {
        SerialStat = true;
        break;
      }

      WiFi.onEvent(WiFiEvent);
      WiFi.mode(WIFI_STA);
      WiFi.disconnect(true);  // clear old connection
      delay(100);

      Serial.println("Scanning for networks...");
      int n = WiFi.scanNetworks();
      if (n <= 0) {
        Serial.println("No networks found");
      } else {
        Serial.printf("%d networks found:\n", n);
        for (int i = 0; i < n; ++i) {
          Serial.printf("  %2d: %s (%d dBm)%s\n",
                        i + 1,
                        WiFi.SSID(i).c_str(),
                        WiFi.RSSI(i),
                        (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "" : " *");
        }
      }
    } while (!connectWiFi());

    ssdpUdp.beginMulticast(SSDP_MCAST_ADDR, SSDP_PORT);
    server.begin();
    server.setNoDelay(true);
  }
#else
  SerialStat = true;
#endif

#if SENSOR_V == SENSOR_RADAR
#ifdef ESP32C3
  Serial1.begin(115200, SERIAL_8N1, 20, 21);  // Baud, format, RX pin, TX pin (check your board!)
#else
  Serial1.begin(115200, SERIAL_8N1, 4, 5);  // Baud, format, RX pin, TX pin (check your board!)
#endif
  while (Serial1.available() > 0) {
    Serial1.read();
  }
  setupRADAR();

#elif SENSOR_V == SENSOR_TRANSPONDER
#ifdef ESP32C3
  Serial1.begin(9600, SERIAL_8N1, 20, 21);  // Baud, format, RX pin, TX pin (check your board!)
#else
  Serial1.begin(9600, SERIAL_8N1, 5, 4);  // Baud, format, RX pin, TX pin (check your board!)
#endif
  while (Serial1.available() > 0) {
    Serial1.read();
  }
  setupTRANSPONDER();

#elif SENSOR_V == SENSOR_IMU
#ifdef ESP32C3
  Serial1.begin(9600, SERIAL_8N1, 20, 21);  // Baud, format, RX pin, TX pin (check your board!)
#else
  Serial1.begin(9600, SERIAL_8N1, 5, 4);  // Baud, format, RX pin, TX pin (check your board!)
#endif
                                            //  imu_setup();
  while (Serial1.available() > 0) {
    Serial1.read();
  }
  Serial.println("Serial Port initiated...");

#elif SENSOR_V == SENSOR_ALTIMETER
  altimeter_setup();

#elif SENSOR_V == SENSOR_AIRSPEED
  airspeed_setup();
#endif

  Serial.println(" Booted...");
}

// -----------------------------------------------------------------------------
// Wi-Fi Credential Management (NVS)
// -----------------------------------------------------------------------------
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  //  Serial.print("WiFiEvent:");
  if (Serial.available() > 0) return;
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.print("DISCONNECTED reason=");
    Serial.println(info.wifi_sta_disconnected.reason);
  }
}

/**
 * @brief Loads Wi-Fi SSID/password from ESP32 NVS storage.
 *
 * Populates @ref storedSsid, @ref storedPass and sets @ref hasStoredWifi accordingly.
 */
void loadStoredCredentials() {
  prefs.begin(PREF_NAMESPACE, true);  // read-only
  storedSsid = prefs.getString(KEY_SSID, "");
  storedPass = prefs.getString(KEY_PASS, "");
  prefs.end();

  if (storedSsid.length() > 0) {
    hasStoredWifi = true;
    Serial.printf("Found stored WiFi credentials: %s\n", storedSsid.c_str());
  } else {
    hasStoredWifi = false;
    Serial.println("No stored WiFi credentials.");
    storedSsid = "";
    storedPass = "";
  }
}

/**
 * @brief Saves new Wi-Fi credentials into NVS.
 * @param ssid SSID string
 * @param pass Password string
 */
void saveCredentials(const String &ssid, const String &pass) {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putString(KEY_SSID, ssid);
  prefs.putString(KEY_PASS, pass);
  prefs.end();

  Serial.println("WiFi credentials saved to NVS.");
}

/**
 * @brief Clears stored Wi-Fi credentials from NVS.
 */
void clearCredentials() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  hasStoredWifi = false;
  storedSsid.clear();
  storedPass.clear();
  Serial.println("Stored WiFi credentials erased.");
}

/**
 * @brief Adds stored Wi-Fi credentials (if any) and hardcoded fallback APs.
 */
void addAccessPoints() {
  if (hasStoredWifi)
    wifiMulti.addAP(storedSsid.c_str(), storedPass.c_str());

  wifiMulti.addAP(ssid0, pass0);
  wifiMulti.addAP(ssid1, pass1);
  wifiMulti.addAP(ssid2, pass2);
  wifiMulti.addAP(ssid3, pass3);
  wifiMulti.addAP(ssid4, pass4);
}

/**
 * @brief Attempts to connect to Wi-Fi using @ref WiFiMulti.
 * @return true on success, false if connection fails after multiple retries
 */
bool connectWiFi() {
  Serial.println("Connecting WiFi...");
  for (int loops = 20; loops > 0; --loops) {
    if (wifiMulti.run() == WL_CONNECTED) {
      Serial.print("WiFi connected. IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Connected SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI());

#ifdef USE_OLED
      u8g2.drawStr(0, 30, WiFi.SSID().c_str());                // write something to the internal memory
      u8g2.drawStr(0, 40, WiFi.localIP().toString().c_str());  // write something to the internal memory
      u8g2.sendBuffer();                                       // transfer internal memory to the display
#endif
      return true;
    } else {
      if (Serial.available() > 0) {
        return false;
      }
    }
    delay(400);
  }
  return false;
}

// -----------------------------------------------------------------------------
// SSDP Discovery Handling
// -----------------------------------------------------------------------------

/**
 * @brief Parses incoming SSDP (UPnP) M-SEARCH requests and sends a proper SSDP
 *        response if the Search Target (ST) matches our @ref SENSOR_TYPE.
 *
 * SSDP flows:
 *   1. Host sends M-SEARCH to 239.255.255.250:1900
 *   2. Device responds unicast to the sender with a 200 OK SSDP packet.
 *
 * @return true if a valid request was processed, false otherwise.
 */
bool handleSsdpReceive() {
  int packetSize = ssdpUdp.parsePacket();
  if (packetSize <= 0) {
    delay(10);
    return false;
  }

  char buf[1024];
  int len = ssdpUdp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return false;
  buf[len] = '\0';

  String req(buf);

  if (!req.startsWith("M-SEARCH") && req.indexOf("M-SEARCH") < 0)
    return false;

  bool stMatch = false;
  if (req.indexOf(String("ST: ") + SSDP_ST) >= 0) stMatch = true;
  if (req.indexOf("ST: ssdp:all") >= 0) stMatch = true;

  if (!stMatch) return false;
  send_ssdp_blind();
  return true;
}

void send_ssdp_blind() {
  WiFiUDP udp;
  IPAddress bcast(SSDP_MCAST_ADDR);  // or your subnet’s broadcast
  udp.beginPacket(bcast, SSDP_PORT);

  IPAddress ip = WiFi.localIP();
  String location = "http://" + ip.toString() + ":80/" + String(SENSOR_TYPE) + ".xml";


  char macStr[20];
  sprintf(macStr, "%012llX", ESP.getEfuseMac());  // uppercase hex, 12 digits

  String resp =
    "NOTIFY * HTTP/1.1\r\n"
    "CACHE-CONTROL: max-age=60\r\n"
    "EXT:\r\n"
    "ST: "
    + String(SSDP_ST) + "\r\n" + "USN: " + String(SENSOR) + "-" + macStr + "\r\n" + "LOCATION: " + location + "\r\n"
                                                                                                              "SERVER: ESP32/1.0 UPnP/1.1 "
    + String(SENSOR) + "/1.0\r\n"
                       "\r\n";

  // Serial.println(resp);
  udp.print(resp);  // or some JSON payload
  udp.endPacket();
}

// -----------------------------------------------------------------------------
// Telnet Handling
// -----------------------------------------------------------------------------

/**
 * @brief Accepts new Telnet clients up to @ref MAX_SRV_CLIENTS.
 *
 * Old/disconnected sockets are automatically reused.
 */
void acceptNewClients() {
  if (!server.hasClient()) return;

  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (!serverClients[i] || !serverClients[i].connected()) {
      if (serverClients[i]) serverClients[i].stop();
      serverClients[i] = server.available();
      Serial.print("Client server: ");
      Serial.println(i);
      return;
    }
  }

  WiFiClient tmp = server.available();
  if (tmp) tmp.stop();  // reject extra client

  Serial.print("error server:");
}

/**
 * @brief Forwards Telnet input into Serial1 (UART bridge mode only).
 * @param activeFlag Set to true if data received
 */
void handleTelnetToSerial(bool &activeFlag) {
  static int state = 0;

#if SENSOR_V == SENSOR_TRANSPONDER
  // Listen for USB serial command: 'v'
  if (SerialStat == true) {
    while (Serial.available() > 0) {
      char ch = Serial.read();
      if (SIMULATE == true) {
        handle_data(ch);
      } else {
        Serial1.print((char)ch);
      }
    }
  }
#endif

  if (SerialStat == false) {
    for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i] && serverClients[i].connected()) {
        while (serverClients[i].available()) {
          char ch = serverClients[i].read();
          activeFlag = true;

#if SENSOR_V == SENSOR_TRANSPONDER
          if (SIMULATE == true) {
            handle_data(ch);
          } else {
            Serial1.print((char)ch);
          }

#elif SENSOR_V == SENSOR_IMU
          if (SIMULATE == true) {
            imu_handle_data(ch);
          } else {
            Serial1.print((char)ch);
          }
#elif SENSOR_V == SENSOR_RADAR
// ..
#elif SENSOR_V == SENSOR_AIRSPEED
//..
#elif SENSOR_V == SENSOR_ALTIMETER
//          altitude_handle_data(ch);
#endif
        }
      }
    }
  }
}

/**
 * @brief Forwards UART Serial1 data to all connected Telnet clients.
 */
void handleSerialToTelnet() {
  if (Serial1.available()) {
    size_t len = Serial1.available();
    uint8_t sbuf[256];
    if (len > sizeof(sbuf)) len = sizeof(sbuf);

    size_t n = Serial1.readBytes(sbuf, len);
    sbuf[n] = '\0';  // ✅ terminate

    for (int i = 0; i < MAX_SRV_CLIENTS; i++)
      if (serverClients[i] && serverClients[i].connected())
        serverClients[i].write(sbuf, len);
  }
}

// -----------------------------------------------------------------------------
// USB Serial CLI (WiFi config commands)
// -----------------------------------------------------------------------------

/**
 * @brief Prints available USB-serial commands to the user.
 */
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  help               - show help");
  Serial.println("  showwifi           - show stored Wi-Fi credentials");
  Serial.println("  wifi <ssid> <pass> - store credentials and reboot");
  Serial.println("  clearwifi          - erase stored Wi-Fi");
  Serial.println("  sim <true/false>   - set operation mode");
  Serial.println("  showsim            - show operation mode");
  Serial.println("  reboot             - reboot");
  Serial.println("  boot               - boot");
}

/**
 * @brief Parses and executes a CLI command entered over USB serial.
 * @param line Full input command
 */
bool processSerialCommand(const String &line) {
  String cmd = line;
  cmd.trim();
  if (cmd.length() == 0) return false;

  //  Serial.print(cmd);

  if (cmd.equalsIgnoreCase("help")) {
    printHelp();
    return false;
  }

  if (cmd.equalsIgnoreCase("reboot")) {
    delay(100);
    ESP.restart();
    return false;
  }

  if (cmd.equalsIgnoreCase("z=?")) {
    SerialStat = true;
    Serial.println(SENSOR);
    return true;
  }

  if (cmd.equalsIgnoreCase("boot")) {
    Serial.println("Now booting...");
    return true;
  }

  if (cmd.equalsIgnoreCase("showwifi")) {
    if (hasStoredWifi) {
      Serial.printf("SSID: %s\n", storedSsid.c_str());
      Serial.printf("PASS: %s\n", storedPass.c_str());
    } else {
      Serial.println("No stored Wi-Fi.");
    }
    return false;
  }

  if (cmd.equalsIgnoreCase("clearwifi")) {
    clearCredentials();
    return false;
  }

  if (cmd.startsWith("showsim")) {
    Serial.println("Simulation mode = ");
    Serial.println(SIMULATE);
    return false;
  }

  if (cmd.startsWith("sim ")) {
    bool s1 = cmd.indexOf(' ');
    SIMULATE = s1;
    Serial.println("Simulation mode is now set to = ");
    Serial.println(SIMULATE);
    return false;
  }

  if (cmd.startsWith("wifi ")) {
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    if (s2 < 0) {
      Serial.println("Usage: wifi <ssid> <pass>");
      return false;
    }
    String ssid = cmd.substring(s1 + 1, s2);
    String pass = cmd.substring(s2 + 1);
    //    ssid.replace('_', ' ');
    pass.replace('_', ' ');

    saveCredentials(ssid, pass);
    Serial.println("Rebooting...");
    delay(500);
    ESP.restart();
    return false;
  }

  printHelp();
  Serial.println("Unknown command.");
  return false;
}

/**
 * @brief Collects characters from USB serial and builds command lines.
 *
 * Called every loop iteration.
 */
bool handleSerialConfig() {
  bool ret = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      ret = processSerialCommand(serialLine);
      serialLine = "";
      break;
    } else if (c != '\r') {
      serialLine += c;
    }
  }
  return ret;
}

/**
 * @brief Main loop.  
 *
 * Performs:
 *   - Wi-Fi reconnection watchdog  
 *   - USB serial command handling  
 *   - SSDP discovery responder  
 *   - Telnet server operations  
 *   - Optional IMU simulation (if SIMULATE defined)
 */
void loop() {
  int tx = millis();
  static bool lo = false;

  static int sleep = 0;
  bool hasclient = false;

  if (SerialStat == false) {
    if (wifiMulti.run() != WL_CONNECTED) {
      Serial.println("WiFi lost!");
      delay(50);
      //    esp_task_wdt_reset();
      return;
    }

    // If there are client connected, then keep sending SSDP messages.
    // The reason for this is tha tthe iPhone and iPad does not send SSDP requests, but can receive...
    for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i] && serverClients[i].connected()) {
        hasclient = true;
      }
    }

    if (!hasclient) {
      static int td = 0;
      if (tx - td > 2500) {
        td = tx;
        send_ssdp_blind();
      }
    }

  handleSsdpReceive();
  acceptNewClients();

  }

#if SENSOR_V == SENSOR_RADAR
  loopRADAR();
/*  if (SIMULATE == false) {
    handleTelnetToSerial(active);
    handleSerialToTelnet();
  } else {
    loopRADAR();
  }
  */
#elif SENSOR_V == SENSOR_TRANSPONDER
  if (SIMULATE == false) {
    handleTelnetToSerial(active);
    handleSerialToTelnet();
  } else {
    loopTRANSPONDER();
    handleTelnetToSerial(active);
    handleSerialToTelnet();
    delay(100);
  }
#elif SENSOR_V == SENSOR_IMU
  if (SIMULATE == false) {
    handleTelnetToSerial(active);
    handleSerialToTelnet();
  } else {
    handleTelnetToSerial(active);
    imu_loop();
  }
#elif SENSOR_V == SENSOR_ALTIMETER
  handleTelnetToSerial(active);
  altimeter_loop();
#elif SENSOR_V == SENSOR_AIRSPEED
  handleTelnetToSerial(active);
  airspeed_loop();
#endif

#ifdef USE_FastLED
  static int flash = 0;
  if (tx - flash > 500) {
    flash = tx;

    if (lo == true) {
      //  leds[0] = CRGB::Red; FastLED.show(); delay(500);
      leds[0] = CRGB::Green;
      FastLED.show();
    } else {
      //  leds[0] = CRGB::Blue; FastLED.show(); delay(500);
      leds[0] = 0;
      FastLED.show();
    }
    lo = !lo;
  }
#endif

#if EXTRA_SENSOR_V == SENSOR_ANGLE
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
#endif
  //  esp_task_wdt_reset();
}