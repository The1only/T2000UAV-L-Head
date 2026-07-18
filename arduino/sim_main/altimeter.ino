/**
 * @file altimeter.cpp
 * @brief Altimeter support using BMP280 pressure sensor with QNH and relative altitude calibration.
 *
 * This module reads pressure and temperature from a BMP280 sensor over I2C,
 * applies low-pass filtering, converts pressure to altitude using a QNH-based
 * barometric formula, and transmits altimeter data over a TCP server channel.
 *
 * Features:
 * - Persistent QNH storage in EEPROM
 * - Startup and command-driven altitude calibration (relative altitude offset)
 * - Simple serial command parser for CAL / QNH= / QNH?
 *
 * Output format (CSV):
 *   Altimeter,<pressure_hPa>,<temperature_C>,<relative_alt_m>,<absolute_alt_m>\n
 *
 * Notes:
 * - Absolute altitude is computed relative to QNH (sea-level reference pressure).
 * - Relative altitude is computed using an offset captured during calibration.
 */

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <math.h>

/* ============================================================================
 * EEPROM configuration
 * ============================================================================
 */

/** Total EEPROM bytes reserved for this module */
#define EEPROM_SIZE 16

/** Address used for a magic marker indicating valid EEPROM contents */
#define EEPROM_MAGIC_ADDR 0

/** Address used to store the QNH float value */
#define EEPROM_QNH_ADDR   1

/** Magic value used to detect valid stored configuration */
#define EEPROM_MAGIC      0x42

/* ============================================================================
 * Barometric constants and defaults
 * ============================================================================
 */

/** Default QNH (hPa) used if no value is stored in EEPROM */
#define DEFAULT_QNH 1013.25f

/** Nominal sea-level pressure (hPa), used only by the alternative formula */
#define SEA_LEVEL_PRESSURE 1013.25f  // hPa

/* ============================================================================
 * Hardware configuration
 * ============================================================================
 */

/** I2C pins used for the BMP280 on this platform */
#define SDA_PIN 7
#define SCL_PIN 6

/** Alternative BMP280 I2C address (common is 0x76 or 0x77) */
#define BMP280_ADDRESS_ALT_LOCAL (0x76)

/* ============================================================================
 * Module state
 * ============================================================================
 */

/** Current QNH reference (hPa). Used in pressure→altitude conversion. */
float qnh = DEFAULT_QNH;

/**
 * Relative altitude offset (meters).
 * Captured during calibration and subtracted to produce relative altitude.
 */
float altitude_offset = 0.0f;

/** BMP280 driver object (I2C interface) */
Adafruit_BMP280 bmp;

/** Convenience sensor pointers from Adafruit unified sensor API */
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
Adafruit_Sensor *bmp_temp     = bmp.getTemperatureSensor();

/** Low-pass filtered pressure (hPa) */
static float filteredPressValue = 0.0f;

/** Low-pass filtered temperature (°C) */
static float filteredTempValue  = 0.0f;

/* ============================================================================
 * Forward declarations
 * ============================================================================
 */

static void loadQNH();
static void saveQNH();
static void calibrateAltitude(float pressure_hPa, float temp_C);
static float getRelativeAltitude(float pressure_hPa, float temp_C);
static float pressureToAltitudeNEW(float pressure_hPa, float temperature_C);
static void altitude_handle_data(char dta);

/* ============================================================================
 * Setup / initialization
 * ============================================================================
 */

/**
 * @brief Initialize BMP280 sensor and load persistent settings.
 *
 * - Loads QNH from EEPROM if available
 * - Initializes I2C
 * - Configures BMP280 sampling parameters
 * - Verifies sensor presence (restarts device on failure)
 *
 * @note This function assumes Serial has already been initialized by the caller.
 */
void altimeter_setup()
{
  Serial.println(F("BMP280 Sensor event test..."));

  // Load previously stored QNH (or default)
  loadQNH();

  // Initialize I2C on configured pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Configure BMP280 sampling (datasheet-like defaults + filtering)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Operating mode
                  Adafruit_BMP280::SAMPLING_X2,      // Temperature oversampling
                  Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,       // IIR filter
                  Adafruit_BMP280::STANDBY_MS_500);  // Standby time

  // Optional: print sensor details to serial for debug
  bmp_temp->printSensorDetails();

  // Initialize the sensor; uses external address/chipid macros from your project
  unsigned status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // Alternatively:
  // unsigned status = bmp.begin();

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP180 or BMP085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP280\n");
    Serial.print("        ID of 0x60 represents a BME280\n");
    Serial.print("        ID of 0x61 represents a BME680\n");
    delay(1000);

    // Hard restart to recover (ESP platform)
//    ESP.restart();
  }
}

/* ============================================================================
 * EEPROM persistence
 * ============================================================================
 */

/**
 * @brief Load QNH from EEPROM if present; otherwise use DEFAULT_QNH.
 *
 * EEPROM format:
 * - EEPROM_MAGIC_ADDR contains EEPROM_MAGIC if valid
 * - EEPROM_QNH_ADDR contains a float (QNH in hPa)
 */
static void loadQNH()
{
  EEPROM.begin(EEPROM_SIZE);

  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_QNH_ADDR, qnh);
    Serial.print("Loaded QNH from EEPROM: ");
    Serial.println(qnh, 2);
  } else {
    qnh = DEFAULT_QNH;
    Serial.println("No stored QNH, using default 1013.25");
  }
}

/**
 * @brief Save current QNH to EEPROM.
 *
 * Writes:
 * - magic byte
 * - float QNH
 */
static void saveQNH()
{
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_QNH_ADDR, qnh);
  EEPROM.commit();

  Serial.print("Saved QNH to EEPROM: ");
  Serial.println(qnh, 2);
}

/* ============================================================================
 * Altitude conversion and calibration
 * ============================================================================
 */

/**
 * @brief Capture the current altitude as the relative altitude reference.
 *
 * After calling this, getRelativeAltitude() will return 0 at the current
 * pressure/temperature (assuming stable conditions).
 */
static void calibrateAltitude(float pressure_hPa, float temp_C)
{
  altitude_offset = pressureToAltitudeNEW(pressure_hPa, temp_C);
}

/**
 * @brief Convert current pressure/temperature to relative altitude (meters).
 *
 * Relative altitude is computed as:
 *   absolute_altitude - altitude_offset
 */
static float getRelativeAltitude(float pressure_hPa, float temp_C)
{
  return pressureToAltitudeNEW(pressure_hPa, temp_C) - altitude_offset;
}

/**
 * @brief Convert pressure (hPa) to altitude (meters) using QNH reference.
 *
 * Uses a standard barometric formula approximation:
 *   h = 44330 * (1 - (P / QNH)^0.1903)
 *
 * @param pressure_hPa Current pressure in hPa
 * @param temperature_C Current temperature in °C (currently unused)
 * @return Altitude above mean sea level (meters) relative to QNH reference
 *
 * @note temperature_C is not used in this formula (kept for compatibility).
 */
static float pressureToAltitudeNEW(float pressure_hPa, float temperature_C)
{
  (void)temperature_C;
  return 44330.0f * (1.0f - pow(pressure_hPa / qnh, 0.1903f));
}

/*
 * Alternative formula (hypsometric equation) retained for reference.
 * This version uses temperature and logarithms:
 *
float pressureToAltitude(float pressure_hPa, float temperature_C) {
  float temperature_K = temperature_C + 273.15f;
  float altitude_m =
    (287.05f * temperature_K / 9.80665f) *
    log(SEA_LEVEL_PRESSURE / pressure_hPa);
  return altitude_m;
}
*/

/* ============================================================================
 * Command handling
 * ============================================================================
 */

/**
 * @brief Handle incoming control characters for altimeter commands.
 *
 * Commands are collected until '\n' and then executed.
 *
 * Supported commands:
 * - "CAL"      : capture current altitude as relative zero
 * - "QNH=xxxx" : set QNH (valid range 900..1100 hPa) and save to EEPROM
 * - "QNH?"     : print current QNH to serial
 *
 * @param dta Incoming character
 */
static void altitude_handle_data(char dta)
{
  static String cmd = "";
  cmd = cmd + dta;

  if (dta == '\n') {
    cmd.trim();
    Serial.println(cmd);

    if (cmd.equalsIgnoreCase("CAL")) {
      calibrateAltitude(filteredPressValue, filteredTempValue);
    }
    else if (cmd.startsWith("QNH=")) {
      float newQnh = cmd.substring(4).toFloat();
      if (newQnh > 900 && newQnh < 1100) {
        qnh = newQnh;
        saveQNH();
      } else {
        Serial.println("Invalid QNH value");
      }
    }
    else if (cmd.equalsIgnoreCase("QNH?")) {
      Serial.print("Current QNH = ");
      Serial.println(qnh, 2);
    }
    else {
      Serial.println("Unknown command");
    }

    cmd = "";
  }
}

/* ============================================================================
 * Main loop
 * ============================================================================
 */

/**
 * @brief Periodic altimeter update loop.
 *
 * - Reads pressure and temperature events from BMP280
 * - Applies low-pass filtering
 * - Performs an automatic calibration after ~10 seconds (200 * 200 ms)
 * - Computes absolute altitude and relative altitude
 * - Sends data over TCP to all connected clients
 *
 * Output (CSV):
 *   Altimeter,<pressure_hPa>,<temperature_C>,<relative_alt_m>,<absolute_alt_m>\n
 */
void altimeter_loop()
{
  static int altsim = 100;
  const float alpha = 0.050f;        // Low-pass filter smoothing factor
  static int calibrate = 0;          // Used to trigger one-time startup calibration

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  // Low-pass filtering (reduces sensor noise)
  filteredPressValue = alpha * pressure_event.pressure +
                       (1.0f - alpha) * filteredPressValue;

  filteredTempValue  = alpha * temp_event.temperature +
                       (1.0f - alpha) * filteredTempValue;

  // Perform one-time auto calibration after ~10 seconds
  if (calibrate == 200) {
    calibrateAltitude(filteredPressValue, filteredTempValue);
  }
  if (++calibrate >= 201) {
    calibrate = 201;
  }

  float altitude = pressureToAltitudeNEW(filteredPressValue, filteredTempValue);
  float relative = getRelativeAltitude(filteredPressValue, filteredTempValue);

  if (SIMULATE == true) { 
    altitude = altsim;
    altsim= altsim+10;
    if(altsim > 5000) altsim= 100;
  }

  // Format and transmit CSV message
  String s = String("ALTIMETER,") +
             String(filteredPressValue, 4) + "," +
             String(filteredTempValue,  4) + "," +
             String(relative,          4)  + "," +
             String(altitude,          4)  + "," +
             String(sensorValue,       4),
             String("\n");

  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      serverClients[i].write((const uint8_t*)s.c_str(), s.length());
    }
  }

  if(serverClients[0] == 0)
    Serial.print(s);

  delay(200);
}