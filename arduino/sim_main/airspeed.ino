/**
 * @file airspeed.cpp
 * @brief Airspeed measurement and simulation using MS4525DO differential pressure sensor.
 *
 * This module reads a MS4525DO differential pressure sensor over I2C,
 * converts raw sensor values to differential pressure (Pa),
 * applies calibration, filtering, and deadband logic,
 * and computes indicated airspeed using the pitot equation.
 *
 * The module also supports a simulation mode for development and testing.
 */
#include <Wire.h>

/* ============================================================================
 * Configuration constants
 * ============================================================================
 */
#define MAX_SRV_CLIENTS 4
/** I2C address of the MS4525DO sensor */
#define MS4525_ADDR 0x28

/** Air density at sea level (ISA), kg/m^3 */
#define AIR_DENSITY 1.225f

/**
 * Sensor full-scale pressure range.
 * Adjust to match the exact MS4525 variant used.
 *
 * ±1 PSI -> 1.0f
 * ±2 PSI -> 2.0f
 * ±5 PSI -> 5.0f
 */
#define SENSOR_FULL_SCALE_PSI 1.0f

/** I2C pin configuration (ESP32-S3) */
#define sdaPin  4
#define clkPin  5

/**
 * Differential pressure deadband in Pascals.
 * Values below this are treated as zero airspeed.
 */
#define DP_DEADBAND_PA 2.5f

/* ============================================================================
 * Module state
 * ============================================================================
 */

/** Zero-offset calibration value in Pascals */
static float pressureOffsetPa = 0.0f;

/** Low-pass filtered differential pressure value (Pa) */
static float filteredSpeedValue = 0.0f;

/** Low-pass filter smoothing factor (0 = no update, 1 = no filtering) */
static const float alpha = 0.075f;

/* ============================================================================
 * MS4525DO low-level access
 * ============================================================================
 */

/**
 * @brief Read raw pressure and temperature data from the MS4525DO sensor.
 *
 * @param[out] rawP   Raw 14-bit pressure value
 * @param[out] rawT   Raw 11-bit temperature value
 * @param[out] status Sensor status bits (0 = OK)
 * @return true if read was successful, false otherwise
 */
static bool readMs4525(uint16_t &rawP, uint16_t &rawT, uint8_t &status)
{
    if (Wire.requestFrom(MS4525_ADDR, (uint8_t)4) != 4)
        return false;

    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();
    uint8_t b4 = Wire.read();

    status = (b1 >> 6) & 0x03;
    rawP   = ((uint16_t)(b1 & 0x3F) << 8) | b2;
    rawT   = ((uint16_t)b3 << 3) | (b4 >> 5);

    return true;
}

/**
 * @brief Convert raw MS4525 pressure counts to Pascals.
 *
 * The MS4525 outputs a 14-bit value spanning 10%–90% of full scale.
 *
 * @param rawP Raw pressure ADC value
 * @return Differential pressure in Pascals
 */
static float rawPressureToPa(uint16_t rawP)
{
    const float P_MIN  = 0.1f * 16384.0f;   // 10%
    const float P_MAX  = 0.9f * 16384.0f;   // 90%
    const float P_SPAN = P_MAX - P_MIN;
    const float P_MID  = P_MIN + (P_SPAN * 0.5f);

    // Signed distance from mid-scale
    float dpCounts = (float)rawP - P_MID;

    // Convert counts to PSI, then to Pascals
    float dpPsi = dpCounts / (P_SPAN * 0.5f) * SENSOR_FULL_SCALE_PSI;
    return dpPsi * 6894.76f;
}

/* ============================================================================
 * Calibration
 * ============================================================================
 */

/**
 * @brief Perform zero-pressure calibration.
 *
 * Assumes the pitot tube is stationary and exposed to still air.
 * Averages multiple samples to determine sensor offset.
 */
static void calibrateZero()
{
    const int samples = 200;
    float sum = 0.0f;
    int good = 0;

    for (int i = 0; i < samples; i++) {
        uint16_t rawP, rawT;
        uint8_t status;

        if (!readMs4525(rawP, rawT, status)) { delay(5); continue; }
        if (status != 0)                     { delay(5); continue; }

        float dpPa = rawPressureToPa(rawP);

        // Polarity correction (depends on tubing orientation)
        dpPa = -dpPa;

        sum += dpPa;
        good++;
        delay(5);
    }

    pressureOffsetPa = (good > 0) ? ((sum / good) + 7.5f) : 0.0f;

    Serial.print("Zero calibration: samples=");
    Serial.print(good);
    Serial.print(" offsetPa=");
    Serial.println(pressureOffsetPa, 2);
}

/* ============================================================================
 * Simulation support
 * ============================================================================
 */

/**
 * @brief Generate simulated differential pressure for testing.
 *
 * Produces a smooth sinusoidal airspeed profile with noise.
 *
 * @return Simulated differential pressure in Pascals
 */
float simulateDpPa()
{
    static uint32_t t0 = millis();
    float t = (millis() - t0) * 0.001f;

    float airspeed = 10.0f * (1.0f + sinf(0.2f * t));

    float dpPa = 0.5f * AIR_DENSITY * airspeed * airspeed;

    // Add small random noise
    dpPa += random(-20, 21) * 0.1f;

    return -dpPa;
}

/* ============================================================================
 * Public interface
 * ============================================================================
 */

/**
 * @brief Initialize the airspeed subsystem.
 *
 * Configures I2C, performs zero calibration,
 * and prepares the sensor for runtime operation.
 */
void airspeed_setup()
{
    Wire.begin(sdaPin, clkPin, 100000);

    Serial.println("MS4525DO starting...");
    Serial.println("Keep the pitot still for zero calibration...");
    delay(500);

    calibrateZero();
}

/**
 * @brief Main airspeed processing loop.
 *
 * Reads sensor or simulation data, applies filtering and calibration,
 * computes airspeed, and transmits results over TCP.
 */
void airspeed_loop()
{
    uint16_t rawP = 0, rawT = 0;
    uint8_t status = 0;
    float dpPa;

    if (SIMULATE == true) { 
        dpPa = simulateDpPa();
    }
    else{
        if (!readMs4525(rawP, rawT, status)) {
            Serial.println("I2C read failed");
            delay(200);
            return;
        }

        if (status != 0) {
            Serial.print("Status=");
            Serial.println(status);
            delay(200);
            return;
        }

        dpPa = rawPressureToPa(rawP);
    }

    // Polarity correction
    dpPa = -dpPa;

    // Low-pass filter
    filteredSpeedValue =
        alpha * dpPa + (1.0f - alpha) * filteredSpeedValue;

    // Apply zero-offset correction
    float correctedPa = filteredSpeedValue - pressureOffsetPa;

    // Compute airspeed with deadband
    float airspeed = 0.0f;
    if (correctedPa > DP_DEADBAND_PA) {
        airspeed = sqrtf((2.0f * correctedPa) / AIR_DENSITY);
    }
    if (correctedPa < -DP_DEADBAND_PA) {
        airspeed = sqrtf((2.0f * correctedPa) / AIR_DENSITY);
    }

    // Format and transmit CSV message
    String s =
        String("AIRSPEED,") +
        String(rawP) + "," +
        String(rawT) + "," +
        String(filteredSpeedValue, 4) + "," +
        String(pressureOffsetPa, 4) + "," +
        String(correctedPa, 2) + "," +
        String(airspeed, 2)  + "," +
        String(sensorValue, 4) + ",\n";


    for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (serverClients[i] && serverClients[i].connected()) {
            serverClients[i].write(
                (const uint8_t *)s.c_str(), s.length());
        }
    }
    if(serverClients[0]  == 0){
        Serial.print(s);
    }
    
    delay(200);
}