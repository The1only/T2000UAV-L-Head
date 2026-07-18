/**
 * @file radar.cpp
 * @brief Interface and parser for the NRA24 24 GHz millimeter-wave radar altimeter.
 *
 * The NRA24 is a compact K-band (24 GHz ISM) radar altimeter developed by
 * Hunan Nanoradar Science and Technology Co., Ltd.
 *
 * Features:
 * - ~2 cm altitude measurement accuracy
 * - Small size, low weight, high sensitivity
 * - Suitable for UAS, helicopters, and small aircraft
 *
 * This module:
 * - Sends initialization commands to the radar over UART
 * - Receives and parses binary measurement frames
 * - Supports a simulation mode for development and testing
 * - Transmits parsed radar data to connected TCP clients
 *
 * Example received frames (hex):
 *   AA AA 0C 07 01 B8 05 4E 2D 02 BE F9 55 55
 *   AA AA 0C 07 01 D8 05 4E 2D 02 BE 19 55 55
 */

#include <Arduino.h>

/* ============================================================================
 * USB identification (must be defined before USB stack includes)
 * ============================================================================
 */

#undef USB_MANUFACTURER
#undef USB_PRODUCT

#define USB_MANUFACTURER "9Tek"
#define USB_PRODUCT      "Radar_v1"
#define USB_SERIAL       "Radar001"   ///< Optional USB serial number

/* ============================================================================
 * Module state
 * ============================================================================
 */

/** Buffer holding incoming raw UART data */
String inputString = "";

/** Completed radar frame */
String outputString = "";

/** Indicates a complete radar frame has been received */
bool stringComplete = false;

/* ============================================================================
 * Radar command frames
 * ============================================================================
 */

/**
 * @brief Generic radar command frame.
 *
 * Frame structure:
 *   [0–1]  Header      (0xAA 0xAA)
 *   [2–3]  Command ID
 *   [4–10] Payload
 *   [11]   Checksum / reserved
 *   [12–13] Footer    (0x55 0x55)
 */
uint8_t command[] = {
  0xAA, 0xAA,
  0x00, 0x02,
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00,
  0x55, 0x55 
};

/** Radar status request command */
uint8_t commandStatus[] = {
  0xAA, 0xAA,
  0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00,
  0x55, 0x55
};

/**
 * @brief Simulated radar frame used when SIMULATE is enabled.
 *
 * Matches the expected NRA24 response layout.
 */
byte myData[] = {
  0xAA, 0xAA, 0x0C, 0x07,
  0x01, 0xA0, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x71,
  0x55, 0x55
};

/* ============================================================================
 * Setup and command handling
 * ============================================================================
 */

/**
 * @brief Initialize radar subsystem.
 *
 * - Reserves memory for input buffers
 * - Sends initial command to the radar
 *
 * @note Serial1 must already be initialized by the caller.
 */
void setupRADAR()
{
  inputString.reserve(200);
  sendCommand(command);
}

/**
 * @brief Compute payload checksum for a radar command.
 *
 * The checksum is the 8-bit sum of payload bytes [4..10].
 *
 * @param data Pointer to command frame buffer
 */
static void calculatePayloadChecksum(uint8_t *data)
{
  uint16_t sum = 0;
  for (int i = 4; i < 4 + 7; i++) {
    sum += data[i];
  }
  data[11] = (uint8_t)(sum & 0xFF);
}

/**
 * @brief Send a binary command frame to the radar.
 *
 * - Prints the frame in hex for debugging
 * - Transmits via Serial1
 *
 * @param cmd Pointer to command buffer
 */
static void sendCommand(uint8_t *cmd)
{
  char buff[10];

  for (int i = 0; i < 14; i++) {
    sprintf(buff, "%.2X ", cmd[i]);
    Serial.print(buff);
  }
  Serial.println(" <- I send...");

  if (SIMULATE == false) {
    Serial1.write(cmd, sizeof(command));
  }
}

/* ============================================================================
 * Main radar processing loop
 * ============================================================================
 */

/**
 * @brief Radar processing loop.
 *
 * - In simulation mode: generates synthetic radar frames
 * - In live mode: receives binary data from Serial1
 * - Detects frame boundaries using 0x55 0x55 footer
 * - Parses radar measurements
 * - Sends processed values to TCP clients
 *
 * Output format (CSV):
 *   <section>,<relative_vertical_velocity>,<range>\n
 */
void loopRADAR()
{
  static int pos = 0;
  char buff[50];
  static bool connected = false;

  if (SIMULATE == true) {
    delay(50);

    // Simulate changing vertical velocity and range
    myData[4 + 3] += 16;
    if (myData[4 + 3] == 1) myData[4 + 2]++;
    if (myData[4 + 2] > 40) myData[4 + 2] = 0;

    myData[4 + 6] += 4;
    if (myData[4 + 6] == 1) myData[4 + 5]++;
    if (myData[4 + 5] > 10) myData[4 + 5] = 0;

    outputString = String((const char *)myData);
    stringComplete = true;
  }

  /* ------------------------------------------------------------------------
   * Frame processing
   * ------------------------------------------------------------------------ */
  if (stringComplete) {

    // Validate header and message ID
    if (outputString[0] == 0xAA &&
        outputString[1] == 0xAA &&
        outputString[2] == 0x0C &&
        outputString[3] == 0x07)
    {
      connected = true;

      /**
       * Payload decoding (inferred from vendor protocol):
       *
       * Byte offsets relative to payload start (index 4):
       *  [0]   Section index
       *  [1]   Section angle (0.5° steps, offset -50°)
       *  [2–3] Relative vertical velocity (scaled)
       *  [5–6] Range / altitude (scaled)
       */

      char index = outputString[4 + 0];
      char tmp   = outputString[4 + 1];

      // Section angle (normalized)
      float section = (((float)tmp * 0.5f) - 50.0f) / 100.0f;

      // Relative vertical velocity
      float VrelH = ((float)outputString[4 + 2] * 256.0f +
                     (float)outputString[4 + 3]) * 0.006f;

      // Radar range / altitude
      float range = ((float)outputString[4 + 5] * 256.0f +
                     (float)outputString[4 + 6]) * 0.05f;

      sprintf(buff, "RADAR,%f,%f,%f\n", section, VrelH, range);

      for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (serverClients[i] && serverClients[i].connected()) {
          serverClients[i].write(buff, strlen(buff));
        }
      }
      if(serverClients[0]  == 0){
        Serial.print(buff);
      }
    }

    // Reset frame buffer
    outputString = "";
    stringComplete = false;
  }

  /* ------------------------------------------------------------------------
   * UART receive handling
   * ------------------------------------------------------------------------ */
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    inputString += inChar;

    // Detect footer sequence 0x55 0x55
    if (pos == 0 && inChar == 0x55) {
      pos = 1;
    } else if (pos == 1 && inChar == 0x55) {
      pos = 0;
      stringComplete = true;
      outputString = inputString;
      inputString = "";
    } else {
      pos = 0;
    }
  }
}