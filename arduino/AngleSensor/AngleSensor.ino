/*
  XIAO RP2040 analog angle sensor

  Sampling rate: 5 Hz
  Low-pass cutoff: 1 Hz
*/

constexpr int SENSOR_PIN = A0;

constexpr float SAMPLE_RATE_HZ = 5.0f;
constexpr float SAMPLE_TIME_S  = 1.0f / SAMPLE_RATE_HZ;

constexpr float CUTOFF_HZ = 0.4f;

const float alpha =
    1.0f - expf(-2.0f * PI * CUTOFF_HZ * SAMPLE_TIME_S);

float filteredAngle = 0.0f;
bool filterInitialized = false;

constexpr unsigned long SAMPLE_INTERVAL_MS = 200;
unsigned long previousSampleTime = 0;

void setup()
{
    Serial.begin(115200);

    analogReadResolution(12);

    previousSampleTime = millis();
}

void loop()
{
    unsigned long now = millis();

    if (now - previousSampleTime >= SAMPLE_INTERVAL_MS)
    {
        previousSampleTime += SAMPLE_INTERVAL_MS;

        int sensorValue = analogRead(SENSOR_PIN);

        // 12-bit ADC: 0–4095
        float angle =
            sensorValue * (360.0f / 4095.0f) - 160.0f;

        if (!filterInitialized)
        {
            filteredAngle = angle;
            filterInitialized = true;
        }
        else
        {
            filteredAngle += alpha * (angle - filteredAngle);
        }
        
//        Serial.print("Raw:");
//        Serial.print(angle, 2);

//        Serial.print(",Filtered:");

        String output = "ANGLE, " + String(filteredAngle, 2);
        Serial.println(output);
    }
}