#define LED_ON 1
#define LED_OFF 0

// Builtin LEDs
#define BUILTIN_LED_RED 0
#define BUILTIN_LED_BLUE 2

// RGB LED
#define GPIO_LED_GREEN 12
#define GPIO_LED_BLUE 13
#define GPIO_LED_RED 14

// Light sensor
#define GPIO_SENSOR_LOG_LIGHT A0

#include "colors.h"

// Temperature/humidity sensor
#include "Adafruit_Si7021.h"

// Gas sensor
#include "Adafruit_SGP30.h"

// Wifi
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
const char* WifiSsid = "xxx";
const char* WifiPass = "xxx";
const char* hostname = "lupo-esp";
#define SERVER_PORT 80
WiFiServer server(SERVER_PORT);

// sprintf target
char stringBuffer128[128];

typedef struct {
    float temperature; // °C
    float humidity;    // %
} TempHumMeasurement;

typedef struct {
    uint16_t eco2; // CO₂ equivalent, ppb
    uint16_t tvoc; // Total volatile organic compounds, ppb
} GasMeasurement;

Adafruit_Si7021 tempHumSensor;
Adafruit_SGP30 gasSensor;



void errorFlashBuiltinLed();
void builtinLedRed(bool);

void setupPinModes();
void setupSerial();
void errorFlashBuiltinLed();
void setupTempHumSensor();
void setupGasSensor();
void setupWifi();
void connectWifi();
void setupMdns();
void setupServer();

int measureBrightness();
TempHumMeasurement measureTempHum();
void calibrateGasTempHum(TempHumMeasurement);
GasMeasurement measureGas();
int mapDoubleInt(double, double, double, int, int);
void setLedHsv(hsvColor);
void loopCsv();
void loopTcp();

void builtinLedRed(bool state) {
    digitalWrite(BUILTIN_LED_RED, state == LED_ON ? LOW : HIGH);
}

void builtinLedBlue(bool state) {
    digitalWrite(BUILTIN_LED_BLUE, state == LED_ON ? LOW : HIGH);
}

void setupPinModes() {
    Serial.println("Setting pin modes");
    pinMode(BUILTIN_LED_RED, OUTPUT);
    pinMode(GPIO_LED_RED, OUTPUT);
    pinMode(GPIO_LED_GREEN, OUTPUT);
    pinMode(GPIO_LED_BLUE, OUTPUT);
    digitalWrite(GPIO_LED_RED, HIGH);
    digitalWrite(GPIO_LED_GREEN, HIGH);
    digitalWrite(GPIO_LED_BLUE, HIGH);
}

void setupSerial() {
    Serial.begin(115200);
    while(!Serial) { delay(10); }
    Serial.println(); // Separate from the initial rubbish sent at low baud rate
    Serial.println("Serial communication set up");
}

void errorFlashBuiltinLed() {
    int state = LED_ON;
    while (true) {
        delay(250);
        state = state == LED_ON ? LED_OFF : LED_ON;
        builtinLedRed(state);
    }
}

void setupTempHumSensor() {
    tempHumSensor = Adafruit_Si7021();
    if (!tempHumSensor.begin()) {
        Serial.println("Did not find temperature/humidity sensor!");
        errorFlashBuiltinLed();
    }
    Serial.print("Identifying temperature/humidity sensor ");
    switch(tempHumSensor.getModel()) {
        case SI_Engineering_Samples:
            Serial.print("SI engineering samples");
            break;
        case SI_7013:
            Serial.print("Si7013");
            break;
        case SI_7020:
            Serial.print("Si7020");
            break;
        case SI_7021:
            Serial.print("Si7021");
            break;
        case SI_UNKNOWN:
        default:
            Serial.print("UnknowMsn");
    }
    Serial.println();
}

void setupGasSensor() {
    if (!gasSensor.begin()){
        Serial.println("Did not find SGP30 gas sensor!");
        errorFlashBuiltinLed();
    }
    sprintf(stringBuffer128,
            "Found SGP30 sensor, serial number %x%x%x",
            gasSensor.serialnumber[0],
            gasSensor.serialnumber[1],
            gasSensor.serialnumber[2]);
    Serial.println(stringBuffer128);

    uint16_t eco2Base = 0x8e4c;
    uint16_t tvocBase = 0x8e49;
    gasSensor.setIAQBaseline(eco2Base, tvocBase);
    sprintf(stringBuffer128, "Baseline values set! eCO2: %x, TVOC: %x", eco2Base, tvocBase);
    Serial.println(stringBuffer128);
}

void setupWifi() {
    Serial.print("Setting up Wifi...");
    Serial.println();
    WiFi.hostname(hostname);
    WiFi.mode(WIFI_STA);
    connectWifi();
}

void connectWifi() {
    builtinLedBlue(LED_ON);
    WiFi.begin(WifiSsid, WifiPass);
    int dotsWritten = 0;
    while(!WiFi.isConnected()) {
        delay(250);
        Serial.print(".");
        ++dotsWritten;
        if(dotsWritten % 80 == 0) { Serial.println(); }
    }
    sprintf(stringBuffer128, "connected as %s", WiFi.localIP().toString().c_str());
    Serial.println(stringBuffer128);
    builtinLedBlue(LED_OFF);
}

void setupMdns() {
    if(!MDNS.begin(hostname)) {
        Serial.println("Error setting up mDNS responder");
        errorFlashBuiltinLed();
    }
    sprintf(stringBuffer128, "mDNS responder started: %s.local", hostname);
    Serial.println(stringBuffer128);
    MDNS.addService("esp", "tcp", 80);
}

void setupServer() {
    sprintf(stringBuffer128, "Starting server on port %d\n", SERVER_PORT);
    Serial.print(stringBuffer128);
    server.begin();
}

void setup() {
    setupPinModes();
    builtinLedRed(LED_ON);
    setupSerial();
    setupTempHumSensor();
    setupGasSensor();
    setupWifi();
    setupMdns();
    setupServer();
    builtinLedRed(LED_OFF);
}

int measureBrightness() {
    return analogRead(GPIO_SENSOR_LOG_LIGHT);
}

TempHumMeasurement measureTempHum() {
    return { .temperature = tempHumSensor.readHumidity()
           , .humidity = tempHumSensor.readTemperature() };
}

void calibrateGasTempHum(TempHumMeasurement tempHum) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((tempHum.humidity / 100.0f) * 6.112f * exp((17.62f * tempHum.temperature) / (243.12f + tempHum.temperature)) / (273.15f + tempHum.temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    gasSensor.setHumidity(absoluteHumidityScaled);
}

GasMeasurement measureGas() {
    if(!gasSensor.IAQmeasure()) {
        Serial.println("# Gas measurement failed");
    }

    return { .eco2 = gasSensor.eCO2
           , .tvoc = gasSensor.TVOC };
}

// Lol C. map from long to long won’t work for doubles.
int mapDoubleInt(double x, double fromLo, double fromHi, int toLo, int toHi) {
    double fromSpan = fromHi - fromLo;
    int toSpan = toHi - toLo;
    return (x - fromLo) * (double) toSpan / fromSpan + (double) toLo;
}

void setLedHsv(hsvColor hsv) {
    rgbColor rgb = hsv2rgb(hsv);

    int r = mapDoubleInt(rgb.r, 0.0, 1.0, 0, 1023);
    int g = mapDoubleInt(rgb.g, 0.0, 1.0, 0, 1023);
    int b = mapDoubleInt(rgb.b, 0.0, 1.0, 0, 1023);

    analogWrite(GPIO_LED_RED, r);
    analogWrite(GPIO_LED_GREEN, g);
    analogWrite(GPIO_LED_BLUE, b);
}

void loopCsv() {
    int brightness = measureBrightness();
    TempHumMeasurement th = measureTempHum();
    calibrateGasTempHum(th);
    GasMeasurement gas = measureGas();

    Serial.print(brightness);
    Serial.print(th.humidity, 2);
    Serial.print(th.temperature, 2);
    Serial.print(gas.eco2);
    Serial.print(gas.tvoc);
    Serial.println();
}

// LED state
double hue = 0;
double saturation = 1;
double lightness = 0;
void updateLed() { setLedHsv({hue, saturation, lightness}); }

// Measurement timing/caches/gauges
unsigned long nowMs;
int brightnessMeasurement;
TempHumMeasurement tempHumMeasurement;
GasMeasurement gasMeasurement;

const unsigned long gasHumidityGaugeIntervalMs = 5*1000;
unsigned long lastGasHumidityAdjustmentMs = 0;

const unsigned long gasBaselineCalibrationReadoutIntervalMs = 1*60*1000;
unsigned long lastGasBaselineCalibrationMs = 0;
// Will be modified by the baseline calibration function
uint16_t eco2Base = 0xff;
uint16_t tvocBase = 0xff;

const unsigned long mdnsUpdateIntervalMs = 1*60*1000;
unsigned long lastMdnsUpdateMs = 0;

void loop() {
    if(!WiFi.isConnected()) {
        Serial.println("Wifi unavailable/connection lost");
        connectWifi();
        return;
    }
    nowMs = millis();
    if(nowMs > lastMdnsUpdateMs + mdnsUpdateIntervalMs) {
        lastMdnsUpdateMs = nowMs;
        MDNS.update();
    }
    WiFiClient client = server.available();
    if(!client) {
        lightness = constrain(lightness - 0.01, 0, 1);
        updateLed();
        delay(5);
        return;
    }
    lightness = 1;
    hue = fmod(hue + 137.5, 360); // golden angle ensures lots of different colors
    updateLed();

    brightnessMeasurement = measureBrightness();
    tempHumMeasurement = measureTempHum();
    unsigned long lastGasHumidityCalibrationAgoMs = nowMs - lastGasHumidityAdjustmentMs;
    if(lastGasHumidityCalibrationAgoMs > gasHumidityGaugeIntervalMs) {
        lastGasHumidityAdjustmentMs = nowMs;
        calibrateGasTempHum(tempHumMeasurement);
    }
    unsigned long lastGasBaselineCalibrationAgoMs = nowMs - lastGasBaselineCalibrationMs;
    if(lastGasBaselineCalibrationAgoMs > gasBaselineCalibrationReadoutIntervalMs) {
        lastGasBaselineCalibrationMs = nowMs;
        if (!gasSensor.getIAQBaseline(&eco2Base, &tvocBase)) {
            Serial.println("Failed to get baseline readings");
            errorFlashBuiltinLed();
        }
    }
    gasMeasurement = measureGas();

    Serial.print("[New client");
    if(client.connected()) {

        {
            client.print("HTTP/1.1 200 OK\r\n");
            client.print("Content-Type: text/plain\r\n");
            client.print("\r\n");

            client.print("##########################################\n");
            client.print("# ESP environment sensor Prometheus feed #\n");
            client.print("##########################################\n");

            client.print("\n");
        }

        {
            sprintf(stringBuffer128, "# MAC:  %s\n", WiFi.macAddress().c_str());
            client.print(stringBuffer128);
            sprintf(stringBuffer128, "# IP:   %s\n", WiFi.localIP().toString().c_str());
            client.print(stringBuffer128);
            sprintf(stringBuffer128, "# mDNS: %s.local\n", hostname);
            client.print(stringBuffer128);
        }

        {
            client.print("\n# Analog brightness sensor\n");
            sprintf(stringBuffer128, "brightness %d\n", brightnessMeasurement);
            client.print(stringBuffer128);
        }

        {
            client.print("\n# Temperature/humidity sensor\n");
            sprintf(stringBuffer128, "temperature %.2f\n", tempHumMeasurement.temperature);
            client.print(stringBuffer128);
            sprintf(stringBuffer128, "humidity    %.2f\n", tempHumMeasurement.humidity);
            client.print(stringBuffer128);
        }

        {
            client.print("\n# Gas sensor\n");
            sprintf(stringBuffer128, "eco2 %d\n", gasMeasurement.eco2);
            client.print(stringBuffer128);
            sprintf(stringBuffer128, "tvoc %d\n", gasMeasurement.tvoc);
            client.print(stringBuffer128);
        }

        {
            client.print("#    Gas calibration values\n");
            sprintf(stringBuffer128, "#    <sensor>.setIAQBaseline(0x%x, 0x%x);\n", eco2Base, tvocBase);
            client.print(stringBuffer128);
            sprintf(stringBuffer128,
                "#    Last absolute humidity calibration: %.2f s ago (runs every %.2f s)\n",
                (double) lastGasHumidityCalibrationAgoMs / 1000,
                (double) gasHumidityGaugeIntervalMs / 1000
                );
            client.print(stringBuffer128);
            sprintf(stringBuffer128,
                "#    Last baseline calibration output: %.2f s ago (runs every %.2f s)\n",
                (double) lastGasBaselineCalibrationAgoMs / 1000,
                (double) gasBaselineCalibrationReadoutIntervalMs / 1000
                );
            client.print(stringBuffer128);

            sprintf(stringBuffer128,
                "gasHumidityCalibration %.3f\ngasBaselineCalibration %.3f\n",
                (double) lastGasHumidityCalibrationAgoMs/gasHumidityGaugeIntervalMs,
                (double) lastGasBaselineCalibrationAgoMs/gasBaselineCalibrationReadoutIntervalMs
                );
            client.print(stringBuffer128);

        }

        Serial.println(", done]");
    }
}
