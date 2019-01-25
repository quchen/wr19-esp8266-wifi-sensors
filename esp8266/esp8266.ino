// Builtin LEDs
#define BUILTIN_LED_RED 0
#define BUILTIN_LED_BLUE 2

// RGB LED
#define GPIO_LED_GREEN 12
#define GPIO_LED_BLUE 13
#define GPIO_LED_RED 14

// I²C configuration
#define GPIO_I2C_SDA 4
#define GPIO_I2C_SCL 5
#define I2C_ADDR_TEMP_HUM 0x40
#define I2C_ADDR_GAS

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
const char* WifiSsid = "TNG";
const char* WifiPass = "Internet!bei!TNG";
const char* hostname = "esp8266-lupo";
WiFiServer server(80);

// sprintf target
char stringBuffer128[128];

typedef struct {
    float temperature; // °C
    float humidity;    // %
} tempHumMeasurement;

typedef struct {
    uint16_t eco2; // CO₂ equivalent, ppb
    uint16_t tvoc; // Total volatile organic compounds, ppb
} gasMeasurement;

Adafruit_Si7021 tempHumSensor = Adafruit_Si7021();
Adafruit_SGP30 gasSensor;

void errorFlashBuiltinLed() {
    int state = HIGH;
    while (true) {
        state = state == HIGH ? LOW : HIGH;
        digitalWrite(BUILTIN_LED_RED, state);
        delay(500);
    }
}

void setup() {
    Serial.begin(115200);
    while(!Serial) { delay(10); }
    Serial.println("Serial communication set up");

    Serial.println("Setting pin modes");
    pinMode(BUILTIN_LED_RED, OUTPUT);
    digitalWrite(BUILTIN_LED_RED, LOW); // Builtin seems to be on on LOW
    pinMode(GPIO_LED_RED, OUTPUT);
    pinMode(GPIO_LED_GREEN, OUTPUT);
    pinMode(GPIO_LED_BLUE, OUTPUT);
    digitalWrite(GPIO_LED_RED, HIGH);
    digitalWrite(GPIO_LED_GREEN, HIGH);
    digitalWrite(GPIO_LED_BLUE, HIGH);

    if (!tempHumSensor.begin()) {
        Serial.println("Did not find Si7021 sensor!");
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
            Serial.print("Unknown");
    }
    Serial.println();

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

    Serial.print("Setting up Wifi");
    WiFi.hostname(hostname);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WifiSsid, WifiPass);
    while(WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(".");
    }
    sprintf(stringBuffer128, "connected as %s", WiFi.localIP().toString().c_str());
    Serial.println(stringBuffer128);

    Serial.println("Starting server");
    server.begin();

    if(!MDNS.begin(hostname)) {
        Serial.println("Error setting up mDNS responder");
        errorFlashBuiltinLed();
    }
    sprintf(stringBuffer128, "mDNS responder started: %s.local", hostname);
    Serial.println(stringBuffer128);
    MDNS.addService("esp", "tcp", 80);

    digitalWrite(BUILTIN_LED_RED, HIGH);
}

int measureBrightness() {
    return analogRead(GPIO_SENSOR_LOG_LIGHT);
}

tempHumMeasurement measureTempHum() {
    return { .temperature = tempHumSensor.readHumidity()
           , .humidity = tempHumSensor.readTemperature() };
}

void calibrateGasTempHum(tempHumMeasurement tempHum) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((tempHum.humidity / 100.0f) * 6.112f * exp((17.62f * tempHum.temperature) / (243.12f + tempHum.temperature)) / (273.15f + tempHum.temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    gasSensor.setHumidity(absoluteHumidityScaled);
}

gasMeasurement measureGas() {
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

    int r = mapDoubleInt(rgb.r, 0.0, 1.0, 1023, 0);
    int g = mapDoubleInt(rgb.g, 0.0, 1.0, 1023, 0);
    int b = mapDoubleInt(rgb.b, 0.0, 1.0, 1023, 0);

    analogWrite(GPIO_LED_RED, r);
    analogWrite(GPIO_LED_GREEN, g);
    analogWrite(GPIO_LED_BLUE, b);

}

void loopCsv() {
    int brightness = measureBrightness();
    tempHumMeasurement th = measureTempHum();
    calibrateGasTempHum(th);
    gasMeasurement gas = measureGas();

    Serial.print(brightness);
    Serial.print(th.humidity, 2);
    Serial.print(th.temperature, 2);
    Serial.print(gas.eco2);
    Serial.print(gas.tvoc);
    Serial.println();
}

// LED state
double hue = 200;
double saturation = 1;
double lightness = 1;
void updateLed() { setLedHsv({hue, saturation, lightness}); }

// Only occasionally measure/do calibration work
unsigned long lastGasHumidityAdjustment = 0;
unsigned long lastGasCalibration = 0;
uint16_t eco2Base = 0xff;
uint16_t tvocBase = 0xff;

void loopTcp() {
    MDNS.update();
    WiFiClient client = server.available();
    if(!client) {
        lightness = lightness > 0 ? lightness - 0.01 : 0;
        updateLed();
        delay(5);
        return;
    }
    lightness = 0.5;
    hue = random(0, 360);
    updateLed();
    Serial.print("[New client");
    if(client.connected()) {

        client.print("HTTP/1.1 200 OK\r\n");
        client.print("Content-Type: text/plain\r\n");
        client.print("\r\n");

        client.print("##########################################\n");
        client.print("# ESP environment sensor Prometheus feed #\n");
        client.print("##########################################\n");

        client.print("\n");

        sprintf(stringBuffer128, "# MAC:  %s\n", WiFi.macAddress().c_str());
        client.print(stringBuffer128);
        sprintf(stringBuffer128, "# IP:   %s\n", WiFi.localIP().toString().c_str());
        client.print(stringBuffer128);
        sprintf(stringBuffer128, "# mDNS: %s.local\n", hostname);
        client.print(stringBuffer128);

        {
            int brightness = measureBrightness();
            client.print("\n# Analog brightness sensor\n");
            sprintf(stringBuffer128, "brightness %d\n", brightness);
            client.print(stringBuffer128);
        }

        tempHumMeasurement th;
        {
            th = measureTempHum();
            client.print("\n# Temperature/humidity sensor\n");
            sprintf(stringBuffer128, "temperature %.2f\n", th.temperature);
            client.print(stringBuffer128);
            sprintf(stringBuffer128, "humidity    %.2f\n", th.humidity);
            client.print(stringBuffer128);
        }

        unsigned long now = millis();
        {
            {
                unsigned long gaugeInterval = 60*1000;
                if(now - lastGasHumidityAdjustment > gaugeInterval) {
                    lastGasHumidityAdjustment = now;
                    calibrateGasTempHum(th);
                }
            }
            gasMeasurement gas = measureGas();
            client.print("\n# Gas sensor\n");
            sprintf(stringBuffer128, "eco2 %d\n", gas.eco2);
            client.print(stringBuffer128);
            sprintf(stringBuffer128, "tvoc %d\n", gas.tvoc);
            client.print(stringBuffer128);
        }

        {
            unsigned long calibrationReadoutInterval = 60*1000;
            if(now - lastGasCalibration > calibrationReadoutInterval) {
                lastGasCalibration = now;
                if (!gasSensor.getIAQBaseline(&eco2Base, &tvocBase)) {
                    Serial.println("Failed to get baseline readings");
                    errorFlashBuiltinLed();
                }
            }
            client.print("# Calibration values\n");
            sprintf(stringBuffer128, "# <sensor>.setIAQBaseline(0x%x, 0x%x);\n", eco2Base, tvocBase);
            client.print(stringBuffer128);
        }
        // gasSensor.setIAQBaseline(0x8E68, 0x8F41);

        Serial.println(", done]");
    }
}

void loop() {
    loopTcp();
}
