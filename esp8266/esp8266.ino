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
WiFiServer server(80);

// sprintf target
char stringBuffer128[128];

typedef struct {
    float temperature; // °C
    float humidity;    // %
} tempHumMeasurement;

typedef struct {
    uint16_t eCo2; // CO₂ equivalent, ppb
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

    const char* hostname = "esp8266";

    Serial.print("Setting up Wifi");
    WiFi.hostname(hostname);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WifiSsid, WifiPass);
    while(WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    sprintf(stringBuffer128, "connected to %s", WiFi.localIP().toString().c_str());
    Serial.println(stringBuffer128);

    Serial.println("Starting server");
    server.begin();

    if(!MDNS.begin(hostname)) {
        Serial.println("Error setting up mDNS responder");
        errorFlashBuiltinLed();
    }
    // MDNS.addService("http", "tcp", 80);
    sprintf(stringBuffer128, "mDNS responder started: %s.local", hostname);
    Serial.println(stringBuffer128);

    digitalWrite(BUILTIN_LED_RED, HIGH);
}

int measureBrightness() {
    return analogRead(GPIO_SENSOR_LOG_LIGHT);
}

tempHumMeasurement measureTempHum() {
    return { .temperature = tempHumSensor.readHumidity()
           , .humidity = tempHumSensor.readTemperature() };
}

uint32_t absoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

void calibrateGas(tempHumMeasurement tempHum) {
    gasSensor.setHumidity(absoluteHumidity(tempHum.temperature, tempHum.humidity));
}

gasMeasurement measureGas() {
    if(!gasSensor.IAQmeasure()) {
        Serial.println("# Gas measurement failed");
    }

    return { .eCo2 = gasSensor.eCO2
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

void loopTcp() {
    MDNS.update();
    WiFiClient client = server.available();
    if(!client) {
        delay(100);
        return;
    }
    Serial.println("New client");
    if(client.connected()) {
        client.print("##########################################\n");
        client.print("# ESP environment sensor Prometheus feed #\n");
        client.print("##########################################\n");

        client.print("\n# IP: ");
        client.print(WiFi.localIP());
        client.print("\n");

        int brightness = measureBrightness();
        client.print("\n# Analog brightness sensor\n");
        client.print("brightness ");
        client.print(brightness);
        client.print(" # log au\n");

        tempHumMeasurement th = measureTempHum();
        client.print("\n# Temperature/humidity sensor\n");
        client.print("temperature ");
        client.print(th.temperature);
        client.print(" # deg C\n");
        client.print("humidity ");
        client.print(th.humidity);
        client.print(" # %\n");

        calibrateGas(th);
        gasMeasurement gas = measureGas();
        client.print("\n# Gas sensor\n");
        client.print("eco2 ");
        client.print(gas.eCo2);
        client.print(" # ppm\n");
        client.print("tvoc ");
        client.print(gas.tvoc);
        client.print(" # ppm\n");
    }
    Serial.println("Done with client");
}

void loop() {
    loopTcp();
}
