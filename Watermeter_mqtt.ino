#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/pcnt.h>
#include <Preferences.h>

// ==== CONFIGURATION ====

// Wi-Fi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// MQTT broker
const char* mqtt_server = "192.168.1.254";
const int   mqtt_port = 1883;
const char* mqtt_user = "mqtt_user";
const char* mqtt_pass = "mqtt_pass";
const char* device_id = "watermeter_01";

// GPIOs
#define PULSE_INPUT_GPIO    5
#define BATTERY_ADC_PIN     0
#define BOOT_PIN            9
#define REFERENCE_PIN       4

// Voltage Divider
#define R1 100000.0
#define R2 100000.0

// PCNT
#define PCNT_UNIT           PCNT_UNIT_0
#define PCNT_H_LIM_VAL      0x7FFF
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    1
#define PCNT_THRESH0_VAL   -1

#define MAX_PULSE_COUNT     100000

// MQTT Topics
#define TOPIC_BATT      "watermeter/status/battery"
#define TOPIC_COUNT     "watermeter/status/count"
#define TOPIC_VOLT      "watermeter/status/voltage"

WiFiClient espClient;
PubSubClient client(espClient);
Preferences prefs;

volatile uint32_t total_count = 0;
volatile unsigned long last_pulse_time = 0;

static void setup_pcnt() {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = PULSE_INPUT_GPIO,
        .ctrl_gpio_num  = REFERENCE_PIN,
        .channel        = PCNT_CHANNEL_0,
        .unit           = PCNT_UNIT,
        .pos_mode       = PCNT_COUNT_DIS,
        .neg_mode       = PCNT_COUNT_INC,
        .lctrl_mode     = PCNT_MODE_KEEP,
        .hctrl_mode     = PCNT_MODE_KEEP,
        .counter_h_lim  = PCNT_H_LIM_VAL,
        .counter_l_lim  = PCNT_L_LIM_VAL
    };
    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);
    pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_1);
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

static int16_t pcnt_get() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count);
    pcnt_counter_clear(PCNT_UNIT);
    return count;
}

float read_battery_voltage() {
    int adc_value = analogRead(BATTERY_ADC_PIN);
    float voltage = (adc_value / 4095.0) * 3.3;
    return voltage * ((R1 + R2) / R2);
}

void setup_wifi() {
    delay(10);
    Serial.printf("Connecting to %s\n", ssid);
    WiFi.begin(ssid, password);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries++ < 40) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connect failed!");
    }
}

void reconnect_mqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(device_id, mqtt_user, mqtt_pass)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(PULSE_INPUT_GPIO, INPUT);
    pinMode(BOOT_PIN, INPUT_PULLUP);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    prefs.begin("pulse_counter", false);
    total_count = prefs.getUInt("count", 0);

    setup_pcnt();
    setup_wifi();

    client.setServer(mqtt_server, mqtt_port);

    Serial.printf("Restored total count: %lu\n", total_count);
    last_pulse_time = millis();
}

void loop() {
    static unsigned long last_report = 0;
    unsigned long now = millis();

    if (!client.connected()) {
        reconnect_mqtt();
    }
    client.loop();

    if (now - last_report >= 3000) {
        float voltage = read_battery_voltage();
        uint8_t battery_pct = min((uint8_t)((voltage - 3.0) / (4.1 - 3.0) * 100), (uint8_t)100);
        int16_t delta = pcnt_get();
        total_count += delta;
        prefs.putUInt("count", total_count);

        Serial.printf("Battery: %.2fV (%u%%), Total: %lu\n", voltage, battery_pct, total_count);

        client.publish(TOPIC_BATT, String(battery_pct).c_str(), true);
        client.publish(TOPIC_VOLT, String(voltage, 2).c_str(), true);
        client.publish(TOPIC_COUNT, String(total_count).c_str(), true);

        last_report = now;
    }

    // Optional: Button long press to reset counter
    if (digitalRead(BOOT_PIN) == LOW) {
        delay(50);
        unsigned long t0 = millis();
        while (digitalRead(BOOT_PIN) == LOW) {
            if (millis() - t0 > 3000) {
                Serial.println("Resetting counter to 0...");
                total_count = 0;
                prefs.putUInt("count", total_count);
                break;
            }
            delay(50);
        }
    }

    delay(1000);
}

