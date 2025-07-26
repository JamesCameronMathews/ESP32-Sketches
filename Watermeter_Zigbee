// Includes
#include <Arduino.h>
#include <driver/pcnt.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include "Zigbee.h"

// GPIO Definitions
#define PULSE_INPUT_GPIO    5     // Pulse sensor input
#define BATTERY_ADC_PIN     0     // Battery voltage measurement
#define BOOT_PIN            9     // Button for Zigbee reset
#define REFERENCE_PIN       4     // Control grnd voltage

// Voltage Divider Resistor Values
#define R1 100000.0  // Top resistor (Ohms)
#define R2 100000.0  // Bottom resistor (Ohms)

// PCNT Configuration
#define PCNT_UNIT           PCNT_UNIT_0
#define PCNT_H_LIM_VAL      0x7FFF        // Max count value
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    1             // Trigger an interrupt after this many counts
#define PCNT_THRESH0_VAL   -1

// Sleep Configuration
#define INACTIVITY_THRESHOLD 30000  // 30 seconds

// Endpoint declaration
#define METER_ENDPOINT      10
#define BATT_ENDPOINT       11
#define MAX_PULSE_COUNT     100000

#define MODEL_NAME          "WATERMETER_BAT"
#define MANUFACTURER_NAME   "JIMBO"

ZigbeeFlowSensor meter_ep(METER_ENDPOINT);
Preferences prefs;

volatile uint32_t total_count = 0;
volatile unsigned long last_pulse_time = 0;

static void setup_pcnt(void)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config;
    // Set PCNT input signal and control GPIOs
    pcnt_config.pulse_gpio_num = PULSE_INPUT_GPIO;
    pcnt_config.ctrl_gpio_num  = REFERENCE_PIN;
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.unit           = PCNT_UNIT;
    // What to do on the positive / negative edge of pulse input?
    pcnt_config.pos_mode       = PCNT_COUNT_DIS;   // Count up on the positive edge
    pcnt_config.neg_mode       = PCNT_COUNT_INC;   // Count up on the negative edge
    // What to do when control input is low or high?
    pcnt_config.lctrl_mode     = PCNT_MODE_KEEP; // Reverse counting direction if low
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
    // Set the maximum and minimum limit values to watch
    pcnt_config.counter_h_lim  = PCNT_H_LIM_VAL;
    pcnt_config.counter_l_lim  = PCNT_L_LIM_VAL;

    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* [Optional] Configure and enable the input filter (to filter glitches */
    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);

    /* [Optional] Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_0);
    /* [Optional] Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);

    /* [Optional] Register ISR handler and enable interrupts for PCNT unit */
    //pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    //pcnt_intr_enable(PCNT_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT);
}

static void pcnt_clear() {
          pcnt_counter_pause(PCNT_UNIT);
          pcnt_counter_clear(PCNT_UNIT);
}

static int16_t pcnt_get() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count);
    pcnt_counter_clear(PCNT_UNIT); // optional: clear after reading
    return count;
}


//------------------------ Read Battery ------------------------
float read_battery_voltage() {
  int adc_value = analogRead(BATTERY_ADC_PIN);
  float voltage = (adc_value / 4095.0) * 3.3;
  return voltage * ((R1 + R2) / R2);
}

//------------------------ Light Sleep ------------------------
void enter_light_sleep() {
  Serial.println("Entering light sleep...");
  gpio_wakeup_enable((gpio_num_t)PULSE_INPUT_GPIO, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  Serial.println("Woke up from light sleep.");
  last_pulse_time = millis();
}

//------------------------ Setup ------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PULSE_INPUT_GPIO, INPUT);
  pinMode(BOOT_PIN, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  prefs.begin("pulse_counter", false);
  prefs.putUInt("count", 0);
  total_count = prefs.getUInt("count", 0);
  Serial.println("Total count set to 0");
  last_pulse_time = millis();

  setup_pcnt();

  meter_ep.setManufacturerAndModel(MANUFACTURER_NAME, MODEL_NAME);
  meter_ep.setPowerSource(ZB_POWER_SOURCE_BATTERY, 75);
  meter_ep.setMinMaxValue(0, MAX_PULSE_COUNT);
  meter_ep.setTolerance(1.0);
  Zigbee.addEndpoint(&meter_ep);

  Serial.println("Zigbee initializing...");
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  } else {
    Serial.println("Zigbee started successfully!");
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  meter_ep.setReporting(0, 3000, 1.0);
  Serial.println();
}

//------------------------ Main Loop ------------------------
void loop() {
  static unsigned long last_report = 0;
  unsigned long now = millis();


  // Report battery & flow periodically
  if (now - last_report >= 3000) { // 5 minutes
    float batt = read_battery_voltage();
    uint8_t pct = min((uint8_t)( (batt - 3.0)/(4.1-3.0)*100 ), (uint8_t)100);
    Serial.printf("Battery: %.2fV (%u%%)\n", batt, pct);
    int16_t delta = pcnt_get();
    total_count += delta;
    prefs.putUInt("count", total_count);  // save updated value
    Serial.printf("Total count: %lu\n", total_count);
    meter_ep.setBatteryPercentage(pct);
    meter_ep.setFlow(total_count);
    meter_ep.report();
    last_report = now;
  }

  // Sleep on inactivity
  //if (now - last_pulse_time >= INACTIVITY_THRESHOLD) {
  //  enter_light_sleep();
  //}

  // Zigbee factory reset via long-press
  if (digitalRead(BOOT_PIN) == LOW) {
    delay(50);
    unsigned long t0 = millis();
    while (digitalRead(BOOT_PIN) == LOW) {
      if (millis() - t0 > 3000) {
        Serial.println("Factory resetting Zigbee...");
        Zigbee.factoryReset();
        break;
      }
      delay(50);
    }
  }

  delay(1000);
}
