#pragma once

#include <Arduino.h>
#include <r_adc_api.h>

namespace StatusConfig {
constexpr unsigned long CANTIMEOUT = 1000;
constexpr int TERMINATION_SENSE_PIN = -1;
constexpr int TERMINATION_ACTIVE_LEVEL = HIGH;
constexpr int TERMINATION_CONTROL_PIN = 3;
constexpr int TERMINATION_ON_LEVEL = HIGH;
// Buttons are active-low (wired to GND, INPUT_PULLUP)
constexpr int BUTTON1_PIN = 6;
constexpr int BUTTON2_PIN = 7;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 30;
constexpr unsigned long BUTTON_LONG_PRESS_MS = 1000;
constexpr int SD_CARD_DETECT_PIN = 8;
constexpr int SD_CARD_PRESENT_LEVEL = LOW;
constexpr bsp_io_port_pin_t VBATT_SENSE_PIN = BSP_IO_PORT_00_PIN_14;
constexpr adc_channel_t VBATT_ADC_CHANNEL = ADC_CHANNEL_9;
constexpr int VBATT_ADC_BITS = 12;
constexpr float VBATT_ADC_REFERENCE_VOLTS = 5.0f;
constexpr float VBATT_DIVIDER_RATIO = 6.0f;
constexpr float VBATT_VOLTS_PER_COUNT =
	VBATT_ADC_REFERENCE_VOLTS * VBATT_DIVIDER_RATIO / ((1UL << VBATT_ADC_BITS) - 1);
constexpr float VEHICLE_POWER_THRESHOLD_VOLTS = 6.0f;
}