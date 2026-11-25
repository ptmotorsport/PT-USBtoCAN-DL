#include <Arduino.h>
#include <FspTimer.h>
#include <SdFat.h>
#include <analog.h>
#include <FastLED.h>

#include "adc_binary_log.h"
#include "adc_sample_queue.h"

namespace {
constexpr uint8_t SD_CS = 10;
constexpr uint8_t SD_DETECT = 8;
constexpr uint8_t STOP_BUTTON = 7;
constexpr uint8_t LED_PIN = 2;
constexpr uint32_t MIN_INTERVAL_US = 50;
constexpr uint32_t MAX_INTERVAL_US = 150;
constexpr size_t QUEUE_SLOTS = 2048;
constexpr size_t BATCH_MIN_SAMPLES = 256;
constexpr uint32_t SYNC_INTERVAL_MS = 1000;

enum class CaptureFault : uint8_t { None, Adc, QueueFull, Late };

SdFat sd;
FsFile file;
ADC_Container adc(0, nullptr);
FspTimer sampleTimer;
FspTimer tachTimer;
TachPeriod tachPeriod;
bool tachTimerReady = false;
AdcSampleQueue<QUEUE_SLOTS, AdcBinaryLog::Sample> samples;
CRGB leds[5];
volatile CaptureFault captureFault = CaptureFault::None;
volatile uint32_t lastSampleMicros = 0;
volatile bool haveSampleTime = false;
bool capturing = false;
uint32_t rowsWritten = 0;
uint32_t lastSyncMillis = 0;

void tachCallback(timer_callback_args_t *args) {
  if (args->event == TIMER_EVENT_CAPTURE_B) tachPeriod.edge(args->capture);
}

bool beginTachCapture() {
  if (R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD) / 16 != TachConfig::TickHz) return false;
  if (R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_06,
      IOPORT_CFG_PERIPHERAL_PIN | IOPORT_CFG_PIM_TTL | IOPORT_PERIPHERAL_GPT1) != FSP_SUCCESS) {
    return false;
  }
  FspTimer::force_use_of_pwm_reserved_timer();
  if (!tachTimer.begin(TIMER_MODE_PERIODIC, GPT_TIMER, 0,
      TachConfig::CounterModulus, 1, TIMER_SOURCE_DIV_16, tachCallback)) return false;
  if (!tachTimer.set_source_capture_b(static_cast<gpt_source_t>(
      GPT_SOURCE_GTIOCB_FALLING_WHILE_GTIOCA_LOW |
      GPT_SOURCE_GTIOCB_FALLING_WHILE_GTIOCA_HIGH))) return false;
  auto *extension = static_cast<gpt_extended_cfg_t *>(
      const_cast<void *>(tachTimer.get_cfg()->p_extend));
  extension->capture_filter_gtiocb = GPT_CAPTURE_FILTER_PCLKD_DIV_64;
  // Match the ADC callback priority so period state cannot be read mid-update.
  if (!tachTimer.setup_capture_b_irq(12) || !tachTimer.open()) return false;
  tachTimerReady = true;
  return true;
}

void sampleCallback(timer_callback_args_t *) {
  if (captureFault != CaptureFault::None) return;
  const uint32_t now = micros();
  const uint32_t interval = now - lastSampleMicros;
  if (haveSampleTime && (interval < MIN_INTERVAL_US || interval > MAX_INTERVAL_US)) {
    captureFault = CaptureFault::Late;
    return;
  }
  lastSampleMicros = now;
  haveSampleTime = true;
  const uint32_t periodTicks = tachPeriod.period(tachTimer.get_counter());

  if (R_ADC_ScanStart(&adc.ctrl) != FSP_SUCCESS) {
    captureFault = CaptureFault::Adc;
    return;
  }
  adc_status_t status;
  bool complete = false;
  for (unsigned int attempt = 0; attempt < 10000; ++attempt) {
    if (R_ADC_StatusGet(&adc.ctrl, &status) != FSP_SUCCESS) break;
    if (status.state != ADC_STATE_SCAN_IN_PROGRESS) {
      complete = true;
      break;
    }
  }
  uint16_t value;
  if (!complete || R_ADC_Read(&adc.ctrl, ADC_CHANNEL_2, &value) != FSP_SUCCESS) {
    captureFault = CaptureFault::Adc;
  } else if (!samples.push({value, periodTicks})) {
    captureFault = CaptureFault::QueueFull;
  }
}

void showStatus(bool error, bool recording) {
  FastLED.clear();
  leds[0] = error ? CRGB(25, 0, 0) : CRGB(0, 25, 0);
  leds[1] = error ? CRGB(25, 0, 0) :
      recording ? CRGB(0, 25, 0) : CRGB(25, 25, 25);
  leds[4] = CRGB(25, 0, 25);
  FastLED.show();
}

void reportError(const char *message) {
  Serial.print(F("ADC test ERROR: "));
  Serial.println(message);
  Serial.print(F("SD error code/data: "));
  Serial.print(sd.sdErrorCode(), HEX);
  Serial.print('/');
  Serial.println(sd.sdErrorData(), HEX);
  showStatus(true, false);
}

bool createFile() {
  char name[16];
  for (unsigned int i = 0; i < 10000; ++i) {
    snprintf(name, sizeof(name), "ADC%04u.BIN", i);
    if (sd.exists(name)) continue;
    if (!file.open(name, O_WRONLY | O_CREAT | O_EXCL)) return false;
    Serial.print(F("Recording to "));
    Serial.println(name);
    return true;
  }
  return false;
}

bool writeSamples() {
  uint8_t buffer[1024];
  size_t used = 0;
  uint32_t rows = 0;
  AdcBinaryLog::Sample value;
  while (used + AdcBinaryLog::RecordSize <= sizeof(buffer) && samples.pop(value)) {
    AdcBinaryLog::encodeSample(buffer + used, value);
    used += AdcBinaryLog::RecordSize;
    ++rows;
  }
  if (!used) return true;
  if (file.write(buffer, used) != used || file.getWriteError()) return false;
  rowsWritten += rows;
  return true;
}

void stopCapture(const char *reason, bool error, bool sdUsable) {
  if (capturing) {
    if (!sampleTimer.stop()) {
      reason = "Timer stop failed";
      error = true;
    }
    capturing = false;
  }
  const CaptureFault fault = captureFault;
  captureFault = CaptureFault::Adc;
  if (tachTimerReady) {
    const bool stopped = tachTimer.stop();
    const bool closed = tachTimer.close();
    tachTimerReady = false;
    if (!stopped || !closed) {
      reason = "Tach timer stop/close failed";
      error = true;
    }
  }
  if (fault != CaptureFault::None && !error) {
    reason = fault == CaptureFault::Adc ? "ADC conversion failed" :
        fault == CaptureFault::QueueFull ? "Sample buffer full" : "Sample timer interval out of range";
    error = true;
  }
  if (sdUsable) {
    uint32_t previous;
    do {
      previous = rowsWritten;
      if (!writeSamples()) {
        reason = "SD write failed; final rows may be incomplete";
        error = true;
        sdUsable = false;
        break;
      }
    } while (rowsWritten != previous);
  }
  if (sdUsable && !file.sync()) {
    reason = "SD sync failed; final rows may be incomplete";
    error = true;
    sdUsable = false;
  }
  if (sdUsable && !file.close()) {
    reason = "SD close failed; final rows may be incomplete";
    error = true;
  }
  if (error) {
    reportError(reason);
  } else {
    Serial.print(F("ADC test stopped: "));
    Serial.println(reason);
    showStatus(false, false);
  }
  Serial.print(F("Rows written: "));
  Serial.println(rowsWritten);
}
}

void setup() {
  pinMode(SD_DETECT, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, 5);
  Serial.begin(115200);
  delay(500);
  if (digitalRead(SD_DETECT) != LOW || !sd.begin(SD_CS, SD_SCK_MHZ(12))) {
    reportError("SD card absent or initialization failed");
    return;
  }
  if (!createFile() ||
      file.write(AdcBinaryLog::Header, AdcBinaryLog::HeaderSize) != AdcBinaryLog::HeaderSize ||
      file.getWriteError() || !file.sync()) {
    reportError("Cannot create or sync ADC binary file");
    return;
  }
  pinPeripheral(BSP_IO_PORT_00_PIN_02, IOPORT_CFG_ANALOG_ENABLE);
  adc.cfg.resolution = ADC_RESOLUTION_14_BIT;
  adc.channel_cfg.scan_mask = 1UL << ADC_CHANNEL_2;
  if (R_ADC_Open(&adc.ctrl, &adc.cfg) != FSP_SUCCESS ||
      R_ADC_ScanCfg(&adc.ctrl, &adc.channel_cfg) != FSP_SUCCESS) {
    stopCapture("ADC initialization failed", true, true);
    return;
  }
  if (!beginTachCapture()) {
    stopCapture("P106 tach input capture initialization failed", true, true);
    return;
  }
  uint8_t timerType = GPT_TIMER;
  const int8_t timerChannel = FspTimer::get_available_timer(timerType);
  if (timerChannel < 0 ||
      !sampleTimer.begin(TIMER_MODE_PERIODIC, timerType, timerChannel,
          static_cast<float>(AdcBinaryLog::SampleRateHz), 50.0f, sampleCallback) ||
      !sampleTimer.setup_overflow_irq(12) || !sampleTimer.open()) {
    stopCapture("Sample timer initialization failed", true, true);
    return;
  }
  Serial.println(F("14-bit AN002 capture at 10000 Hz; hold button 2 to stop."));
  Serial.println(F("P106 tach: falling edge, 1 pulse/revolution, 300 ms timeout."));
  showStatus(false, true);
  lastSyncMillis = millis();
  if (!tachTimer.start()) {
    stopCapture("Tach timer start failed", true, true);
    return;
  }
  capturing = true;
  if (!sampleTimer.start()) stopCapture("Sample timer start failed", true, true);
}

void loop() {
  if (!capturing) return;
  if (digitalRead(SD_DETECT) != LOW) {
    stopCapture("SD card removed; final rows may be incomplete", true, false);
    return;
  }
  const CaptureFault fault = captureFault;
  if (fault != CaptureFault::None) {
    const char *reason = fault == CaptureFault::Adc ? "ADC conversion failed" :
        fault == CaptureFault::QueueFull ? "Sample buffer full" : "Sample timer interval out of range";
    stopCapture(reason, true, true);
    return;
  }
  static uint32_t buttonPressedAt = 0;
  static bool buttonHeld = false;
  if (digitalRead(STOP_BUTTON) == LOW) {
    if (!buttonHeld) {
      buttonHeld = true;
      buttonPressedAt = millis();
    } else if (millis() - buttonPressedAt >= 1000) {
      stopCapture("button 2", false, true);
      return;
    }
  } else {
    buttonHeld = false;
  }
  if (samples.size() >= BATCH_MIN_SAMPLES && !writeSamples()) {
    stopCapture("SD write failed; final rows may be incomplete", true, false);
    return;
  }
  if (millis() - lastSyncMillis >= SYNC_INTERVAL_MS) {
    if (!file.sync()) {
      stopCapture("SD sync failed; final rows may be incomplete", true, false);
      return;
    }
    lastSyncMillis = millis();
  }
}
