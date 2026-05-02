#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <driver/i2s_std.h>
#include <math.h>

// ESP32-2432S028R, "Cheap Yellow Display" / CYD
//
// I2S HIRES ADC Audio I2S Capture Card Module:
// - ADC/module is I2S master.
// - ESP32 is I2S slave.
// - Set the module jumper so CLOCK OUT feeds MCLK IN on the module itself.
//
// These GPIOs are the external pins you said you can use. GPIO27 is left free
// because this slave-I2S setup does not need an ESP32 MCLK pin.
static constexpr int PIN_I2S_BCK = 23;
static constexpr int PIN_I2S_WS = 19;
static constexpr int PIN_I2S_DATA = 18;
static constexpr int PIN_I2S_SPARE = 27;
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

static constexpr int PIN_LED_R = 4;
static constexpr int PIN_LED_G = 16;
static constexpr int PIN_LED_B = 17;
static constexpr int PIN_BUZZER = 26;
static constexpr bool RGB_LED_ACTIVE_LOW = true;
static constexpr uint8_t RGB_LED_PWM_BITS = 8;
static constexpr uint16_t RGB_LED_PWM_FREQ = 1000;
static constexpr uint8_t BEEP_PWM_BITS = 8;
static constexpr uint16_t BEEP_PWM_FREQ = 2400;
static constexpr uint8_t BEEP_PWM_DUTY = 128;
static constexpr uint8_t BEEP_MS = 4;
static constexpr uint16_t BLUE_PULSE_FLASH_MS = 25;
static constexpr float CPS_LED_FULL_SCALE = 40.0f;
static constexpr bool TOUCH_MIRROR_X = true;
static constexpr uint8_t LOWPASS_SHIFT = 2;  // 2 = alpha 1/4, mild HF smoothing.
static constexpr uint8_t HYSTERESIS_LOW_DIV = 2;
static constexpr uint8_t TRAP_RISE_DEFAULT = 3;
static constexpr uint8_t TRAP_GAP_DEFAULT = 1;
static constexpr uint8_t TRAP_RISE_MIN = 1;
static constexpr uint8_t TRAP_RISE_MAX = 16;
static constexpr uint8_t TRAP_GAP_MIN = 0;
static constexpr uint8_t TRAP_GAP_MAX = 16;
static constexpr uint8_t TRAP_BUFFER_SIZE = TRAP_RISE_MAX * 2 + TRAP_GAP_MAX + 1;

class CYDDisplay : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 panel;
  lgfx::Bus_SPI displayBus;
  lgfx::Light_PWM backlight;
  lgfx::Touch_XPT2046 touch;

public:
  CYDDisplay() {
    {
      auto cfg = displayBus.config();
      cfg.spi_host = HSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = 1;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc = 2;
      displayBus.config(cfg);
      panel.setBus(&displayBus);
    }

    {
      auto cfg = panel.config();
      cfg.pin_cs = 15;
      cfg.pin_rst = -1;
      cfg.pin_busy = -1;
      cfg.memory_width = 240;
      cfg.memory_height = 320;
      cfg.panel_width = 240;
      cfg.panel_height = 320;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      panel.config(cfg);
    }

    {
      auto cfg = backlight.config();
      cfg.pin_bl = 21;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      backlight.config(cfg);
      panel.setLight(&backlight);
    }

    {
      auto cfg = touch.config();
      cfg.x_min = 200;
      cfg.x_max = 3700;
      cfg.y_min = 240;
      cfg.y_max = 3800;
      cfg.pin_int = 36;
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;
      cfg.spi_host = VSPI_HOST;
      cfg.freq = 2500000;
      cfg.pin_sclk = 25;
      cfg.pin_mosi = 32;
      cfg.pin_miso = 39;
      cfg.pin_cs = 33;
      touch.config(cfg);
      panel.setTouch(&touch);
    }

    setPanel(&panel);
  }
};

static CYDDisplay lcd;

static constexpr uint16_t C_BLACK = 0x0000;
static constexpr uint16_t C_WHITE = 0xFFFF;
static constexpr uint16_t C_GREEN = 0x07E0;
static constexpr uint16_t C_YELLOW = 0xFFE0;
static constexpr uint16_t C_ORANGE = 0xFD20;
static constexpr uint16_t C_RED = 0xF800;

static constexpr uint8_t CHANNELS = 2;
static constexpr size_t I2S_FRAMES = 512;
static constexpr uint16_t MAX_PULSE_SAMPLES = 256;
static constexpr uint16_t PRE_TRIGGER_SAMPLES = 10;
static constexpr uint16_t REARM_SAMPLES = 4;
static constexpr uint16_t HISTOGRAM_BINS = 192;
static constexpr uint16_t SCOPE_COLUMNS = 320;
static constexpr uint16_t SCOPE_SAMPLES_PER_COLUMN = 4;
static constexpr uint8_t BASELINE_SHIFT = 12;

static constexpr uint32_t SAMPLE_RATE_OPTIONS[] = {48000, 96000, 192000};

enum ViewMode {
  VIEW_SPECTRUM,
  VIEW_PULSE,
  VIEW_SCOPE,
  VIEW_SETTINGS
};

struct RuntimeSettings {
  float spectrumMaxFraction = 0.05f;
  float thresholdFraction = 0.007f;
  uint8_t sampleRateIndex = 1;
  uint16_t pulseSamples = 48;
  uint8_t selectedChannel = 0;
  int8_t pulsePolarity = 1;
  bool spectrumLogCounts = false;
  bool lowPassEnabled = false;
  bool hysteresisEnabled = true;
  bool trapezoidEnabled = true;
  bool beepEnabled = false;
  uint8_t trapRiseSamples = TRAP_RISE_DEFAULT;
  uint8_t trapGapSamples = TRAP_GAP_DEFAULT;
};

struct Rect {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

static RuntimeSettings settings;
static ViewMode viewMode = VIEW_SPECTRUM;
static uint8_t settingsPage = 0;

static int32_t i2sFrames[I2S_FRAMES * CHANNELS];
static int32_t pulseBuffer[MAX_PULSE_SAMPLES];
static int32_t preTrigger[PRE_TRIGGER_SAMPLES];
static uint32_t histogram[HISTOGRAM_BINS];
static int32_t scopeMin[SCOPE_COLUMNS];
static int32_t scopeMax[SCOPE_COLUMNS];

static size_t framesAvailable = 0;
static size_t frameIndex = 0;
static int64_t baseline = 0;
static int32_t lowPassState = 0;
static int32_t trapBuffer[TRAP_BUFFER_SIZE];
static int64_t trapAccumulator = 0;
static bool baselineReady = false;
static bool lowPassReady = false;
static uint8_t trapIndex = 0;
static uint16_t preIndex = 0;
static uint16_t captureIndex = 0;
static uint16_t rearmCountdown = 0;
static int32_t currentPeak = 0;
static int32_t lastPeak = 0;
static uint32_t pulseCount = 0;
static uint32_t histogramMax = 1;
static uint32_t lastPulseMs = 0;
static uint32_t rateWindowStartMs = 0;
static uint32_t rateWindowPulses = 0;
static float pulsesPerSecond = 0.0f;
static bool pulseReady = false;
static bool paused = false;
static bool i2sInstalled = false;
static i2s_chan_handle_t rxHandle = nullptr;
static bool redrawRequested = true;
static bool wasTouching = false;
static uint32_t blueFlashUntilMs = 0;
static uint32_t beepUntilMs = 0;

enum DetectorState {
  WAITING_FOR_PULSE,
  CAPTURING_PULSE,
  REARMING_DETECTOR
};

static DetectorState detectorState = WAITING_FOR_PULSE;

static void handleTouch();

static uint32_t sampleRate() {
  return SAMPLE_RATE_OPTIONS[settings.sampleRateIndex];
}

static int32_t thresholdCounts() {
  return static_cast<int32_t>(2147483647.0f * settings.thresholdFraction);
}

static int32_t histogramFullScaleCounts() {
  float maxFraction = settings.spectrumMaxFraction;
  if (maxFraction < settings.thresholdFraction * 1.2f) {
    maxFraction = settings.thresholdFraction * 1.2f;
  }
  return static_cast<int32_t>(2147483647.0f * maxFraction);
}

static uint32_t pulseWindowUs() {
  return (static_cast<uint32_t>(settings.pulseSamples) * 1000000UL) / sampleRate();
}

static uint16_t colorBg() { return C_BLACK; }
static uint16_t colorPanel() { return 0x1082; }
static uint16_t colorGrid() { return 0x2945; }
static uint16_t colorAxis() { return 0x5AEB; }
static uint16_t colorWave() { return C_GREEN; }
static uint16_t colorAccent() { return C_ORANGE; }
static uint16_t colorText() { return C_WHITE; }

static uint8_t clampByte(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return static_cast<uint8_t>(value);
}

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static void cpsRedGreen(float cps, uint8_t &r, uint8_t &g) {
  float ratio = cps / CPS_LED_FULL_SCALE;
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  r = clampByte(static_cast<int>(ratio * 255.0f + 0.5f));
  g = clampByte(static_cast<int>((1.0f - ratio) * 255.0f + 0.5f));
}

static int contentTop() { return 30; }
static int navTop() { return lcd.height() - 40; }
static int contentBottom() { return navTop() - 4; }
static int contentHeight() { return contentBottom() - contentTop() + 1; }
static int screenW() { return lcd.width(); }
static int screenH() { return lcd.height(); }

static void resetDetector() {
  baselineReady = false;
  baseline = 0;
  lowPassReady = false;
  lowPassState = 0;
  trapAccumulator = 0;
  trapIndex = 0;
  for (uint8_t i = 0; i < TRAP_BUFFER_SIZE; i++) {
    trapBuffer[i] = 0;
  }
  preIndex = 0;
  captureIndex = 0;
  rearmCountdown = 0;
  currentPeak = 0;
  pulseReady = false;
  detectorState = WAITING_FOR_PULSE;
  for (uint16_t i = 0; i < PRE_TRIGGER_SAMPLES; i++) {
    preTrigger[i] = 0;
  }
}

static void resetMeasurements() {
  pulseCount = 0;
  lastPeak = 0;
  lastPulseMs = 0;
  histogramMax = 1;
  rateWindowStartMs = millis();
  rateWindowPulses = 0;
  pulsesPerSecond = 0.0f;
  for (uint16_t i = 0; i < HISTOGRAM_BINS; i++) {
    histogram[i] = 0;
  }
  resetDetector();
  redrawRequested = true;
}

static bool setupI2S();

static void restartI2S() {
  if (i2sInstalled) {
    i2s_channel_disable(rxHandle);
    i2s_del_channel(rxHandle);
    rxHandle = nullptr;
    i2sInstalled = false;
  }
  framesAvailable = 0;
  frameIndex = 0;
  resetMeasurements();
  setupI2S();
}

static bool readI2SBlock() {
  size_t bytesRead = 0;
  const esp_err_t err = i2s_channel_read(rxHandle,
                                         reinterpret_cast<void *>(i2sFrames),
                                         sizeof(i2sFrames),
                                         &bytesRead,
                                         100);
  if (err != ESP_OK) {
    framesAvailable = 0;
    frameIndex = 0;
    return false;
  }

  framesAvailable = bytesRead / (sizeof(int32_t) * CHANNELS);
  frameIndex = 0;
  return framesAvailable > 0;
}

static int32_t readSample() {
  while (frameIndex >= framesAvailable) {
    if (!readI2SBlock()) {
      handleTouch();
      delay(1);
    }
  }

  const int32_t sample = i2sFrames[frameIndex * CHANNELS + settings.selectedChannel];
  frameIndex++;
  return sample;
}

static int32_t pulseSignalFromRaw(int32_t rawSample) {
  if (!baselineReady) {
    baseline = rawSample;
    baselineReady = true;
  }

  int64_t signal = (static_cast<int64_t>(rawSample) - baseline) * settings.pulsePolarity;
  if (signal > INT32_MAX) signal = INT32_MAX;
  if (signal < INT32_MIN) signal = INT32_MIN;
  return static_cast<int32_t>(signal);
}

static void updateBaseline(int32_t rawSample) {
  baseline += (static_cast<int64_t>(rawSample) - baseline) / (1L << BASELINE_SHIFT);
}

static int32_t detectionSignal(int32_t rawSignal) {
  int32_t signal = rawSignal;

  if (settings.lowPassEnabled) {
    if (!lowPassReady) {
      lowPassState = rawSignal;
      lowPassReady = true;
    } else {
      lowPassState += static_cast<int32_t>(
          (static_cast<int64_t>(rawSignal) - lowPassState) / (1L << LOWPASS_SHIFT));
    }
    signal = lowPassState;
  } else {
    lowPassReady = false;
  }

  if (!settings.trapezoidEnabled) {
    trapAccumulator = 0;
    trapIndex = 0;
    return signal;
  }

  const uint8_t l = settings.trapRiseSamples;
  const uint8_t g = settings.trapGapSamples;
  const uint8_t i0 = trapIndex;
  const uint8_t iL = (trapIndex + TRAP_BUFFER_SIZE - l) % TRAP_BUFFER_SIZE;
  const uint8_t iLG = (trapIndex + TRAP_BUFFER_SIZE - l - g) % TRAP_BUFFER_SIZE;
  const uint8_t i2LG = (trapIndex + TRAP_BUFFER_SIZE - (2 * l) - g) % TRAP_BUFFER_SIZE;

  trapAccumulator += static_cast<int64_t>(signal) - trapBuffer[iL] - trapBuffer[iLG] +
                     trapBuffer[i2LG];
  trapBuffer[i0] = signal;
  trapIndex = (trapIndex + 1) % TRAP_BUFFER_SIZE;

  int64_t shaped = trapAccumulator / settings.trapRiseSamples;
  if (shaped > INT32_MAX) shaped = INT32_MAX;
  if (shaped < INT32_MIN) shaped = INT32_MIN;
  return static_cast<int32_t>(shaped);
}

static uint8_t ledDuty(uint8_t logicalDuty) {
  return RGB_LED_ACTIVE_LOW ? 255 - logicalDuty : logicalDuty;
}

static void setupStatusLed() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_LED_R, RGB_LED_PWM_FREQ, RGB_LED_PWM_BITS);
  ledcAttach(PIN_LED_G, RGB_LED_PWM_FREQ, RGB_LED_PWM_BITS);
  ledcAttach(PIN_LED_B, RGB_LED_PWM_FREQ, RGB_LED_PWM_BITS);
#else
  ledcSetup(0, RGB_LED_PWM_FREQ, RGB_LED_PWM_BITS);
  ledcSetup(1, RGB_LED_PWM_FREQ, RGB_LED_PWM_BITS);
  ledcSetup(2, RGB_LED_PWM_FREQ, RGB_LED_PWM_BITS);
  ledcAttachPin(PIN_LED_R, 0);
  ledcAttachPin(PIN_LED_G, 1);
  ledcAttachPin(PIN_LED_B, 2);
#endif
}

static void setupBeeper() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_BUZZER, BEEP_PWM_FREQ, BEEP_PWM_BITS);
  ledcWrite(PIN_BUZZER, 0);
#else
  ledcSetup(3, BEEP_PWM_FREQ, BEEP_PWM_BITS);
  ledcAttachPin(PIN_BUZZER, 3);
  ledcWrite(3, 0);
#endif
}

static void writeBeeper(uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_BUZZER, duty);
#else
  ledcWrite(3, duty);
#endif
}

static void updateBeeper() {
  if (!settings.beepEnabled || static_cast<int32_t>(beepUntilMs - millis()) <= 0) {
    writeBeeper(0);
  }
}

static void writeStatusLed(uint8_t r, uint8_t g, uint8_t b) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_LED_R, ledDuty(r));
  ledcWrite(PIN_LED_G, ledDuty(g));
  ledcWrite(PIN_LED_B, ledDuty(b));
#else
  ledcWrite(0, ledDuty(r));
  ledcWrite(1, ledDuty(g));
  ledcWrite(2, ledDuty(b));
#endif
}

static void updateStatusLed() {
  if (static_cast<int32_t>(blueFlashUntilMs - millis()) > 0) {
    writeStatusLed(0, 0, 255);
    return;
  }

  uint8_t r = 0;
  uint8_t g = 0;
  cpsRedGreen(pulsesPerSecond, r, g);
  writeStatusLed(r, g, 0);
}

static void beginPulseCapture(int32_t firstPulseSample) {
  uint16_t out = 0;
  for (uint16_t i = 0; i < PRE_TRIGGER_SAMPLES; i++) {
    const uint16_t index = (preIndex + i) % PRE_TRIGGER_SAMPLES;
    pulseBuffer[out++] = preTrigger[index];
  }

  pulseBuffer[out++] = firstPulseSample;
  captureIndex = out;
  currentPeak = firstPulseSample;
  detectorState = CAPTURING_PULSE;
}

static void finishPulseCapture() {
  lastPeak = currentPeak;
  pulseCount++;
  rateWindowPulses++;
  lastPulseMs = millis();
  blueFlashUntilMs = lastPulseMs + BLUE_PULSE_FLASH_MS;
  if (settings.beepEnabled) {
    beepUntilMs = lastPulseMs + BEEP_MS;
    writeBeeper(BEEP_PWM_DUTY);
  }

  int32_t peak = lastPeak;
  if (peak < 0) peak = 0;
  const int32_t histogramFullScale = histogramFullScaleCounts();
  if (peak >= histogramFullScale) {
    pulseReady = true;
    rearmCountdown = REARM_SAMPLES;
    detectorState = REARMING_DETECTOR;
    return;
  }
  uint16_t bin = static_cast<uint16_t>(
      (static_cast<uint64_t>(peak) * HISTOGRAM_BINS) / histogramFullScale);
  if (bin >= HISTOGRAM_BINS) {
    bin = HISTOGRAM_BINS - 1;
  }
  histogram[bin]++;
  if (histogram[bin] > histogramMax) {
    histogramMax = histogram[bin];
  }

  pulseReady = true;
  rearmCountdown = REARM_SAMPLES;
  detectorState = REARMING_DETECTOR;
}

static void processSample(int32_t rawSample) {
  const int32_t rawSignal = pulseSignalFromRaw(rawSample);
  const int32_t signal = detectionSignal(rawSignal);

  switch (detectorState) {
    case WAITING_FOR_PULSE:
      if (signal >= thresholdCounts()) {
        beginPulseCapture(signal);
      } else {
        preTrigger[preIndex] = signal;
        preIndex = (preIndex + 1) % PRE_TRIGGER_SAMPLES;
        updateBaseline(rawSample);
      }
      break;

    case CAPTURING_PULSE:
      if (captureIndex < MAX_PULSE_SAMPLES) {
        pulseBuffer[captureIndex++] = signal;
      }
      if (signal > currentPeak) {
        currentPeak = signal;
      }
      if (captureIndex >= settings.pulseSamples || captureIndex >= MAX_PULSE_SAMPLES) {
        finishPulseCapture();
      }
      break;

    case REARMING_DETECTOR:
      if (settings.hysteresisEnabled) {
        if (signal <= thresholdCounts() / HYSTERESIS_LOW_DIV) {
          detectorState = WAITING_FOR_PULSE;
        }
      } else if (rearmCountdown > 0) {
        rearmCountdown--;
      } else {
        detectorState = WAITING_FOR_PULSE;
      }
      break;
  }
}

static void updatePulseRate() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - rateWindowStartMs;
  if (elapsed >= 1000) {
    pulsesPerSecond = static_cast<float>(rateWindowPulses) * 1000.0f /
                      static_cast<float>(elapsed);
    rateWindowPulses = 0;
    rateWindowStartMs = now;
  }
}

static void drawButton(const Rect &r, const char *label, bool active) {
  const uint16_t fill = active ? colorAccent() : colorPanel();
  lcd.fillRoundRect(r.x, r.y, r.w, r.h, 5, fill);
  lcd.drawRoundRect(r.x, r.y, r.w, r.h, 5, colorAxis());
  lcd.setTextColor(active ? C_BLACK : colorText(), fill);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
  lcd.setTextDatum(textdatum_t::top_left);
}

static Rect navButton(uint8_t index) {
  const int w = screenW() / 4;
  return Rect{static_cast<int16_t>(index * w + 2),
              static_cast<int16_t>(navTop() + 3),
              static_cast<int16_t>(w - 4),
              34};
}

static Rect spectrumResetButton() {
  const int16_t gaugeX = static_cast<int16_t>(screenW() - 16 - 4);
  return Rect{static_cast<int16_t>(gaugeX - 68),
              static_cast<int16_t>(contentTop() + 8),
              62,
              30};
}

static Rect spectrumLogButton() {
  const int16_t gaugeX = static_cast<int16_t>(screenW() - 16 - 4);
  return Rect{static_cast<int16_t>(gaugeX - 68),
              static_cast<int16_t>(contentTop() + 44),
              62,
              30};
}

static Rect pulseBeepButton() {
  return Rect{static_cast<int16_t>(screenW() - 82),
              static_cast<int16_t>(contentTop() + 8),
              72,
              30};
}

static void drawNav() {
  lcd.fillRect(0, navTop(), screenW(), screenH() - navTop(), colorBg());
  drawButton(navButton(0), "Spectrum", viewMode == VIEW_SPECTRUM);
  drawButton(navButton(1), "Pulse", viewMode == VIEW_PULSE);
  drawButton(navButton(2), "Scope", viewMode == VIEW_SCOPE);
  drawButton(navButton(3), "Settings", viewMode == VIEW_SETTINGS);
}

static void drawHeader(const char *title) {
  lcd.fillRect(0, 0, screenW(), contentTop(), colorBg());
  lcd.setTextColor(colorText(), colorBg());
  lcd.setTextDatum(textdatum_t::top_left);
  lcd.drawString(title, 4, 6);
  lcd.setCursor(104, 6);
  lcd.printf("%luk CH%c", sampleRate() / 1000, settings.selectedChannel == 0 ? 'L' : 'R');
  lcd.setCursor(190, 6);
  lcd.printf("%.1f CPS", pulsesPerSecond);
  lcd.setCursor(268, 6);
  lcd.print(paused ? "PAU" : "RUN");
}

static void drawPlotFrame() {
  lcd.fillRect(0, contentTop(), screenW(), contentHeight(), colorBg());
  lcd.drawFastHLine(0, contentTop(), screenW(), colorAxis());
  lcd.drawFastHLine(0, contentBottom(), screenW(), colorAxis());
  for (int x = 0; x < screenW(); x += screenW() / 10) {
    lcd.drawFastVLine(x, contentTop(), contentHeight(), colorGrid());
  }
  for (int y = contentTop(); y <= contentBottom(); y += contentHeight() / 4) {
    lcd.drawFastHLine(0, y, screenW(), colorGrid());
  }
}

static int amplitudeY(int32_t value, int32_t fullScale) {
  if (value < 0) value = 0;
  if (value > fullScale) value = fullScale;
  const float normalized = static_cast<float>(value) / static_cast<float>(fullScale);
  return contentBottom() - static_cast<int>(normalized * (contentHeight() - 4));
}

static void drawSpectrum() {
  drawHeader("Spectrum");
  drawPlotFrame();

  const int gaugeW = 16;
  const int gaugeX = screenW() - gaugeW - 4;
  const int plotRight = gaugeX - 6;
  const float logMax = log10f(static_cast<float>(histogramMax) + 1.0f);
  for (uint16_t i = 0; i < HISTOGRAM_BINS; i++) {
    const int x0 = static_cast<int>((static_cast<uint32_t>(i) * plotRight) / HISTOGRAM_BINS);
    int x1 = static_cast<int>((static_cast<uint32_t>(i + 1) * plotRight) / HISTOGRAM_BINS);
    if (x1 <= x0) x1 = x0 + 1;
    const int barWidth = x1 - x0;
    int barHeight = 0;
    if (histogramMax > 0 && histogram[i] > 0) {
      if (settings.spectrumLogCounts && logMax > 0.0f) {
        const float normalized = log10f(static_cast<float>(histogram[i]) + 1.0f) / logMax;
        barHeight = static_cast<int>(normalized * static_cast<float>(contentHeight() - 5));
      } else {
        barHeight = static_cast<int>((static_cast<uint64_t>(histogram[i]) *
                                      (contentHeight() - 5)) /
                                     histogramMax);
      }
    }
    if (barHeight > 0) {
      lcd.fillRect(x0, contentBottom() - barHeight + 1, barWidth, barHeight, colorWave());
    }
  }

  const int thresholdX = static_cast<int>(
      (static_cast<float>(thresholdCounts()) / static_cast<float>(histogramFullScaleCounts())) *
      plotRight);
  if (thresholdX >= 0 && thresholdX < plotRight) {
    lcd.drawFastVLine(thresholdX, contentTop(), contentHeight(), colorAxis());
  }

  const float cpsClamped = pulsesPerSecond > CPS_LED_FULL_SCALE ? CPS_LED_FULL_SCALE
                                                                : pulsesPerSecond;
  const int gaugeH = static_cast<int>((cpsClamped / CPS_LED_FULL_SCALE) *
                                      (contentHeight() - 6));
  uint8_t gaugeR = 0;
  uint8_t gaugeG = 0;
  cpsRedGreen(pulsesPerSecond, gaugeR, gaugeG);
  const uint16_t gaugeColor = rgb565(gaugeR, gaugeG, 0);
  lcd.drawRect(gaugeX, contentTop() + 3, gaugeW, contentHeight() - 6, colorAxis());
  if (gaugeH > 0) {
    lcd.fillRect(gaugeX + 2, contentBottom() - gaugeH - 1, gaugeW - 4, gaugeH, gaugeColor);
  }
  lcd.setTextColor(colorText(), colorBg());
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.drawString("CPS", gaugeX + gaugeW / 2, contentTop() + 14);
  lcd.setTextDatum(textdatum_t::top_left);

  drawButton(spectrumResetButton(), "Reset", false);
  drawButton(spectrumLogButton(), settings.spectrumLogCounts ? "Log ON" : "Log OFF",
             settings.spectrumLogCounts);

  lcd.setTextColor(colorText(), colorBg());
  lcd.setCursor(4, contentBottom() - 14);
  lcd.printf("N %lu", static_cast<unsigned long>(pulseCount));
  lcd.setCursor(78, contentBottom() - 14);
  lcd.printf("Zoom %.0f%%", settings.spectrumMaxFraction * 100.0f);
  lcd.setCursor(162, contentBottom() - 14);
  lcd.print(settings.spectrumLogCounts ? "LogY" : "LinY");
  lcd.setCursor(206, contentBottom() - 14);
  lcd.printf("%.1f CPS", pulsesPerSecond);
  drawNav();
}

static void drawPulse() {
  drawHeader("Pulse");
  drawPlotFrame();

  const int64_t scaledPeak = static_cast<int64_t>(lastPeak) * 6 / 5;
  int32_t fullScale = lastPeak > thresholdCounts()
                          ? (scaledPeak > INT32_MAX ? INT32_MAX : static_cast<int32_t>(scaledPeak))
                          : thresholdCounts() * 2;
  if (fullScale <= 0) fullScale = thresholdCounts() * 2;

  const int thresholdY = amplitudeY(thresholdCounts(), fullScale);
  lcd.drawFastHLine(0, thresholdY, screenW(), colorAxis());
  drawButton(pulseBeepButton(), settings.beepEnabled ? "Bip ON" : "Bip OFF",
             settings.beepEnabled);

  if (pulseCount > 0 && lastPeak > 0) {
    int lastX = 0;
    int lastY = amplitudeY(pulseBuffer[0], fullScale);
    const uint16_t samples = settings.pulseSamples;
    for (uint16_t i = 1; i < samples; i++) {
      const int x = static_cast<int>((static_cast<uint32_t>(i) * (screenW() - 1)) / (samples - 1));
      const int y = amplitudeY(pulseBuffer[i], fullScale);
      lcd.drawLine(lastX, lastY, x, y, colorWave());
      lastX = x;
      lastY = y;
    }
  } else {
    lcd.setTextColor(colorText(), colorBg());
    lcd.drawString("Waiting for pulse", 92, 104);
  }

  const float peakPercent = (static_cast<float>(lastPeak) / 2147483648.0f) * 100.0f;
  lcd.setTextColor(colorText(), colorBg());
  lcd.setCursor(4, contentBottom() - 14);
  lcd.printf("Peak %.2f%%FS", peakPercent);
  lcd.setCursor(122, contentBottom() - 14);
  lcd.printf("Window %luus", static_cast<unsigned long>(pulseWindowUs()));
  drawNav();
}

static void acquireScopeFrame() {
  for (int x = 0; x < screenW() && x < SCOPE_COLUMNS; x++) {
    int32_t mn = INT32_MAX;
    int32_t mx = INT32_MIN;
    for (uint16_t i = 0; i < SCOPE_SAMPLES_PER_COLUMN; i++) {
      const int32_t signal = pulseSignalFromRaw(readSample());
      if (signal < mn) mn = signal;
      if (signal > mx) mx = signal;
    }
    scopeMin[x] = mn;
    scopeMax[x] = mx;
  }
}

static void drawScope() {
  acquireScopeFrame();
  drawHeader("Scope");
  drawPlotFrame();

  int32_t fullScale = histogramFullScaleCounts();
  if (fullScale < thresholdCounts() * 2) fullScale = thresholdCounts() * 2;
  const int zeroY = amplitudeY(0, fullScale);
  const int thresholdY = amplitudeY(thresholdCounts(), fullScale);
  lcd.drawFastHLine(0, zeroY, screenW(), colorAxis());
  lcd.drawFastHLine(0, thresholdY, screenW(), colorAxis());

  for (int x = 0; x < screenW() && x < SCOPE_COLUMNS; x++) {
    const int y1 = amplitudeY(scopeMin[x], fullScale);
    const int y2 = amplitudeY(scopeMax[x], fullScale);
    const int top = y1 < y2 ? y1 : y2;
    const int bottom = y1 > y2 ? y1 : y2;
    lcd.drawFastVLine(x, top, bottom - top + 1, colorWave());
  }

  lcd.setTextColor(colorText(), colorBg());
  lcd.setCursor(4, contentBottom() - 14);
  const float usPerDiv = (static_cast<float>(SCOPE_SAMPLES_PER_COLUMN) *
                          static_cast<float>(screenW()) * 1000000.0f) /
                         (10.0f * static_cast<float>(sampleRate()));
  const float yPercentPerDiv = (static_cast<float>(fullScale) / 2147483648.0f) *
                               100.0f / 4.0f;
  if (usPerDiv >= 1000.0f) {
    lcd.printf("X %.1fms/div  Y %.2f%%FS/div", usPerDiv / 1000.0f, yPercentPerDiv);
  } else {
    lcd.printf("X %.0fus/div  Y %.2f%%FS/div", usPerDiv, yPercentPerDiv);
  }
  drawNav();
}

static Rect settingButton(uint8_t row, uint8_t col) {
  return Rect{static_cast<int16_t>(col == 0 ? 212 : 266),
              static_cast<int16_t>(38 + row * 31),
              46,
              26};
}

static Rect settingsPageButton() {
  return Rect{200, 166, 110, 28};
}

static Rect filterPageButton(uint8_t index) {
  const int16_t x = index % 2 == 0 ? 8 : 164;
  const int16_t y = index < 2 ? 36 : 78;
  return Rect{x, y, 148, 38};
}

static Rect trapAdjustButton(uint8_t row, uint8_t col) {
  return Rect{static_cast<int16_t>(col == 0 ? 176 : 242),
              static_cast<int16_t>(124 + row * 28),
              56,
              24};
}

static void drawSettingRow(uint8_t row, const char *label, const char *value) {
  const int y = 45 + row * 31;
  lcd.setTextColor(colorText(), colorBg());
  lcd.drawString(label, 10, y);
  lcd.setTextColor(colorAccent(), colorBg());
  lcd.drawString(value, 118, y);
  drawButton(settingButton(row, 0), "-", false);
  drawButton(settingButton(row, 1), "+", false);
}

static void drawGeneralSettings() {
  drawHeader("Settings");
  lcd.fillRect(0, contentTop(), screenW(), contentHeight(), colorBg());

  char value[32];
  snprintf(value, sizeof(value), "%.0f%% FS", settings.spectrumMaxFraction * 100.0f);
  drawSettingRow(0, "Spectrum zoom", value);

  snprintf(value, sizeof(value), "%.1f%% FS", settings.thresholdFraction * 100.0f);
  drawSettingRow(1, "Noise threshold", value);

  snprintf(value, sizeof(value), "%lu kHz", sampleRate() / 1000);
  drawSettingRow(2, "Sample rate", value);

  snprintf(value, sizeof(value), "%lu us", static_cast<unsigned long>(pulseWindowUs()));
  drawSettingRow(3, "Pulse window", value);

  drawButton(settingsPageButton(), "Filters", false);

  lcd.setTextColor(colorText(), colorBg());
  lcd.setCursor(10, 194);
  lcd.printf("Pulses %lu", static_cast<unsigned long>(pulseCount));
  drawNav();
}

static void drawFilterSettings() {
  drawHeader("Filters");
  lcd.fillRect(0, contentTop(), screenW(), contentHeight(), colorBg());

  drawButton(filterPageButton(0), settings.lowPassEnabled ? "HF ON" : "HF OFF",
             settings.lowPassEnabled);
  drawButton(filterPageButton(1), settings.hysteresisEnabled ? "Hyst ON" : "Hyst OFF",
             settings.hysteresisEnabled);
  drawButton(filterPageButton(2), settings.trapezoidEnabled ? "Trap ON" : "Trap OFF",
             settings.trapezoidEnabled);
  drawButton(filterPageButton(3), "Back", false);

  lcd.setTextColor(colorText(), colorBg());
  lcd.drawString("Rise", 10, 130);
  lcd.setTextColor(colorAccent(), colorBg());
  lcd.setCursor(96, 130);
  lcd.printf("%u", static_cast<unsigned>(settings.trapRiseSamples));
  drawButton(trapAdjustButton(0, 0), "-", false);
  drawButton(trapAdjustButton(0, 1), "+", false);

  lcd.setTextColor(colorText(), colorBg());
  lcd.drawString("Gap", 10, 158);
  lcd.setTextColor(colorAccent(), colorBg());
  lcd.setCursor(96, 158);
  lcd.printf("%u", static_cast<unsigned>(settings.trapGapSamples));
  drawButton(trapAdjustButton(1, 0), "-", false);
  drawButton(trapAdjustButton(1, 1), "+", false);

  lcd.setTextColor(colorText(), colorBg());
  lcd.setCursor(10, 184);
  lcd.printf("LP 1/%u  Hyst 1/%u",
             static_cast<unsigned>(1U << LOWPASS_SHIFT),
             static_cast<unsigned>(HYSTERESIS_LOW_DIV));
  drawNav();
}

static void drawSettings() {
  if (settingsPage == 0) {
    drawGeneralSettings();
  } else {
    drawFilterSettings();
  }
}

static void drawCurrentView() {
  switch (viewMode) {
    case VIEW_SPECTRUM:
      drawSpectrum();
      break;
    case VIEW_PULSE:
      drawPulse();
      break;
    case VIEW_SCOPE:
      drawScope();
      break;
    case VIEW_SETTINGS:
      drawSettings();
      break;
  }
  redrawRequested = false;
}

static bool contains(const Rect &r, int x, int y) {
  return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}

static void adjustSetting(uint8_t row, int8_t delta) {
  switch (row) {
    case 0:
      settings.spectrumMaxFraction += delta * 0.01f;
      if (settings.spectrumMaxFraction < 0.01f) settings.spectrumMaxFraction = 0.01f;
      if (settings.spectrumMaxFraction > 1.0f) settings.spectrumMaxFraction = 1.0f;
      resetMeasurements();
      break;
    case 1:
      settings.thresholdFraction += delta * 0.001f;
      if (settings.thresholdFraction < 0.0025f) settings.thresholdFraction = 0.0025f;
      if (settings.thresholdFraction > 0.25f) settings.thresholdFraction = 0.25f;
      resetDetector();
      break;
    case 2:
      if (delta > 0 && settings.sampleRateIndex < 2) settings.sampleRateIndex++;
      if (delta < 0 && settings.sampleRateIndex > 0) settings.sampleRateIndex--;
      restartI2S();
      return;
    case 3: {
      int32_t samples = settings.pulseSamples + delta * 16;
      if (samples < 32) samples = 32;
      if (samples > MAX_PULSE_SAMPLES) samples = MAX_PULSE_SAMPLES;
      settings.pulseSamples = static_cast<uint16_t>(samples);
      resetDetector();
      break;
    }
    case 4:
      settings.lowPassEnabled = !settings.lowPassEnabled;
      resetMeasurements();
      break;
    case 5:
      settings.hysteresisEnabled = !settings.hysteresisEnabled;
      resetMeasurements();
      break;
    case 6:
      settings.trapezoidEnabled = !settings.trapezoidEnabled;
      resetMeasurements();
      break;
    case 7: {
      int32_t value = settings.trapRiseSamples + delta;
      if (value < TRAP_RISE_MIN) value = TRAP_RISE_MIN;
      if (value > TRAP_RISE_MAX) value = TRAP_RISE_MAX;
      settings.trapRiseSamples = static_cast<uint8_t>(value);
      resetMeasurements();
      break;
    }
    case 8: {
      int32_t value = settings.trapGapSamples + delta;
      if (value < TRAP_GAP_MIN) value = TRAP_GAP_MIN;
      if (value > TRAP_GAP_MAX) value = TRAP_GAP_MAX;
      settings.trapGapSamples = static_cast<uint8_t>(value);
      resetMeasurements();
      break;
    }
  }
  redrawRequested = true;
}

static void handleTouch() {
  int32_t tx = 0;
  int32_t ty = 0;
  const bool touching = lcd.getTouch(&tx, &ty);
  if (!touching) {
    wasTouching = false;
    return;
  }
  if (wasTouching) {
    return;
  }
  wasTouching = true;

  const int x = TOUCH_MIRROR_X ? screenW() - 1 - tx : tx;
  const int y = ty;

  if (viewMode == VIEW_SPECTRUM && contains(spectrumResetButton(), x, y)) {
    resetMeasurements();
    return;
  }
  if (viewMode == VIEW_SPECTRUM && contains(spectrumLogButton(), x, y)) {
    settings.spectrumLogCounts = !settings.spectrumLogCounts;
    redrawRequested = true;
    return;
  }
  if (viewMode == VIEW_PULSE && contains(pulseBeepButton(), x, y)) {
    settings.beepEnabled = !settings.beepEnabled;
    beepUntilMs = 0;
    writeBeeper(0);
    redrawRequested = true;
    return;
  }

  for (uint8_t i = 0; i < 4; i++) {
    if (contains(navButton(i), x, y)) {
      viewMode = static_cast<ViewMode>(i);
      redrawRequested = true;
      return;
    }
  }

  if (viewMode == VIEW_SETTINGS && settingsPage == 0) {
    if (contains(settingsPageButton(), x, y)) {
      settingsPage = 1;
      redrawRequested = true;
      return;
    }
    for (uint8_t row = 0; row < 4; row++) {
      if (contains(settingButton(row, 0), x, y)) {
        adjustSetting(row, -1);
        return;
      }
      if (contains(settingButton(row, 1), x, y)) {
        adjustSetting(row, 1);
        return;
      }
    }
  } else if (viewMode == VIEW_SETTINGS) {
    for (uint8_t index = 0; index < 4; index++) {
      if (contains(filterPageButton(index), x, y)) {
        if (index == 3) {
          settingsPage = 0;
          redrawRequested = true;
        } else {
          adjustSetting(index + 4, 0);
        }
        return;
      }
    }
    for (uint8_t row = 0; row < 2; row++) {
      if (contains(trapAdjustButton(row, 0), x, y)) {
        adjustSetting(row == 0 ? 7 : 8, -1);
        return;
      }
      if (contains(trapAdjustButton(row, 1), x, y)) {
        adjustSetting(row == 0 ? 7 : 8, 1);
        return;
      }
    }
  }
}

static bool setupI2S() {
  Serial.printf("I2S ADC config: CYD, port=%d RX SLAVE, %lu Hz, 32 bit, stereo, APLL on\n",
                static_cast<int>(I2S_PORT), sampleRate());
  Serial.printf("I2S pins: BCK=%d WS=%d DATA=%d spare=%d\n",
                PIN_I2S_BCK, PIN_I2S_WS, PIN_I2S_DATA, PIN_I2S_SPARE);

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_SLAVE);
  chan_cfg.dma_desc_num = 12;
  chan_cfg.dma_frame_num = 128;
  chan_cfg.intr_priority = 1;

  esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &rxHandle);
  if (err != ESP_OK) {
    Serial.printf("i2s_new_channel failed: %d\n", static_cast<int>(err));
  }

  if (err == ESP_OK) {
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate()),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = static_cast<gpio_num_t>(PIN_I2S_BCK),
            .ws = static_cast<gpio_num_t>(PIN_I2S_WS),
            .dout = I2S_GPIO_UNUSED,
            .din = static_cast<gpio_num_t>(PIN_I2S_DATA),
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;

    err = i2s_channel_init_std_mode(rxHandle, &std_cfg);
    if (err != ESP_OK) {
      Serial.printf("i2s_channel_init_std_mode failed: %d\n", static_cast<int>(err));
    }
  }

  if (err == ESP_OK) {
    err = i2s_channel_enable(rxHandle);
    if (err != ESP_OK) {
      Serial.printf("i2s_channel_enable failed: %d\n", static_cast<int>(err));
    }
  }

  i2sInstalled = err == ESP_OK;
  return i2sInstalled;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  setupStatusLed();
  setupBeeper();
  updateStatusLed();

  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(255);
  lcd.fillScreen(colorBg());
  lcd.setTextSize(1);
  lcd.setTextDatum(textdatum_t::top_left);
  resetMeasurements();
  if (!setupI2S()) {
    lcd.fillScreen(C_RED);
    lcd.setTextColor(C_WHITE, C_RED);
    lcd.drawString("I2S begin failed", 90, 110);
    while (true) {
      delay(1000);
    }
  }
  drawCurrentView();
}

void loop() {
  handleTouch();
  updatePulseRate();
  updateStatusLed();
  updateBeeper();

  bool pulseSeen = false;
  if (!paused) {
    for (uint16_t i = 0; i < I2S_FRAMES; i++) {
      processSample(readSample());
      if (pulseReady) {
        pulseReady = false;
        pulseSeen = true;
        updateStatusLed();
      }
    }
  }

  static uint32_t lastRefreshMs = 0;
  static uint32_t lastPulseRedrawMs = 0;
  const uint32_t now = millis();
  if (pulseSeen && viewMode == VIEW_PULSE && now - lastPulseRedrawMs >= 100) {
    redrawRequested = true;
    lastPulseRedrawMs = now;
  }
  const uint16_t refresh = viewMode == VIEW_SCOPE ? 160 : 500;
  if (redrawRequested || now - lastRefreshMs >= refresh) {
    drawCurrentView();
    lastRefreshMs = now;
  }
}
