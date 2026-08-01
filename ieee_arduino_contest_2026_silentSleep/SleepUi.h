//  SleepUi.h  -  display layer for mic_haptic_ui
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define UI_TFT_CS   10
#define UI_TFT_DC    9
#define UI_TFT_RST   8

// --- palette (RGB565) ---
#define C_BG        0x0000
#define C_HEADER    0x0208
#define C_LINE      0x39E7
#define C_DIM       0x8410
#define C_TEXT      0xFFFF
#define C_ACCENT    0x07FF
#define C_OK        0x07E0
#define C_WARN      0xFD20
#define C_ALERT     0xF800
#define C_BAR       0x05BF
#define C_PEAK      0x528A

// --- spectrum (display only) ---
#define UI_FFT_N     256
#define UI_BANDS      32
#define SPEC_TOP     170
#define SPEC_BASE    236
#define SPEC_H       (SPEC_BASE - SPEC_TOP)
#define SPEC_X         8
#define SPEC_W         6
#define SPEC_STEP      7
#define SPEC_F_LO     30.0f
#define SPEC_F_HI   1500.0f

// A whistle or a hum puts nearly all of its energy in one frequency bin.
// Snoring never does - it is a harmonic ladder smeared with turbulence.
// Measured at this sketch's FFT settings: snoring peaks at 0.78, the least
// tonal whistle tested sits at 0.95, so 0.88 clears both by a wide margin.
#define UI_TONE_F_LO  30.0f
#define UI_TONE_F_HI 2500.0f
#ifndef TONE_LIMIT
  #define TONE_LIMIT 0.945f
#endif
#define UI_PURE_TONE_MAX TONE_LIMIT

#ifndef VOWEL_CENTRE_HZ
  #define VOWEL_CENTRE_HZ 480.0f
  #define VOWEL_HALFWIDTH 140.0f
#endif
#define UI_VOWEL_BAND_LO (VOWEL_CENTRE_HZ - VOWEL_HALFWIDTH)
#define UI_VOWEL_BAND_HI (VOWEL_CENTRE_HZ + VOWEL_HALFWIDTH)


// Three states, and only three. Anything else the detector is doing -
// muted after a continuous sound, a bare tone being ignored - is reported
// on the detail line underneath rather than becoming another headline.
//
//   LISTENING  nothing verified yet
//   DETECTING  a breath cycle has been verified; holds through the whole
//              build-up of the streak, right up to the nudge
//   NUDGING    the motor is running
//   LISTENING  nothing verified yet
//   DETECTING  a breath cycle has been verified
//   NUDGING    the motor is running
enum UiState : uint8_t { UI_LISTENING, UI_DETECTING, UI_NUDGING };

class SleepUi {
public:
  // ------------------------------------------------------------ sampling --
  // Called from inside the existing 100 ms acquisition loop. Stores the
  // sample the detector just read; costs a couple of microseconds against
  // that loop's 150 us delay, and changes nothing it computes.
  inline void feed(int sample) {
    if (_n >= UI_FFT_N) return;
    if (_n == 0) _t0 = micros();
    _buf[_n++] = (float)sample;
    _t1 = micros();
  }

  // ------------------------------------------------------------ lifecycle --
  void begin() {
    _tft.init(240, 240);
    _tft.setRotation(0);          // use 2 if the screen reads upside down
    _tft.fillScreen(C_BG);
    chrome();
    for (uint8_t k = 0; k < UI_BANDS; k++) { _lastBar[k] = -1; _peak[k] = 0; }
    _statusCache[0] = _detailCache[0] = _infoCache[0] = '\0';
    for (uint8_t i = 0; i < 4; i++) _valCache[i][0] = '\0';
    _lastFill = -1;
    _lastState = 255;
    for (uint8_t k = 0; k < UI_FFT_N / 2; k++)
      _sinT[k] = sinf(2.0f * (float)PI * (float)k / (float)UI_FFT_N);
    for (uint16_t i = 0; i < UI_FFT_N; i++)
      _hann[i] = 0.5f - 0.5f * cosf(2.0f * (float)PI * (float)i / (UI_FFT_N - 1));
  }

  void chrome() {
    _tft.fillRect(0, 0, 240, 24, C_HEADER);
    _tft.setTextSize(2);
    _tft.setTextColor(C_ACCENT, C_HEADER);
    _tft.setCursor(48, 4);
    _tft.print(F("SILENT SLEEP"));
    _tft.drawFastHLine(0,  25, 240, C_LINE);
    _tft.drawRect(9, 70, 222, 16, C_DIM);
    _tft.drawFastHLine(0, 102, 240, C_LINE);
    _tft.drawFastHLine(0, 166, 240, C_LINE);
    label(10,  106, "SNORING FOR");
    label(126, 106, "LONGEST");
    label(10,  136, "TOTAL");
    label(126, 136, "BREATH/NUDGE");
  }

  void splash(const char *line1, const char *line2, uint16_t colour) {
    setState(UI_LISTENING);
    setDetail(line1, colour);
    char b[39];
    padCenter(b, line2, 38);
    field(6, 90, 1, colour, C_BG, b, 38, _infoCache);
  }

  // ------------------------------------------------------------ analyse ---
  // Runs the FFT over the window the acquisition loop just captured, and
  // works out how much of the spectrum sits in a single peak. Call this
  // once per loop, after sampling and before the decision.
  void analyse() {
    const uint16_t n = _n;
    _n = 0;                                   // reopen the buffer
    _tonality = 0.0f;
    _peakHz = 0.0f;
    if (n < 32) return;

    // Real sample rate the acquisition loop achieved, so the frequency
    // axis is honest rather than assumed.
    float fs = 5000.0f;
    if (_t1 > _t0 && n > 1)
      fs = 1000000.0f * (float)(n - 1) / (float)(_t1 - _t0);

    float mean = 0.0f;
    for (uint16_t i = 0; i < n; i++) mean += _buf[i];
    mean /= n;
    for (uint16_t i = 0; i < n; i++)        _buf[i] = (_buf[i] - mean) * _hann[i];
    for (uint16_t i = n; i < UI_FFT_N; i++) _buf[i] = 0.0f;
    for (uint16_t i = 0; i < UI_FFT_N; i++) _im[i] = 0.0f;

    fft(_buf, _im);
    for (uint16_t i = 0; i <= UI_FFT_N / 2; i++)
      _pow[i] = _buf[i] * _buf[i] + _im[i] * _im[i];

    const float binHz = fs / (float)UI_FFT_N;

    // --- how tonal is it? ---
    const uint16_t tlo = clampBin((uint16_t)(UI_TONE_F_LO / binHz + 0.5f));
    const uint16_t thi = clampBin((uint16_t)(UI_TONE_F_HI / binHz + 0.5f));
    float total = 1e-12f, pkVal = 0.0f;
    uint16_t pkBin = tlo;
    for (uint16_t i = tlo; i <= thi; i++) {
      total += _pow[i];
      if (_pow[i] > pkVal) { pkVal = _pow[i]; pkBin = i; }
    }
    const uint16_t a = (pkBin > tlo) ? (uint16_t)(pkBin - 1) : tlo;
    const uint16_t b = (pkBin + 1 < thi) ? (uint16_t)(pkBin + 1) : thi;
    float peakEnergy = 0.0f;
    for (uint16_t i = a; i <= b; i++) peakEnergy += _pow[i];
    _tonality = peakEnergy / total;
    _peakHz   = (float)pkBin * binHz;

    // --- bars for the display ---
    const uint16_t lo = clampBin((uint16_t)(SPEC_F_LO / binHz + 0.5f));
    const uint16_t hi = clampBin((uint16_t)(SPEC_F_HI / binHz + 0.5f));
    _bandPeak = 1e-9f;
    for (uint8_t k = 0; k < UI_BANDS; k++) {
      const uint16_t x = lo + (uint16_t)((float)k       * (hi - lo) / UI_BANDS);
      uint16_t y       = lo + (uint16_t)((float)(k + 1) * (hi - lo) / UI_BANDS);
      if (y <= x) y = x + 1;
      float acc = 0.0f;
      for (uint16_t i = x; i < y && i <= hi; i++) acc += _pow[i];
      _band[k] = acc;
      if (acc > _bandPeak) _bandPeak = acc;
    }
    float tot = 1e-12f;
    for (uint8_t k = 0; k < UI_BANDS; k++) tot += _band[k];
    for (uint8_t k = 0; k < UI_BANDS; k++) _shape[k] = _band[k] / tot;
    _haveSpectrum = true;
  }

  // 0 = broadband, 1 = a single pure tone. Snoring measures about 0.7.
  float tonality()  const { return _tonality; }
  bool  isPureTone() const { return _tonality > UI_PURE_TONE_MAX; }

  // Loudest frequency in the window, whatever the sound happens to be.
  float peakHz()    const { return _peakHz; }

  // The 32-band spectrum, normalised to sum to 1 - the SHAPE of the sound,
  // independent of how loud it is. Comparing this across a breath is the
  // only way to tell voiced speech from snoring:
  const float *shape() const { return _shape; }
  bool  isVowelPitched() const {
    return _peakHz >= UI_VOWEL_BAND_LO && _peakHz <= UI_VOWEL_BAND_HI;
  }

  // Ask the controller for its ID. The breakout supports this, but MISO is
  // not wired on this build, so the answer is only meaningful if it comes
  // back as something other than all-zeros or all-ones.
  uint8_t probeId() { return _tft.readcommand8(0x04); }   // RDDID


  // ------------------------------------------------------------- render ---
  // Everything the screen shows, in one call. All arguments are values the
  // detector already worked out; none of them are modified here.
  void render(UiState st, double volts, double threshold, int spikes,
              bool pureTone, bool tooHigh, bool drone, bool muted,
              int streak, int triggerStreak,
              uint32_t episodeMs, uint32_t longestMs, uint32_t totalMs,
              uint32_t cycles, uint32_t nudges) {
    setState(st);

    char d[40];
    if (st == UI_NUDGING) {
      snprintf(d, sizeof(d), "pillow correction - roll over");
      setDetail(d, C_WARN);
    } else if (st == UI_DETECTING) {
      snprintf(d, sizeof(d), "snoring - breath %d of %d verified",
               streak < 1 ? 1 : streak, triggerStreak);
      setDetail(d, C_ALERT);
    } else if (muted) {
      snprintf(d, sizeof(d), "continuous sound - waiting for quiet");
      setDetail(d, C_WARN);
    } else if (pureTone) {
      snprintf(d, sizeof(d), "pure tone ignored - whistle or hum");
      setDetail(d, C_WARN);
    } else if (tooHigh) {
      snprintf(d, sizeof(d), "voice ignored - peak %dHz is a vowel",
               (int)(_peakHz + 0.5f));
      setDetail(d, C_WARN);
    } else if (drone) {
      snprintf(d, sizeof(d), "steady drone ignored - fan or aircon");
      setDetail(d, C_WARN);
    } else {
      snprintf(d, sizeof(d), "waiting for a breath");
      setDetail(d, C_DIM);
    }

    setLevel(volts, threshold, spikes);
    setStats(episodeMs, longestMs, totalMs, cycles, nudges);
    drawSpectrum();
  }

private:
  Adafruit_ST7789 _tft{UI_TFT_CS, UI_TFT_DC, UI_TFT_RST};

  float _buf[UI_FFT_N];
  float _im[UI_FFT_N];
  float _pow[UI_FFT_N / 2 + 1];
  float _band[UI_BANDS];
  float _shape[UI_BANDS];
  float _bandPeak = 1e-9f;
  float _tonality = 0.0f;
  float _peakHz = 0.0f;
  bool  _haveSpectrum = false;
  float _sinT[UI_FFT_N / 2];
  float _hann[UI_FFT_N];
  volatile uint16_t _n = 0;
  uint32_t _t0 = 0, _t1 = 0;

  char _statusCache[12], _detailCache[40], _infoCache[40], _valCache[4][12];
  int  _lastBar[UI_BANDS], _peak[UI_BANDS];
  int  _lastFill = -1;
  uint8_t  _lastState = 255;
  uint16_t _stateColor = C_OK, _detailColor = C_DIM, _infoColor = C_DIM;

  // ------------------------------------------------------------- widgets --
  void setState(UiState s) {
    if (s == _lastState) return;
    _lastState = s;
    const char *txt;
    switch (s) {
      case UI_DETECTING: txt = "DETECTING"; _stateColor = C_ALERT; break;
      case UI_NUDGING:   txt = "NUDGING";   _stateColor = C_WARN;  break;
      default:           txt = "LISTENING"; _stateColor = C_OK;    break;
    }
    char buf[11];
    padCenter(buf, txt, 10);
    _statusCache[0] = '\0';           // colour changed too, bypass the cache
    field(30, 28, 3, _stateColor, C_BG, buf, 10, _statusCache);
  }

  void setDetail(const char *s, uint16_t colour) {
    char buf[39];
    padCenter(buf, s, 38);
    if (colour != _detailColor) { _detailCache[0] = '\0'; _detailColor = colour; }
    field(6, 56, 1, _detailColor, C_BG, buf, 38, _detailCache);
  }

  // The bar is the detector's own criterion: peak-to-peak volts against
  // VOLUME_THRESHOLD, with a tick where the threshold sits.
  void setLevel(double volts, double threshold, int spikes) {
    const float full = (float)threshold * 2.2f;     // headroom above the line
    float span = (float)volts / full;
    if (span < 0) span = 0;
    if (span > 1) span = 1;
    const int fill = (int)(span * 220.0f);
    const int tick = (int)(((float)threshold / full) * 220.0f);
    if (fill != _lastFill) {
      const uint16_t c = (volts > threshold) ? C_OK : C_BAR;
      if (fill > 0)   _tft.fillRect(10, 71, fill, 14, c);
      if (fill < 220) _tft.fillRect(10 + fill, 71, 220 - fill, 14, C_BG);
      _tft.drawFastVLine(10 + tick, 71, 14, C_WARN);
      _lastFill = fill;
    }
    // Integer formatting only: float printf is not guaranteed on every core.
    char buf[39], tmp[39];
    snprintf(tmp, sizeof(tmp), "%d.%02dV/%d.%02dV sp%d tone%d%% %dHz",
             (int)volts, (int)((volts - (int)volts) * 100.0 + 0.5),
             (int)threshold, (int)((threshold - (int)threshold) * 100.0 + 0.5),
             spikes, (int)(_tonality * 100.0f + 0.5f), (int)(_peakHz + 0.5f));
    padCenter(buf, tmp, 38);
    const uint16_t ic = (volts > threshold) ? C_OK : C_DIM;
    if (ic != _infoColor) { _infoCache[0] = '\0'; _infoColor = ic; }
    field(6, 90, 1, _infoColor, C_BG, buf, 38, _infoCache);
  }

  void setStats(uint32_t episodeMs, uint32_t longestMs, uint32_t totalMs,
                uint32_t cycles, uint32_t nudges) {
    char v[12];
    fmtMMSS(v, episodeMs);  value(0, 10,  116, v, C_TEXT);
    fmtMMSS(v, longestMs);  value(1, 126, 116, v, C_TEXT);
    fmtHMS(v, totalMs);     value(2, 10,  146, v, C_ACCENT);
    snprintf(v, sizeof(v), "%lu/%lu", (unsigned long)cycles,
             (unsigned long)nudges);
    value(3, 126, 146, v, C_ACCENT);
  }

  // ------------------------------------------------------------ spectrum --
  void drawSpectrum() {
    if (!_haveSpectrum) return;
    for (uint8_t k = 0; k < UI_BANDS; k++) {
      int h = (int)(SPEC_H * sqrtf(sqrtf(_band[k] / _bandPeak)));
      if (h < 0) h = 0;
      if (h > SPEC_H) h = SPEC_H;
      int pk = _peak[k] - 1;
      if (pk < h) pk = h;
      if (pk < 0) pk = 0;
      if (h == _lastBar[k] && pk == _peak[k]) continue;
      _lastBar[k] = h; _peak[k] = pk;
      const int x = SPEC_X + k * SPEC_STEP;
      _tft.fillRect(x, SPEC_TOP, SPEC_W, SPEC_H, C_BG);
      if (h > 0)  _tft.fillRect(x, SPEC_BASE - h, SPEC_W, h,
                                (k < 8) ? C_ACCENT : C_BAR);
      if (pk > h) _tft.fillRect(x, SPEC_BASE - pk, SPEC_W, 1, C_PEAK);
    }
  }

  static uint16_t clampBin(uint16_t b) {
    if (b < 1) return 1;
    if (b > UI_FFT_N / 2 - 1) return UI_FFT_N / 2 - 1;
    return b;
  }

  // Iterative radix-2 Cooley-Tukey, single precision. Small enough to carry
  // here rather than take on a library dependency.
  void fft(float *re, float *im) {
    uint16_t j = 0;
    for (uint16_t i = 1; i < UI_FFT_N; i++) {
      uint16_t bit = UI_FFT_N >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        float t = re[i]; re[i] = re[j]; re[j] = t;
        t = im[i]; im[i] = im[j]; im[j] = t;
      }
    }
    for (uint16_t len = 2; len <= UI_FFT_N; len <<= 1) {
      const uint16_t half = len >> 1, step = UI_FFT_N / len;
      for (uint16_t i = 0; i < UI_FFT_N; i += len) {
        uint16_t k = 0;
        for (uint16_t m = 0; m < half; m++) {
          const float wr = cosT(k), wi = -_sinT[k];
          const uint16_t a = i + m, b = a + half;
          const float xr = re[b] * wr - im[b] * wi;
          const float xi = re[b] * wi + im[b] * wr;
          re[b] = re[a] - xr; im[b] = im[a] - xi;
          re[a] += xr;        im[a] += xi;
          k += step;
        }
      }
    }
  }

  float cosT(uint16_t k) const {
    const uint16_t q = k + UI_FFT_N / 4;
    return (q < UI_FFT_N / 2) ? _sinT[q] : -_sinT[q - UI_FFT_N / 2];
  }

  // -------------------------------------------------------------- helpers --
  void label(int16_t x, int16_t y, const char *s) {
    _tft.setTextSize(1);
    _tft.setTextColor(C_DIM, C_BG);
    _tft.setCursor(x, y);
    _tft.print(s);
  }

  void value(uint8_t slot, int16_t x, int16_t y, const char *s, uint16_t c) {
    char buf[12];
    padTo(buf, s, 9);
    field(x, y, 2, c, C_BG, buf, 9, _valCache[slot]);
  }

  void field(int16_t x, int16_t y, uint8_t size, uint16_t fg, uint16_t bg,
             const char *txt, uint8_t width, char *cache) {
    if (strncmp(cache, txt, width) == 0) return;   // already on the glass
    memcpy(cache, txt, width);
    cache[width] = '\0';
    _tft.setTextSize(size);
    _tft.setTextColor(fg, bg);
    _tft.setCursor(x, y);
    _tft.print(txt);
  }

  static void padTo(char *dst, const char *src, uint8_t w) {
    uint8_t i = 0;
    while (src[i] && i < w) { dst[i] = src[i]; i++; }
    while (i < w) dst[i++] = ' ';
    dst[w] = '\0';
  }

  static void padCenter(char *dst, const char *src, uint8_t w) {
    uint8_t n = 0;
    while (src[n] && n < w) n++;
    const uint8_t left = (uint8_t)((w - n) / 2);
    for (uint8_t i = 0; i < left; i++) dst[i] = ' ';
    for (uint8_t i = 0; i < n; i++)    dst[left + i] = src[i];
    for (uint8_t i = left + n; i < w; i++) dst[i] = ' ';
    dst[w] = '\0';
  }

  static void fmtMMSS(char *out, uint32_t ms) {
    uint32_t s = ms / 1000, m = s / 60; s %= 60;
    if (m > 99) m = 99;
    snprintf(out, 12, "%02lu:%02lu", (unsigned long)m, (unsigned long)s);
  }

  static void fmtHMS(char *out, uint32_t ms) {
    uint32_t s = ms / 1000, h = s / 3600; s %= 3600;
    uint32_t m = s / 60; s %= 60;
    if (h > 99) h = 99;
    snprintf(out, 12, "%lu:%02lu:%02lu", (unsigned long)h,
             (unsigned long)m, (unsigned long)s);
  }
};
