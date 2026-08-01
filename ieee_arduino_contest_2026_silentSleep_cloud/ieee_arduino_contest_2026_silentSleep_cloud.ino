
#include "thingProperties.h"
#include "Arduino_LED_Matrix.h"
#include <Adafruit_DRV2605.h>

#define SPEECH_REJECTION  50

#define VOWEL_CENTRE_HZ   480.0f
#define VOWEL_HALFWIDTH   (2.8f * (float)SPEECH_REJECTION)
#if SPEECH_REJECTION == 0
  #define TONE_LIMIT      1.00f
#else
  #define TONE_LIMIT      (0.80f - 0.0045f * (float)SPEECH_REJECTION)
#endif

#include "SleepUi.h"                                              // UI
Adafruit_DRV2605 drv;
int drvFlag = 0;
const int MIC_PIN = A1;             // Sampling pin: A1
const int SAMPLE_WINDOW_MS = 100;

// CALIBRATED HARDWARE PARAMETERS
const double VOLUME_THRESHOLD = 0.55; // Secure wall against background video hiss


const uint8_t  NUDGE_STRENGTH = 0x7F;   // 0..0x7F
const uint16_t NUDGE_MS       = 1500;   // how long the motor actually runs

// THE BIOLOGICAL LAWS (In Milliseconds)
const unsigned long MIN_SNORE_MS = 600;
const unsigned long MAX_SNORE_MS = 3500;
const unsigned long BREATH_GAP_MS = 600;

// State Machine Variables
unsigned long soundStartTime = 0;
unsigned long lastSoundEndTime = 0;
bool soundActiveNow = false;
bool isCurrentlySnoring = false;
bool systemMuteActive = false;

int validSnoreCycles = 0;
const int TRIGGER_STREAK = 3;

// ---------------------------------------------------------------- UI ------
// Display-only state. Nothing here is read by the detection logic.
SleepUi  ui;
double   uiVolts        = 0.0;    // last peak-to-peak reading, for the bar
int      uiSpikes       = 0;      // last density count, for the readout
int      uiPrevCycles   = 0;      // to notice the streak counter going up
uint32_t uiEpisodeStart = 0;      // when the current snoring run began
uint32_t uiLastEvidence = 0;      // last time the detector saw a breath
uint32_t uiTotalSnoreMs = 0;
uint32_t uiLongestMs    = 0;
uint32_t uiCycleCount   = 0;      // breath cycles verified this session
uint32_t uiNudgeCount   = 0;
bool     uiEpisodeOn    = false;
const uint32_t UI_EPISODE_END_MS = 15000;   // quiet this long -> run is over
uint32_t uiLastPrint    = 0;      // throttle for the per-loop serial line

const float BREATH_TONE_MAX = 0.38f;   // set >1.0 to measure without gating
float    toneSum = 0.0f;
float    toneMin = 1.0f, toneMax = 0.0f;
uint16_t toneN   = 0;

bool     sawLowContent  = false;  // has a snore fundamental been heard?
uint32_t lastLowMs = 0;
const float    SNORE_LOW_MIN_HZ  = 100.0f;
const float    SNORE_LOW_MAX_HZ  = 400.0f;
const uint32_t SNORE_LOW_HOLD_MS = 30000;


// True when the previous frame was carried over by the one-frame bridge.
bool bridgedLast = false;

// Rolling peak-to-peak history, for the breathing-envelope test. Snoring
// rises and falls; a fan does not. ~2 s of history at the ~130 ms loop.
const uint8_t ENV_WINDOW = 15;
const float   ENV_MIN    = 0.50f;   // required (max-min)/max
double   envHist[ENV_WINDOW];
uint8_t  envCount = 0, envIdx = 0;
float    envSpread = 0.0f;          // last computed value, for the readout

ArduinoLEDMatrix matrix;

// 8 rows and 12 columns
uint8_t cloudFrame[8][12] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
  {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
  {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
  {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

int8_t previousCloudState = -1;

bool envelopeVaries(double volts) {
  envHist[envIdx] = volts;
  envIdx = (uint8_t)((envIdx + 1) % ENV_WINDOW);
  if (envCount < ENV_WINDOW) envCount++;

  double mn = envHist[0], mx = envHist[0];
  for (uint8_t i = 1; i < envCount; i++) {
    if (envHist[i] < mn) mn = envHist[i];
    if (envHist[i] > mx) mx = envHist[i];
  }
  envSpread = (mx > 0.0) ? (float)((mx - mn) / mx) : 0.0f;
  return envSpread >= ENV_MIN;
}


void reportHardware();

void setup() {

  Serial.begin(9600);
  matrix.begin();
  matrix.clear();
  delay(5000);

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  snoreDetected = false;

  ui.begin();                                                     // UI
  ui.splash("starting up", "SilentSleep", C_DIM);                 // UI

 

  reportHardware();                                               // UI

  Serial.println("Starting DRV2605L Test...");

  if (!drv.begin()) {
    Serial.println("ERROR: DRV2605L NOT FOUND");
    drvFlag = 0;
    Serial.println("HAPTIC : NOT DETECTED  (DRV2605L did not answer on I2C)");
    ui.splash("DRV2605L NOT FOUND", "check I2C wiring", C_ALERT); // UI
    while (1)
    {
      Serial.println("ERROR: DRV2605L NOT FOUND");
      delay(5000);
    }
  }
  drvFlag = 1;
  Serial.println("HAPTIC : detected      (DRV2605L at 0x5A)");
  Serial.println("DRV2605L FOUND");
  ui.splash("DRV2605L found", "warming up", C_OK);                // UI
  delay(5000);
  drv.selectLibrary(1);
  drv.useERM();          // Use for coin vibration motors

  drv.writeRegister8(DRV2605_REG_CLAMPV, 0xA0);

  Serial.println("Ready");
  pinMode(MIC_PIN, INPUT);

  delay(1000);
  Serial.println("\n=====================================================");
  Serial.println("=== SILENT SLEEP: DYNAMIC CALIBRATION CORE ACTIVE ===");
  Serial.println("=====================================================\n");
}

void loop() {
  ArduinoCloud.update();
  bool cloudOnline = ArduinoCloud.connected();

  // Redraw only when the Cloud connection changes
  if ((int8_t)cloudOnline != previousCloudState) {
    previousCloudState = cloudOnline;

    if (cloudOnline) {
      matrix.renderBitmap(cloudFrame, 8, 12);
      Serial.println("Cloud online - showing cloud");
    } else {
      matrix.clear();
      Serial.println("Cloud offline - matrix cleared");
    }
  }
  bool cloudConnected = ArduinoCloud.connected() != 0;
  ui.showCloud(cloudConnected);
  uiRender();                                                     // UI

  unsigned long startMillis = millis();
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  long sampleSum = 0;
  long totalReadings = 0;

  // 1. Gather raw data and dynamically calculate the live center point
  while (millis() - startMillis < SAMPLE_WINDOW_MS) {
    int sample = analogRead(MIC_PIN);
    if (sample < 1024) {
      sampleSum += sample;
      totalReadings++;
      if (sample > signalMax) signalMax = sample;
      if (sample < signalMin) signalMin = sample;
      ui.feed(sample);                                            // UI
    }
    delayMicroseconds(150); // Steady execution delay
  }

  // Calculate stats using the exact readings gathered during this window
  unsigned int peakToPeak = signalMax - signalMin;
  double volts = ((double)peakToPeak * 5.0) / 1024.0;
  unsigned long currentMillis = millis();

  // DYNAMIC FILTER ENGINE: Re-verify the raw density against the live center point
  long dynamicCenter = (totalReadings > 0) ? (sampleSum / totalReadings) : 512;

  // Re-read a fast sample subset to calculate instant density
  int fastLoudSpikes = 0;
  for(int i = 0; i < 40; i++) {
    int fastSample = analogRead(MIC_PIN);
    if (abs(fastSample - dynamicCenter) > 35) {
      fastLoudSpikes++;
    }
    delayMicroseconds(50);
  }

  uiVolts = volts;                                                // UI
  uiSpikes = fastLoudSpikes;                                      // UI

  ui.analyse();                                                   // UI
  const bool pureTone = ui.isPureTone();                          // UI
  const bool vowel    = ui.isVowelPitched();                      // UI
  reportSound(volts, fastLoudSpikes);                             // UI

  const bool envVaries = envelopeVaries(volts);                   // UI


  bool isDenseRespiratoryWave = (volts > VOLUME_THRESHOLD && fastLoudSpikes >= 5
                                 && !pureTone && !vowel && envVaries);


  const bool stillLoud = (volts > VOLUME_THRESHOLD && fastLoudSpikes >= 5);


  if (stillLoud) {
    const float tn = ui.tonality();
    toneSum += tn; toneN++;
    if (tn < toneMin) toneMin = tn;
    if (tn > toneMax) toneMax = tn;
  }

  if (stillLoud &&
      ui.peakHz() >= SNORE_LOW_MIN_HZ && ui.peakHz() <= SNORE_LOW_MAX_HZ) {
    sawLowContent = true;
    lastLowMs = currentMillis;
  }


  if (!isDenseRespiratoryWave && stillLoud && soundActiveNow && !bridgedLast) {
    bridgedLast = true;                                                   // UI
    isDenseRespiratoryWave = true;                                        // UI
  } else if (isDenseRespiratoryWave) {
    bridgedLast = false;                                                  // UI
  }


  // 2. Real-Time Signal Processing Pipeline
  if (isDenseRespiratoryWave) {
    if (!soundActiveNow) {
      soundStartTime = currentMillis;
      soundActiveNow = true;
      toneSum = 0.0f; toneN = 0; toneMin = 1.0f; toneMax = 0.0f;

      // Validate rhythm gap boundaries
      unsigned long silentInterval = currentMillis - lastSoundEndTime;
      if (lastSoundEndTime > 0 && (silentInterval < 300 || silentInterval > 5000)) {
        if (validSnoreCycles > 0) {
          Serial.println("[RESET] Spacing irregular (Talking/Transient detected). Clear streak.");
          validSnoreCycles = 0;
          systemMuteActive = true;
          return;
        }
      }
    }

    unsigned long currentDuration = currentMillis - soundStartTime;

    // UPPER TIME GATE LOCKOUT
    if (currentDuration > MAX_SNORE_MS) {
      if (!systemMuteActive) {
        Serial.println("\n[TIME GATE LOCKOUT] Continuous wave detected. Muting background noise/talk...");
        systemMuteActive = true;
        isCurrentlySnoring = false;
        validSnoreCycles = 0;
      }
      return;
    }

    // --- LIVE CADENCE BREATH VALIDATION ---
    if (!systemMuteActive && currentDuration >= MIN_SNORE_MS) {
      if (!isCurrentlySnoring) {
        // Snoring always drops to its low fundamental somewhere in the
        // breath; whistling never does. Measured peaks within one breath:
        //
        //   snore      964 -> 300, 236, 278, 300
        //   snore      942 -> 257, 900, 171, 257, 278
        //   whistle    1478, 1585, 1071, 1114
        //   whistle    1242, 1264, 1157, 1178
        //

        const float meanTone = (toneN > 0) ? (toneSum / (float)toneN) : 0.0f;
        Serial.print("[BREATH] mean ");
        Serial.print((int)(meanTone * 100.0f + 0.5f));
        Serial.print("%  min ");
        Serial.print((int)(toneMin * 100.0f + 0.5f));
        Serial.print("%  max ");
        Serial.print((int)(toneMax * 100.0f + 0.5f));
        Serial.print("%  frames ");
        Serial.print(toneN);
        Serial.print("  limit ");
        Serial.print((int)(BREATH_TONE_MAX * 100.0f + 0.5f));
        Serial.println("%");

        if (meanTone > BREATH_TONE_MAX) {
          Serial.println("[SKIP] breath too tonal - REJECTED BY THE RULE UNDER TEST");
          isCurrentlySnoring = true;
          return;
        }

        const bool recentlyLow = sawLowContent &&
              (currentMillis - lastLowMs) < SNORE_LOW_HOLD_MS;
        if (!recentlyLow) {
          Serial.println("[SKIP] no snore fundamental heard - whistle?");
          isCurrentlySnoring = true;   // consume it; do not count it
          return;
        }

        isCurrentlySnoring = true;
        validSnoreCycles++;
        Serial.print("[MATCH] Valid Breath Cycle Verified. Streak: ");
        Serial.print(validSnoreCycles);
        Serial.print("/");
        Serial.println(TRIGGER_STREAK);

        // 3. Actuate Closed-Loop Virtual Correction Nudge
        if (validSnoreCycles >= TRIGGER_STREAK) {
          Serial.println("\n>>> SNORE CONFIRMED <<<");

          // Send the event to Arduino Cloud
          snoreDetected = true;
          ArduinoCloud.update();
        
          validSnoreCycles = 0;
          isCurrentlySnoring = false;
          soundActiveNow = false;
          lastSoundEndTime = millis();
        
          Serial.println("Vibrating...");
        
          if (drvFlag == 1) {
            Serial.println("DRV2605L FOUND");
          } else {
            Serial.println("DRV2605L NOT FOUND");
          }
        
          drv.setMode(DRV2605_MODE_REALTIME);
          drv.setRealtimeValue(NUDGE_STRENGTH);
        
          uiOnNudge();
        
          // Motor running; Cloud connection also remains active
          cloudDelay(NUDGE_MS);
        
          drv.setRealtimeValue(0);
          drv.setMode(DRV2605_MODE_INTTRIG);
        
          // Continue servicing Cloud during the quiet period
          cloudDelay(3000 - NUDGE_MS);
        
          // Reset so a future snore can generate another notification
          snoreDetected = false;
          ArduinoCloud.update();
        
          return;
        }
      }
    }

  } else {
    soundActiveNow = false;

    // Reset loop constraints when the room falls completely silent
    if (currentMillis - soundStartTime > BREATH_GAP_MS) {
      if (systemMuteActive) {
        Serial.println("--- [SYSTEM RESET] Room went quiet. Clear lockouts. ---");
        systemMuteActive = false;
        validSnoreCycles = 0;
      }

      if (isCurrentlySnoring) {
        Serial.println("--- [BREATH PAUSE] ---");
        isCurrentlySnoring = false;
        lastSoundEndTime = currentMillis;
      }
    }
  }
}

void cloudDelay(unsigned long waitTime) {
  unsigned long startTime = millis();

  while (millis() - startTime < waitTime) {
    ArduinoCloud.update();
    delay(20);
  }
}

// ============================================================== UI =========
// Boot self test. The microphone and the display are checked here; the
// haptic driver reports itself where drv.begin() already runs.
void reportHardware() {
  Serial.println();
  Serial.println("=============== HARDWARE CHECK ===============");


  long sum = 0;
  int lo = 1023, hi = 0;
  const int N = 400;
  for (int i = 0; i < N; i++) {
    int v = analogRead(MIC_PIN);
    sum += v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
    delayMicroseconds(200);
  }
  const int dc  = (int)(sum / N);
  const int p2p = hi - lo;
  const double dcVolts = dc * 5.0 / 1024.0;

  Serial.print("MIC    : ");
  if (dc > 120 && dc < 700) Serial.print("detected      ");
  else                      Serial.print("NOT DETECTED  ");
  Serial.print("(A1 bias ");
  Serial.print(dc);
  Serial.print(" counts = ");
  Serial.print(dcVolts, 2);
  Serial.print(" V, expect ~256 / 1.25 V; noise ");
  Serial.print(p2p);
  Serial.println(" counts)");
  if (dc <= 120 || dc >= 700)
    Serial.println("         check MAX9814 OUT is on A1 and VDD is on 5V");

  // --- display ----------------------------------------------------------
  // The controller can report an ID, but MISO is not wired on this build,
  // so the reply is only meaningful if it is neither all-zeros nor all-ones.
  const uint8_t id = ui.probeId();
  Serial.print("TFT    : ");
  if (id != 0x00 && id != 0xFF) {
    Serial.print("detected      (ST7789 answered 0x");
    Serial.print(id, HEX);
    Serial.println(")");
  } else {
    Serial.println("no read-back  (MISO not wired - confirm the boot text is on screen)");
  }
  Serial.println("==============================================");
  Serial.println();
}



void uiOnNudge() {
  uiNudgeCount++;
  uiLastEvidence = millis();
  ui.render(UI_NUDGING, uiVolts, VOLUME_THRESHOLD, uiSpikes,
            false, false, false, false,
            TRIGGER_STREAK, TRIGGER_STREAK,
            uiEpisodeOn ? (millis() - uiEpisodeStart) : 0,
            uiLongestMs, uiTotalSnoreMs, uiCycleCount, uiNudgeCount);
}

// Dominant frequency of whatever is being heard, snoring or not. Chatty
// while there is sound, quiet when the room is, so the [MATCH] lines from
// the detector stay findable in the log.
void reportSound(double volts, int spikes) {
  const uint32_t now = millis();
  const bool audible = volts > 0.05;
  if (!audible && (now - uiLastPrint) < 2000) return;
  uiLastPrint = now;

  Serial.print("[SOUND] ");
  Serial.print((int)(ui.peakHz() + 0.5f));
  Serial.print(" Hz   ");
  Serial.print(volts, 2);
  Serial.print(" V   spikes ");
  Serial.print(spikes);
  Serial.print("   tone ");
  Serial.print((int)(ui.tonality() * 100.0f + 0.5f));
  Serial.print("%   env ");
  Serial.print((int)(envSpread * 100.0f + 0.5f));
  Serial.print("%");
  // Only explain a rejection for a frame that was actually a candidate.
  // Printing "ignored" on frames that were never loud enough to count made
  // a silent room look like the detector was throwing sound away.
  if (volts > VOLUME_THRESHOLD && spikes >= 5) {
    if (ui.isPureTone())          Serial.print("  <- pure tone, ignored");
    else if (ui.isVowelPitched()) Serial.print("  <- vowel band, ignored");
    else if (envSpread < ENV_MIN) Serial.print("  <- steady drone, ignored");
  }
  Serial.println();
}

void uiRender() {
  const uint32_t now = millis();


  if (validSnoreCycles > uiPrevCycles) {
    uiCycleCount++;
    uiLastEvidence = now;
    if (!uiEpisodeOn) { uiEpisodeOn = true; uiEpisodeStart = now; }
  }
  uiPrevCycles = validSnoreCycles;
  if (isCurrentlySnoring) uiLastEvidence = now;

  // A run of snoring ends once nothing has been verified for a while.
  if (uiEpisodeOn && (now - uiLastEvidence) > UI_EPISODE_END_MS) {
    const uint32_t dur = uiLastEvidence - uiEpisodeStart;
    uiTotalSnoreMs += dur;
    if (dur > uiLongestMs) uiLongestMs = dur;
    uiEpisodeOn = false;
  }


  UiState st = UI_LISTENING;
  if (validSnoreCycles > 0 || isCurrentlySnoring) st = UI_DETECTING;

  const uint32_t live = uiEpisodeOn ? (uiLastEvidence - uiEpisodeStart) : 0;
  ui.render(st, uiVolts, VOLUME_THRESHOLD, uiSpikes,
            ui.isPureTone(), ui.isVowelPitched(),
            envSpread < ENV_MIN, systemMuteActive,
            validSnoreCycles, TRIGGER_STREAK,
            live,
            (live > uiLongestMs) ? live : uiLongestMs,
            uiTotalSnoreMs + live,
            uiCycleCount, uiNudgeCount);
}
