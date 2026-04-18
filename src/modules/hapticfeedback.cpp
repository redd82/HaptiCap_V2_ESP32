#include "hapticfeedback.h"

extern Config config;
extern DebugSettings debugSettings;

extern RTC_DATA_ATTR bool bTargetReachedAck;

// PWM settings for vibrator guidance
const int hapticfront = 25;
const int hapticright = 26;
const int hapticrear  = 32;
const int hapticleft  = 33;
const int pwmhapticfront = 9;
const int pwmhapticright = 10;
const int pwmhapticrear  = 11;
const int pwmhapticleft  = 12;

static const float kB = 255.0f / 90.0f;  // PWM scaled per degree in a 90-degree quadrant

static void ensureHapticPinsAttached() {
  // Guidance modes are mutually exclusive; detach LED pins that overlap haptic pins.
  ledcDetachPin(33);
  ledcDetachPin(25);
  ledcDetachPin(26);
  ledcAttachPin(hapticfront, pwmhapticfront);
  ledcAttachPin(hapticright, pwmhapticright);
  ledcAttachPin(hapticrear,  pwmhapticrear);
  ledcAttachPin(hapticleft,  pwmhapticleft);
}

// ---------------------------------------------------------------------------
// Non-blocking pulsed-output state machine
// ---------------------------------------------------------------------------

enum class HapticPhase : uint8_t { IDLE, PULSE_ON, PULSE_OFF, CONTINUOUS };

struct HapticPattern {
  int front, right, rear, left;  // target PWM duty (0-255)
  int pulseMs;                   // ON / OFF phase duration; 0 = hold continuously
  int burstsTotal;               // number of ON+OFF cycles per trigger
};

static HapticPhase   hPhase          = HapticPhase::IDLE;
static unsigned long hPhaseEndMs     = 0;
static int           hBurstsLeft     = 0;
static HapticPattern hCurrent        = {};
static HapticPattern hPending        = {};
static bool          hPendingReady   = false;

static void hWrite(int f, int r, int rr, int l) {
  ledcWrite(pwmhapticfront, f);
  ledcWrite(pwmhapticright, r);
  ledcWrite(pwmhapticrear,  rr);
  ledcWrite(pwmhapticleft,  l);
}

static void hWriteZero() { hWrite(0, 0, 0, 0); }

static void hStartPattern(const HapticPattern &p) {
  hCurrent = p;
  hWrite(p.front, p.right, p.rear, p.left);
  if (p.pulseMs == 0) {
    hPhase = HapticPhase::CONTINUOUS;
  } else {
    hPhase      = HapticPhase::PULSE_ON;
    hBurstsLeft = p.burstsTotal;
    hPhaseEndMs = millis() + p.pulseMs;
  }
}

void HapticFeedbackStop() {
  hPhase        = HapticPhase::IDLE;
  hPendingReady = false;
  hWriteZero();
}

// Call from main loop() on every iteration — zero blocking.
void HapticFeedbackTick() {
  unsigned long now = millis();

  switch (hPhase) {
    case HapticPhase::IDLE:
      if (hPendingReady) {
        hPendingReady = false;
        hStartPattern(hPending);
      }
      break;

    case HapticPhase::CONTINUOUS:
      // Apply updated parameters immediately (e.g. heading changed).
      if (hPendingReady) {
        hPendingReady = false;
        hStartPattern(hPending);
      }
      break;

    case HapticPhase::PULSE_ON:
      // If a new pattern arrived, restart with the updated heading straight away.
      if (hPendingReady) {
        hPendingReady = false;
        hStartPattern(hPending);
        break;
      }
      if (now >= hPhaseEndMs) {
        hWriteZero();
        hBurstsLeft--;
        hPhase      = HapticPhase::PULSE_OFF;
        hPhaseEndMs = now + hCurrent.pulseMs;
      }
      break;

    case HapticPhase::PULSE_OFF:
      // Also restart mid-gap if a new pattern is waiting — avoids dead time.
      if (hPendingReady) {
        hPendingReady = false;
        hStartPattern(hPending);
        break;
      }
      if (now >= hPhaseEndMs) {
        if (hBurstsLeft > 0) {
          hWrite(hCurrent.front, hCurrent.right, hCurrent.rear, hCurrent.left);
          hPhase      = HapticPhase::PULSE_ON;
          hPhaseEndMs = now + hCurrent.pulseMs;
        } else {
          hPhase = HapticPhase::IDLE;
        }
      }
      break;
  }
}

void HapticFeedbackHeading(float relativeHeading, bool bEnable, bool debug, int distancePOI) {
  ensureHapticPinsAttached();

  const int   mindelay = 150;
  const float dc = (config.maxDelay - mindelay) / (float)(config.maxDistance - config.targetReached);
  const float dOffset = mindelay - (dc * config.targetReached);
  int distancepulse = (int)(distancePOI * dc + dOffset);
  if (distancepulse < mindelay) distancepulse = mindelay;
  if (distancepulse > 1000)     distancepulse = 1000;

  const bool bTargetReached = (distancePOI < config.targetReached);

  int pFront = 0, pRight = 0, pRear = 0, pLeft = 0;

  if (bEnable) {
    if ((315.0f <= relativeHeading && relativeHeading <= 359.99f) ||
        (0.0f   <= relativeHeading && relativeHeading <= 44.99f)) {
      if (relativeHeading <= 44.0f) {
        pFront = (int)(255.0f - relativeHeading * kB);
        pRight = (int)(255.0f - (90.0f - relativeHeading) * kB);
      } else {
        pFront = (int)(255.0f - (360.0f - relativeHeading) * kB);
        pLeft  = (int)(255.0f - (relativeHeading - 270.0f) * kB);
      }
    } else if (45.0f <= relativeHeading && relativeHeading <= 134.99f) {
      if (relativeHeading <= 89.99f) {
        pRight = (int)(255.0f - (90.0f - relativeHeading) * kB);
        pFront = (int)(255.0f - relativeHeading * kB);
      } else {
        pRight = (int)(255.0f - (relativeHeading - 90.0f) * kB);
        pRear  = (int)(255.0f - (180.0f - relativeHeading) * kB);
      }
    } else if (135.0f <= relativeHeading && relativeHeading <= 224.99f) {
      if (relativeHeading <= 179.99f) {
        pRear  = (int)(255.0f - (180.0f - relativeHeading) * kB);
        pRight = (int)(255.0f - (relativeHeading - 90.0f) * kB);
      } else {
        pRear  = (int)(255.0f - (relativeHeading - 180.0f) * kB);
        pLeft  = (int)(255.0f - (270.0f - relativeHeading) * kB);
      }
    } else if (225.0f <= relativeHeading && relativeHeading <= 314.99f) {
      if (relativeHeading <= 269.99f) {
        pLeft = (int)(255.0f - (270.0f - relativeHeading) * kB);
        pRear = (int)(255.0f - (relativeHeading - 180.0f) * kB);
      } else {
        pLeft  = (int)(255.0f - (relativeHeading - 270.0f) * kB);
        pFront = (int)(255.0f - (360.0f - relativeHeading) * kB);
      }
    }
  } else if (debug) {
    pFront = pRight = pRear = pLeft = 255;
  }

  HapticPattern pat = {};
  if (bTargetReached) {
    pat.front = pat.right = pat.rear = pat.left = 255;
    pat.pulseMs     = 75;
    pat.burstsTotal = 2;
  } else if (distancePOI < config.maxDistance) {
    pat.front = pFront; pat.right = pRight;
    pat.rear  = pRear;  pat.left  = pLeft;
    pat.pulseMs     = distancepulse;
    pat.burstsTotal = 2;
  } else {
    // Beyond navigation range: hold directional motors on continuously.
    pat.front = pFront; pat.right = pRight;
    pat.rear  = pRear;  pat.left  = pLeft;
    pat.pulseMs     = 0;  // continuous
    pat.burstsTotal = 0;
  }

  hPending      = pat;
  hPendingReady = true;
  // Tick immediately so the first motor write happens this cycle.
  HapticFeedbackTick();

  if (debugSettings.debug2Serial) {
    Serial.print("relativeHeading: ");    Serial.println(relativeHeading);
    Serial.print("distancePOI: ");        Serial.println(distancePOI);
    Serial.print("Target reached: ");     Serial.println(bTargetReached);
    Serial.print("Target reached ack: "); Serial.println(bTargetReachedAck);
  }
}
