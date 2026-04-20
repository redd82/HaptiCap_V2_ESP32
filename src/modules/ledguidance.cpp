#include "ledguidance.h"

#include "hapticfeedback.h"
#include "../structs/config.h"
#include "../structs/debugsettings.h"

extern Config config;
extern DebugSettings debugSettings;
extern int intensityOffsetGreen;

namespace {
constexpr int kPwmLedCommon = 8;
constexpr int kPwmLed0 = 0;
constexpr int kPwmLed1 = 1;
constexpr int kPwmLed2 = 2;
constexpr int kPwmLed3 = 3;
constexpr int kPwmLed4 = 4;
constexpr int kPwmLed5 = 5;
constexpr int kPwmLed6 = 6;
constexpr int kPwmLed7 = 7;
}

float bLed = (255.0 / 90.0);

static void ensureLedPinsAttached() {
  // Guidance modes are mutually exclusive; detach haptic pins before driving shared LED pins.
  ledcDetachPin(25);
  ledcDetachPin(26);
  ledcDetachPin(32);
  ledcDetachPin(33);

  ledcAttachPin(19, kPwmLedCommon);
  ledcAttachPin(33, kPwmLed0);
  ledcAttachPin(25, kPwmLed1);
  ledcAttachPin(26, kPwmLed2);
  ledcAttachPin(27, kPwmLed3);
  ledcAttachPin(14, kPwmLed4);
  ledcAttachPin(4, kPwmLed5);
  ledcAttachPin(5, kPwmLed6);
  ledcAttachPin(18, kPwmLed7);
}

void LedGuidanceStop() {
  ensureLedPinsAttached();
  ledcWrite(kPwmLedCommon, 0);
  ledcWrite(kPwmLed0, 0);
  ledcWrite(kPwmLed1, 0);
  ledcWrite(kPwmLed2, 0);
  ledcWrite(kPwmLed3, 0);
  ledcWrite(kPwmLed4, 0);
  ledcWrite(kPwmLed5, 0);
  ledcWrite(kPwmLed6, 0);
  ledcWrite(kPwmLed7, 0);
}

static void writeDirectionalLeds(int front, int right, int rear, int left) {
  const int greenOffset = intensityOffsetGreen > 0 ? intensityOffsetGreen : 0;
  const int frontLevel = max(0, front - greenOffset);
  const int rightLevel = max(0, right - greenOffset);
  const int rearLevel = max(0, rear - greenOffset);
  const int leftLevel = max(0, left - greenOffset);

  // Use one LED per direction: 0=front, 2=right, 4=rear, 6=left.
  ledcWrite(kPwmLedCommon, 0);
  ledcWrite(kPwmLed0, frontLevel);
  ledcWrite(kPwmLed1, 0);
  ledcWrite(kPwmLed2, rightLevel);
  ledcWrite(kPwmLed3, 0);
  ledcWrite(kPwmLed4, rearLevel);
  ledcWrite(kPwmLed5, 0);
  ledcWrite(kPwmLed6, leftLevel);
  ledcWrite(kPwmLed7, 0);
}

void LedGuidanceHeading(float relativeHeading, bool bEnable, bool debug, int distancePOI) {
  ensureLedPinsAttached();
  HapticFeedbackStop();

  int pwmFront = 0;
  int pwmRight = 0;
  int pwmRear = 0;
  int pwmLeft = 0;

  if (bEnable) {
    if ((315.0 <= relativeHeading && relativeHeading <= 359.99) ||
        (0.0 <= relativeHeading && relativeHeading <= 44.99)) {
      if (0.0 <= relativeHeading && relativeHeading <= 44.0) {
        pwmFront = (int)(255.0 - (relativeHeading * bLed));
        pwmRight = (int)(255.0 - ((90.0 - relativeHeading) * bLed));
      } else {
        pwmFront = (int)(255.0 - ((360.0 - relativeHeading) * bLed));
        pwmLeft = (int)(255.0 - ((relativeHeading - 270.0) * bLed));
      }
    } else if (45.0 <= relativeHeading && relativeHeading <= 134.99) {
      if (relativeHeading <= 89.99) {
        pwmRight = (int)(255.0 - ((90.0 - relativeHeading) * bLed));
        pwmFront = (int)(255.0 - (relativeHeading * bLed));
      } else {
        pwmRight = (int)(255.0 - ((relativeHeading - 90.0) * bLed));
        pwmRear = (int)(255.0 - ((180.0 - relativeHeading) * bLed));
      }
    } else if (135.0 <= relativeHeading && relativeHeading <= 224.99) {
      if (relativeHeading <= 179.99) {
        pwmRear = (int)(255.0 - ((180.0 - relativeHeading) * bLed));
        pwmRight = (int)(255.0 - ((relativeHeading - 90.0) * bLed));
      } else {
        pwmRear = (int)(255.0 - ((relativeHeading - 180.0) * bLed));
        pwmLeft = (int)(255.0 - ((270.0 - relativeHeading) * bLed));
      }
    } else if (225.0 <= relativeHeading && relativeHeading <= 314.99) {
      if (relativeHeading <= 269.99) {
        pwmLeft = (int)(255.0 - ((270.0 - relativeHeading) * bLed));
        pwmRear = (int)(255.0 - ((relativeHeading - 180.0) * bLed));
      } else {
        pwmLeft = (int)(255.0 - ((relativeHeading - 270.0) * bLed));
        pwmFront = (int)(255.0 - ((360.0 - relativeHeading) * bLed));
      }
    }
  } else if (debug) {
    pwmFront = 255;
    pwmRight = 255;
    pwmRear = 255;
    pwmLeft = 255;
  }

  if (distancePOI > config.maxDistance) {
    pwmFront = 0;
    pwmRight = 0;
    pwmRear = 0;
    pwmLeft = 0;
  }

  writeDirectionalLeds(pwmFront, pwmRight, pwmRear, pwmLeft);

  if (debugSettings.debug2Serial) {
    Serial.print("LED guidance heading: ");
    Serial.println(relativeHeading);
    Serial.print("LED guidance distance: ");
    Serial.println(distancePOI);
  }
}
