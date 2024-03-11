#include <Arduino.h>
#include "pwm.h"

extern int pwmledcommon, pwmled0, pwmled1, pwmled2, pwmled3, pwmled4, pwmled5, pwmled6, pwmled7;
extern int ledcommon, led0, led1, led2, led3, led4, led5, led6, led7;
extern int hapticfreq, resolution;

void PWMSetup(){
//PWM setup
  // ledcSetup(pwmhapticfront, hapticfreq, resolution);
  // ledcSetup(pwmhapticright, hapticfreq, resolution);
  // ledcSetup(pwmhapticrear, hapticfreq, resolution);
  // ledcSetup(pwmhapticleft, hapticfreq, resolution);
  // ledcAttachPin(hapticfront, pwmhapticfront);
  // ledcAttachPin(hapticright, pwmhapticright);
  // ledcAttachPin(hapticrear, pwmhapticrear);
  // ledcAttachPin(hapticleft, pwmhapticleft);

  ledcSetup(pwmledcommon, hapticfreq, resolution);
  ledcSetup(pwmled0, hapticfreq, resolution);
  ledcSetup(pwmled1, hapticfreq, resolution);
  ledcSetup(pwmled2, hapticfreq, resolution);
  ledcSetup(pwmled3, hapticfreq, resolution);
  ledcSetup(pwmled4, hapticfreq, resolution);
  ledcSetup(pwmled5, hapticfreq, resolution);
  ledcSetup(pwmled6, hapticfreq, resolution);
  ledcSetup(pwmled7, hapticfreq, resolution);
  ledcAttachPin(ledcommon, pwmledcommon);
  ledcAttachPin(led0, pwmled0);
  ledcAttachPin(led1, pwmled1);
  ledcAttachPin(led2, pwmled2);
  ledcAttachPin(led3, pwmled3);
  ledcAttachPin(led4, pwmled4);
  ledcAttachPin(led5, pwmled5);
  ledcAttachPin(led6, pwmled6);        
  ledcAttachPin(led7, pwmled7);  
}