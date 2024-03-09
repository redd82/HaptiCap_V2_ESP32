#include "hapticfeedback.h"

extern Config config;
extern DebugSettings debugSettings;

extern RTC_DATA_ATTR bool bTargetReachedAck;
extern const int pwmledcommon;
extern const int pwmled0;
extern const int pwmled1;
extern const int pwmled2;
extern const int pwmled3;
extern const int pwmled4;
extern const int pwmled5;
extern const int pwmled6;
extern const int pwmled7;
extern const int intensityOffsetGreen;


// PWM settings for haptic feedback and LED feedback
const int distancePOI_scalar = 2;
const int hapticfront = 25;
const int hapticright = 26;
const int hapticrear = 32;
const int hapticleft = 33;
const int pwmhapticfront = 0;
const int pwmhapticright = 1;
const int pwmhapticrear = 2;
const int pwmhapticleft = 3;

float b = (255.0/90.0);   // 256/90 (2.8333333F) pwm scaled to 90 degrees (quadrant)
int pwm_front = 0;
int pwm_right = 0;
int pwm_rear = 0;
int pwm_left = 0;

void HapticFeedbackHeading(float relativeHeading,bool bEnable,bool debug,int distancePOI){
Serial.println("HapticFeedback Called");
float front;
float right;
float rear;
float left;
int mindelay = 150;
float DelayCoefficient = (config.maxDelay - mindelay) /(config.maxDistance - config.targetReached);
float DelayOffset = mindelay - (DelayCoefficient * config.targetReached);
int distancepulse;                                                                            
bool bTargetReached = 0;
int haptic_counter = 0;
int haptic_bursts = 2;

distancepulse = distancePOI * DelayCoefficient + DelayOffset;

if(distancepulse < mindelay){
  distancepulse = mindelay;
}

if(distancepulse > 1000){
  distancepulse = 1000;
}
  
if (distancePOI < config.targetReached){
  bTargetReached = true;
}else{
  bTargetReached = false;
}
    
if (bEnable){
  if((315.0 <= relativeHeading && relativeHeading <= 359.99) || (0.0 <= relativeHeading && relativeHeading <= 44.99)){
  // front haptic on
  // 0 as midpoint ok
    if(0.0 <= relativeHeading && relativeHeading <= 44){
      front = 255.0 - (relativeHeading * b);
      right = 255.0 - ((90.0 - relativeHeading) * b);
      pwm_front = (int)front;
      pwm_right = (int)right;
      pwm_rear = 0;
      pwm_left = 0;
    }else if (315.0 <= relativeHeading && relativeHeading <= 359.99){
      front = 255.0 - ((360.0 - relativeHeading) * b);
      left = 255.0 - ((relativeHeading - 270.0) * b);
      pwm_front = (int)front;
      pwm_left = (int)left;
      pwm_rear = 0;
      pwm_right = 0; 
    }
  }
  else if(45.0 <= relativeHeading && relativeHeading <= 134.99){
  // right haptic on
  // 90 as midpoint ok
      if(relativeHeading <= 89.99){
        right = 255.0 - ((90.0 - relativeHeading) * b);
        front = 255.0 - (relativeHeading * b);
        pwm_right = (int)right;
        pwm_front = (int)front;
        pwm_rear = 0;
        pwm_left = 0;
      }
      else if(relativeHeading >= 90.0){
        right = 255.0 - ((relativeHeading - 90.0) * b);
        rear = 255.0 - ((180.0 - relativeHeading) * b);
        pwm_right = (int)right;
        pwm_rear = (int)rear;
        pwm_front = 0;
        pwm_left = 0;
      }
  }
  else if(135.0 <= relativeHeading && relativeHeading <= 224.99 ){
  // rear haptic on
  // 180 as midpoint ok
      if(relativeHeading <= 179.99){
        rear = 255.0 - ((180.0 - relativeHeading) * b);
        right = 255.0 - ((relativeHeading - 90.0) * b);          
        pwm_rear = (int)rear;
        pwm_right = (int)right;
        pwm_front = 0;
        pwm_left = 0;
      }
      else if(relativeHeading >= 180.0){
        rear = 255.0 - ((relativeHeading - 180.0) * b);
        left = 255.0 - ((270.0 - relativeHeading) * b);
        pwm_rear = (int)rear;
        pwm_left = (int)left;
        pwm_front = 0;
        pwm_left = 0;
      }    
  }
  else if(225.0 <= relativeHeading && relativeHeading <= 314.99 ){
  // left haptic on
  //270 as midpoint ok
      if(relativeHeading <= 269.99){
        left = 255.0 - ((270.0 - relativeHeading) * b);
        rear = 255.0 - ((relativeHeading - 180) * b);
        pwm_left = (int)left;
        pwm_rear = (int)rear;
        pwm_front = 0;
        pwm_right = 0;
      }
      else if(relativeHeading >= 270.0){
        left = 255.0 - ((relativeHeading - 270.0) * b);
        front = 255.0 - ((360.0 - relativeHeading) * b);
        pwm_left = (int)left;
        pwm_front = (int)front;
        pwm_rear = 0;
        pwm_right = 0;
      }
    }
  }else if (debug){
      pwm_left = 255;
      pwm_front = 255;
      pwm_rear = 255;
      pwm_right = 255;
  }
  else {
      pwm_left = 0;
      pwm_front = 0;
      pwm_rear = 0;
      pwm_right = 0;
  }

  if (bTargetReached){
    while (haptic_counter < haptic_bursts){
      ledcWrite(pwmhapticright, 255); 
      ledcWrite(pwmhapticfront, 255);
      ledcWrite(pwmhapticrear, 255);
      ledcWrite(pwmhapticleft, 255);
      delay(75);
      ledcWrite(pwmhapticright, 0); 
      ledcWrite(pwmhapticfront, 0);
      ledcWrite(pwmhapticrear, 0);
      ledcWrite(pwmhapticleft, 0);
      delay(75);
      haptic_counter++;
    }
  }else{
    if( distancePOI < config.maxDistance){
      while (haptic_counter < haptic_bursts){
          ledcWrite(pwmhapticright, pwm_right); 
          ledcWrite(pwmhapticfront, pwm_front);
          ledcWrite(pwmhapticrear, pwm_rear);
          ledcWrite(pwmhapticleft, pwm_left);
        delay(distancepulse);
          ledcWrite(pwmhapticright, 0); 
          ledcWrite(pwmhapticfront, 0);
          ledcWrite(pwmhapticrear, 0);
          ledcWrite(pwmhapticleft, 0);
        delay(distancepulse);
        haptic_counter++;
      }  
    }else{
      ledcWrite(pwmhapticright, pwm_right); 
      ledcWrite(pwmhapticfront, pwm_front);
      ledcWrite(pwmhapticrear, pwm_rear);
      ledcWrite(pwmhapticleft, pwm_left);                
    }
  }
  //delay(500);   
  if (debugSettings.debug2Serial){
      Serial.print("relativeHeading: ");
      Serial.println(relativeHeading);
      Serial.print("distancePOI: ");
      Serial.println(distancePOI);
      Serial.print("Target reached: ");
      Serial.println(bTargetReached);
      Serial.print("Target reached ack: ");
      Serial.println(bTargetReachedAck);
      delay(100);                
  }
}

void switchLed(int led, int speed, int color, int intensity){
  int pwm = ledcRead(led);
  int pwm_target = intensity;
  int pwm_common = ledcRead(pwmledcommon);  
  if(color == 1){
    pwm_target = pwm_target - intensityOffsetGreen;
    ledcWrite(pwmledcommon, 0);
    ledcWrite(pwmled0, 0);
    ledcWrite(pwmled1, 0);
    ledcWrite(pwmled2, 0);
    ledcWrite(pwmled3, 0);
    ledcWrite(pwmled4, 0);
    ledcWrite(pwmled5, 0);
    ledcWrite(pwmled6, 0);
    ledcWrite(pwmled7, 0);
  }else if (color == 2){
    pwm_target = ~intensity;
    pwm_target = pwm_target + 255;
    Serial.println(pwm_target);
    ledcWrite(pwmledcommon, 255);
    ledcWrite(pwmled0, 255);
    ledcWrite(pwmled1, 255);
    ledcWrite(pwmled2, 255);
    ledcWrite(pwmled3, 255);
    ledcWrite(pwmled4, 255);
    ledcWrite(pwmled5, 255);
    ledcWrite(pwmled6, 255);
    ledcWrite(pwmled7, 255);
  }else{

  }

  while (pwm != pwm_target){
    switch(led){
      case 0: 
        ledcWrite(pwmled0, pwm);
        break;
      case 1: 
        ledcWrite(pwmled1, pwm);
        break;
      case 2: 
        ledcWrite(pwmled2, pwm);
        break;
      case 3: 
        ledcWrite(pwmled3, pwm);
        break;
      case 4: 
        ledcWrite(pwmled4, pwm);
        break;
      case 5: 
        ledcWrite(pwmled5, pwm);
        break;
      case 6: 
        ledcWrite(pwmled6, pwm);
        break;
      case 7: 
        ledcWrite(pwmled7, pwm);
        break;             
    }
    if(color == 1){
      if(pwm_target > pwm){
        pwm++;
      }else{
        pwm--;
      }
    }else if(color == 2){
      if(pwm > pwm_target){
        pwm--;
      }else{
        pwm++;
      }       
    }else{

    }
    delay(speed);
  }  
}

void ledTesting(){
  switchLed(0,5,1,254);           // switch led0 on with 5 ms delay between steps, green, full intensity 
  switchLed(0,5,1,0);

  switchLed(6,5,2,254);           // switch led6 on with 5 ms delay between steps, green, full intensity 
  delay(2000);
  switchLed(6,5,2,0);

  switchLed(3,2,1,254);           // switch led3 on with 2 ms delay between steps, green, full intensity 
//  Serial.println("1");
  // for(i = 0; i < 8; i++){
  //   switchLedUp(i,5,1,255);
  // }
  // Serial.println("2");
  // for(i = 0; i < 8; i++){
  //   switchLedDown(i,5,1,0);
  // }
  // Serial.println("3");
  // for(i = 0; i < 8; i++){
  //   switchLedUp(i,5,2,255);
  // }
  // Serial.println("4");
  // for(i = 0; i < 8; i++){
  //   switchLedDown(i,5,2,0);
  // } 
   // switchLed(3,2,1,0);   
};