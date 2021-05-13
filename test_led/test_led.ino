#define CALIB_PENDING 13
#define CALIB_DONE 8

#include "blink_new.h"

blk_n led1(CALIB_PENDING);

void setup() {
  // put your setup code here, to run once:
//  pinMode(CALIB_PENDING, OUTPUT);
//  pinMode(CALIB_DONE, OUTPUT);

//  digitalWrite(CALIB_PENDING, HIGH);
//  digitalWrite(CALIB_DONE, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
//  delay(2000);
//  digitalWrite(CALIB_PENDING, LOW);
//  digitalWrite(CALIB_DONE, HIGH);
  led1.blk_led(100);

}
