/*

   blink_new.cpp

   Autor: Eng. Wagner Rambo
   Março de 2018


*/

#include "Arduino.h"
#include "blink_new.h"

blk_n::blk_n(int pin)
{
   pinMode(pin, OUTPUT);
   _pin = pin;  
  
}

void blk_n::blk_led(int ms)
{
   digitalWrite(_pin, HIGH);
   delay(ms);
   digitalWrite(_pin,  LOW);
   delay(ms);
  
}









