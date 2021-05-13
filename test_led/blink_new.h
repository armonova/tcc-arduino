/*

   blink_new.h

   Autor: Eng. Wagner Rambo
   Março de 2018


*/

#ifndef blink_n
#define blink_n

#include "Arduino.h"

class blk_n
{
   public:
       blk_n(int pin);  
       void blk_led(int ms);

   private:
       int _pin;
  
  
};

#endif














