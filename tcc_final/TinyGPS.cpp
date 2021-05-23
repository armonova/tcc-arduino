/*
TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
Based on work by and "distance_to" and "course_to" courtesy of Maarten Lamers.
Suggestion to add satellites(), course_to(), and cardinal(), by Matt Monson.
Precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "TinyGPS.h"

#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"

TinyGPS::TinyGPS()
  :  _latitude(GPS_INVALID_ANGLE)
  ,  _longitude(GPS_INVALID_ANGLE)
  ,  _speed(GPS_INVALID_SPEED)
  ,  _course(GPS_INVALID_ANGLE)
  ,  _hdop(GPS_INVALID_HDOP)
  ,  _parity(0)
  ,  _is_checksum_term(false)
  ,  _sentence_type(_GPS_SENTENCE_OTHER)
  ,  _term_number(0)
  ,  _term_offset(0)
  ,  _gps_data_good(false)
#ifndef _GPS_NO_STATS
  ,  _encoded_characters(0)
  ,  _good_sentences(0)
  ,  _failed_checksum(0)
#endif
{
  _term[0] = '\0';
}

//
// public methods
//

bool TinyGPS::encode(char c)
{
  bool valid_sentence = false;

#ifndef _GPS_NO_STATS
  ++_encoded_characters;
#endif
  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term))
    {
      _term[_term_offset] = 0;
      valid_sentence = term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _is_checksum_term = false;
    _gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1)
    _term[_term_offset++] = c;
  if (!_is_checksum_term)
    _parity ^= c;

  return valid_sentence;
}

#ifndef _GPS_NO_STATS
void TinyGPS::stats(unsigned long *chars, unsigned short *sentences, unsigned short *failed_cs)
{
  if (chars) *chars = _encoded_characters;
  if (sentences) *sentences = _good_sentences;
  if (failed_cs) *failed_cs = _failed_checksum;
}
#endif

//
// internal utilities
//
int TinyGPS::from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

unsigned long TinyGPS::parse_decimal()
{
  char *p = _term;
  bool isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * gpsatol(p);
  while (gpsisdigit(*p)) ++p;
  if (*p == '.')
  {
    if (gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}

// Parse a string in the form ddmm.mmmmmmm...
unsigned long TinyGPS::parse_degrees()
{
  char *p;
  unsigned long left_of_decimal = gpsatol(_term);
  unsigned long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
  for (p=_term; gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 10000;
    while (gpsisdigit(*++p))
    {
      hundred1000ths_of_minute += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPS::term_complete()
{
  if (_is_checksum_term)
  {
    byte checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
    if (checksum == _parity)
    {
      if (_gps_data_good)
      {
#ifndef _GPS_NO_STATS
        ++_good_sentences;
#endif
        _last_position_fix = _new_position_fix;

        switch(_sentence_type)
        {
        case _GPS_SENTENCE_GPRMC:
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _speed     = _new_speed;
          _course    = _new_course;
          break;
        case _GPS_SENTENCE_GPGGA:
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _hdop      = _new_hdop;
          break;
        }

        return true;
      }
    }

#ifndef _GPS_NO_STATS
    else
      ++_failed_checksum;
#endif
    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!gpsstrcmp(_term, _GPRMC_TERM))
      _sentence_type = _GPS_SENTENCE_GPRMC;
    else if (!gpsstrcmp(_term, _GPGGA_TERM))
      _sentence_type = _GPS_SENTENCE_GPGGA;
    else
      _sentence_type = _GPS_SENTENCE_OTHER;
    return false;
  }

  if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
    switch(COMBINE(_sentence_type, _term_number))
  {
    case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
      _gps_data_good = _term[0] == 'A';
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 2):
      _new_latitude = parse_degrees();
      _new_position_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
    case COMBINE(_GPS_SENTENCE_GPGGA, 3):
      if (_term[0] == 'S')
        _new_latitude = -_new_latitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 5): // Longitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 4):
      _new_longitude = parse_degrees();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 6): // E/W
    case COMBINE(_GPS_SENTENCE_GPGGA, 5):
      if (_term[0] == 'W')
        _new_longitude = -_new_longitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
      _new_speed = parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
      _new_course = parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      _gps_data_good = _term[0] > '0';
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 8): // HDOP
      _new_hdop = parse_decimal();
      break;
  }

  return false;
}

void TinyGPS::standard_deviation_gps() {
  char _acquires = 10; // número de aquisição para cálculo do desvio padrão
  
  float _posi_gps_DP_N[_acquires];
  float _posi_gps_DP_N[_acquires];
  float _posi_gps_DP_MOD[_acquires];
  float _vel_gps_DP_MOD[_acquires];

  float _sum_posi_gps_N = 0.0;
  float _sum_posi_gps_E = 0.0;
  float _sum_posi_gps_MOD = 0.0;
  float _sum_vel_gps_MOD = 0.0;
  
  for (char i = 0; i < _acquires; i++) {
    // TODO
    make_conversion();
    _posi_gps_DP_N[i] = _acc_N; // TODO
    _posi_gps_DP_E[i] = _acc_E; // TODO
    _vel_gps_DP_MOD[i] = sqrt((_acc_acc_DP_N[i] * _acc_acc_DP_N[i]) * (_acc_acc_DP_E[i]) * _acc_acc_DP_E[i]);
  
    _sum_acc_acc_N += _acc_acc_DP_N[i];
    _sum_acc_acc_E += _acc_acc_DP_E[i];
    _sum_acc_acc_MOD += _acc_acc_DP_MOD[i];
    delay(25); // frequencia de 40Hz
  }

  delay(2000);

  float _avg_acc_acc_N = _sum_acc_acc_N / _acquires;  
  float _avg_acc_acc_E = _sum_acc_acc_E / _acquires;
  float _avg_acc_acc_MOD = _sum_acc_acc_MOD / _acquires;
  
  float _auxN_DP_acc_acc_N = 0.0;
  float _auxE_DP_acc_acc_E = 0.0;
  float _auxE_DP_acc_acc_MOD = 0.0;

  for (int i = 0; i < _acquires; i++) {
    _auxN_DP_acc_acc_N += (_acc_acc_DP_N[i] - _avg_acc_acc_N) * (_acc_acc_DP_N[i] - _avg_acc_acc_N);
    _auxE_DP_acc_acc_E += (_acc_acc_DP_E[i] - _avg_acc_acc_E) * (_acc_acc_DP_E[i] - _avg_acc_acc_E);
    _auxE_DP_acc_acc_MOD += (_acc_acc_DP_MOD[i] - _avg_acc_acc_MOD) * (_acc_acc_DP_MOD[i] - _avg_acc_acc_MOD);
  }

  _DP_acc_acc_N = sqrt(_auxN_DP_acc_acc_N / _acquires);  
  _DP_acc_acc_E = sqrt(_auxE_DP_acc_acc_E / _acquires);
  _DP_acc_acc_MOD = sqrt(_auxE_DP_acc_acc_MOD / _acquires);
}

long TinyGPS::gpsatol(const char *str)
{
  long ret = 0;
  while (gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}

int TinyGPS::gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}

const char *TinyGPS::cardinal (float course)
{
  static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

// lat/long in MILLIONTHs of a degree and age of fix in milliseconds
// (note: versions 12 and earlier gave this value in 100,000ths of a degree.
void TinyGPS::get_position(long *latitude, long *longitude, unsigned long *fix_age)
{
  if (latitude) *latitude = _latitude;
  if (longitude) *longitude = _longitude;
   GPS_INVALID_AGE : millis() - _last_position_fix;
}

// date as ddmmyy, time as hhmmsscc, and age in milliseconds

void TinyGPS::f_get_position(float *latitude, float *longitude, unsigned long *fix_age)
{
  long lat, lon;
  get_position(&lat, &lon, fix_age);
  *latitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lat / 1000000.0);
  *longitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lon / 1000000.0);
}

float TinyGPS::f_course()
{
  return _course == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : _course / 100.0;
}

float TinyGPS::f_speed_knots() 
{
  return _speed == GPS_INVALID_SPEED ? GPS_INVALID_F_SPEED : _speed / 100.0;
}

float TinyGPS::f_speed_mps()   
{ 
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPS_PER_KNOT * sk; 
}

const float TinyGPS::GPS_INVALID_F_ANGLE = 1000.0;
const float TinyGPS::GPS_INVALID_F_ALTITUDE = 1000000.0;
const float TinyGPS::GPS_INVALID_F_SPEED = -1.0;
