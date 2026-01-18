#pragma once
#include <Arduino.h>

class MBusParser {
public:
  MBusParser() {}

  char     CustomerNumber[10];
  uint8_t  Alarm1;
  uint8_t  Alarm2;
  bool     ChecksumOk;
  uint32_t EnergyValue;
  int8_t   EnergyUnit;
  double   EnergykWh;
  uint32_t VolumeValue;
  int8_t   VolumeUnit;
  double   Volumem3;
  uint8_t  TimePointHour;
  uint8_t  TimePointMin;
  uint8_t  TimePointDay;
  uint8_t  TimePointMonth;
  uint8_t  TimePointYear;
  float    FlowTempHot;
  float    FlowTempCold;

  void ParseLL(uint8_t *buf, int bufLen);
};
