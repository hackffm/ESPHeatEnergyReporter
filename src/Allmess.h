#pragma once
#include <Arduino.h>
#include <Stream.h>

extern const int SerRxBufMaxLen;
extern uint8_t   SerRxBuf[];
extern int       SerRxBufLen;
void AllmessRead(int rxpin, int txpin, int rxenpin, Stream &DebugOut);