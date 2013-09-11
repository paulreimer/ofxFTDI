#pragma once

extern "C" {
#include "mpsse.h"
}

#include <vector>
#include <cstddef>

class ofxMPSSE
{
public:
  ofxMPSSE();
  ~ofxMPSSE();
  
  bool connect(enum modes mode, int freq, int endianess, int index, interface iface=IFACE_A);
  bool connect(enum modes mode=SPI0, int freq=ONE_MHZ, int endianess=MSB, interface iface=IFACE_A, const char* description=NULL, const char* serial=NULL, int index=0);
  void send(const std::vector<uint8_t>& data);
  void read(std::vector<uint8_t>& data);

  void setGPIO(uint8_t value);
  uint8_t getGPIO();

  void setGPIOAddresses(uint8_t _maxGPIOAddress);
  void setGPIOAddress(uint8_t addr);
  uint8_t getGPIOAddress();

  struct mpsse_context *ftdi;
  bool connected;

  uint8_t maxGPIOAddressMask;

protected:
  uint8_t _getGPIO();
};
