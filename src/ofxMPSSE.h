#pragma once

extern "C" {
#include "mpsse.h"
}

#include <vector>

class ofxMPSSE
{
public:
  ofxMPSSE();
  ~ofxMPSSE();
  
  bool connect(enum modes mode, int freq, int endianess, int index, interface iface=IFACE_A);
  bool connect(enum modes mode=SPI0, int freq=ONE_MHZ, int endianess=MSB, interface iface=IFACE_A, char* description=NULL, char* serial=NULL, int index=0);
  void send(const std::vector<uint8_t>& data);

  struct mpsse_context *ftdi;
  bool connected;
};
