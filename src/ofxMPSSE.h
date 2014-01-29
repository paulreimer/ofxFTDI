#pragma once

#define ASYNC_SUPPORT

#ifdef ASYNC_SUPPORT
#include "ofThread.h"
#endif

extern "C" {
#include "mpsse.h"
}

#include <vector>
#include <cstddef>

class ofxMPSSE
: public ofThread
{
public:
  ofxMPSSE();
  ~ofxMPSSE();
  
  bool connect(enum modes _mode, int _freq, int _endianess, int _index, interface _iface=IFACE_A);
  bool connect(enum modes _mode=SPI0, int _freq=ONE_MHZ, int _endianess=MSB, interface _iface=IFACE_A, const char* _description=NULL, const char* _serial=NULL, int _index=0, bool verbose=true);

  bool reconnect();

#ifdef ASYNC_SUPPORT
  void asyncConnect(enum modes _mode, int _freq, int _endianess, int _index, interface _iface=IFACE_A, size_t _asyncConnectInterval=1e6);
  void asyncConnect(enum modes _mode=SPI0, int _freq=ONE_MHZ, int _endianess=MSB, interface _iface=IFACE_A, const char* _description=NULL, const char* _serial=NULL, int _index=0, size_t _asyncConnectInterval=1e6);

  void asyncReconnect(size_t _asyncConnectInterval=1e6);

  void threadedFunction();
  ofMutex asyncMutex;
  size_t asyncConnectInterval;
#endif

  bool send(const std::vector<uint8_t>& data);
  bool read(std::vector<uint8_t>& data);
  bool transfer(const std::vector<uint8_t>& dataOut,
                std::vector<uint8_t>& dataIn);

  bool setGPIO(uint8_t value);
  uint8_t getGPIO();

  void setGPIOAddresses(uint8_t _maxGPIOAddress);
  bool setGPIOAddress(uint8_t addr);
  uint8_t getGPIOAddress();

  bool isConnected();

  struct mpsse_context *ftdi;
  bool connected;

  uint8_t maxGPIOAddressMask;

private:
  enum modes mode;
  int freq;
  int endianess;
  interface iface;
  const char* description;
  const char* serial;
  int index;
};
