#include "ofxMPSSE.h"
#include <iostream>

//--------------------------------------------------------------
ofxMPSSE::ofxMPSSE()
: ftdi(NULL)
, connected(false)
{}

//--------------------------------------------------------------
ofxMPSSE::~ofxMPSSE()
{
  if (connected)
    mpsse_close(ftdi);
}

//--------------------------------------------------------------
bool
ofxMPSSE::connect(enum modes mode, int freq, int endianess)
{
  ftdi = mpsse_easy_open(mode, freq, endianess);
  if (ftdi != NULL && ftdi->open)
  {
    std::cout
    << "FTDI device '"      << mpsse_get_description(ftdi)
    << "' initialized at "  << mpsse_get_clock(ftdi) << "Hz"
    << std::endl;

    connected = true;
  }
  else {
    std::cout
    << "Unable to connect to FTDI device; "
    << "don't forget to disable (or uninstall) the FTDI official/VCP drivers:"
    << std::endl
    << "sudo kextunload /System/Library/Extensions/FTDIUSBSerialDriver.kext"
    << std::endl;
  }
}

//--------------------------------------------------------------
void
ofxMPSSE::send(const std::vector<uint8_t>& data)
{
  if (connected)
    mpsse_write(ftdi, (char*)data.data(), data.size());
}
