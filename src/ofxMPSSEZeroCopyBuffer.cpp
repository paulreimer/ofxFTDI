#include "ofxMPSSEZeroCopyBuffer.h"

#include <iostream>

extern "C" {
#include <string.h>
#include "mpsse.h"
#include "support.h"
}

//--------------------------------------------------------------
ofxMPSSEZeroCopyBuffer::ofxMPSSEZeroCopyBuffer()
{}

//--------------------------------------------------------------
ofxMPSSEZeroCopyBuffer::~ofxMPSSEZeroCopyBuffer()
{}

//--------------------------------------------------------------
void
ofxMPSSEZeroCopyBuffer::resize(size_t _userByteSize)
{
  userByteSize = _userByteSize;

  internalBuffer.resize(userByteSize + 3);

  uint16_t rsize = (userByteSize - 1);
  uint16_t i = 0;

  uint8_t cmd = MPSSE_DO_WRITE | MPSSE_WRITE_NEG | MSB; // MODE0, MSB (default)

  // Copy in the command for this block
  internalBuffer[i++] = cmd;
  internalBuffer[i++] = (rsize & 0xFF);
  internalBuffer[i++] = ((rsize >> 8) & 0xFF);
}

//--------------------------------------------------------------
size_t
ofxMPSSEZeroCopyBuffer::size() const
{
  return userByteSize;
}

//--------------------------------------------------------------
uint8_t*
ofxMPSSEZeroCopyBuffer::data()
{
  return (internalBuffer.size() > 3)? &internalBuffer[3] : NULL;
}
