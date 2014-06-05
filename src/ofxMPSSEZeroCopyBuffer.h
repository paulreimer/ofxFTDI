#pragma once

#include <vector>

class ofxMPSSEZeroCopyBuffer
{
public:
  ofxMPSSEZeroCopyBuffer();
  virtual ~ofxMPSSEZeroCopyBuffer();

  void resize(size_t _userByteSize);

  size_t size() const;

  uint8_t* data();

  std::vector<uint8_t> internalBuffer;

private:
  size_t userByteSize;
  uint8_t* userData;
};