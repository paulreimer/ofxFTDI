#include <string>
#include <sstream>

template<typename T>
std::string getBinaryString(const T& binaryValue)
{
  std::stringstream ss;
  size_t bitsSize = sizeof(T)*8;
  //  unsigned int mask = 0x80;
  T mask = (1<<(bitsSize-1));
  
  for (size_t b=0; b<bitsSize; ++b)
  {
    if (binaryValue & mask)
      ss << "1";
    else
      ss << "0";
    
    mask >>= 1;
  }
  
  return ss.str();
}
