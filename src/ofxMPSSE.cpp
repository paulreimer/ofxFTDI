#include "ofxMPSSE.h"

#include <Poco/Timestamp.h>

#include <iostream>
#include <sstream>

extern "C" {
#include "support.h"
#include "ftdi.h"
#include <string.h>
#include <errno.h>
#include <sys/sysctl.h>
}

// List of known FT2232-based devices
struct vid_pid my_supported_devices[] = {
  { 0x0403, 0x6010, "FT2232 Future Technology Devices International, Ltd" },
  { 0x0403, 0x6011, "FT4232 Future Technology Devices International, Ltd" },
  { 0x0403, 0x6014, "FT232H Future Technology Devices International, Ltd" },
  
  /* These devices are based on FT2232 chips, but have not been tested. */
  { 0x0403, 0x8878, "Bus Blaster v2 (channel A)" },
  { 0x0403, 0x8879, "Bus Blaster v2 (channel B)" },
  { 0x0403, 0xBDC8, "Turtelizer JTAG/RS232 Adapter A" },
  { 0x0403, 0xCFF8, "Amontec JTAGkey" },
  { 0x0403, 0x8A98, "TIAO Multi Protocol Adapter"},
  { 0x15BA, 0x0003, "Olimex Ltd. OpenOCD JTAG" },
  { 0x15BA, 0x0004, "Olimex Ltd. OpenOCD JTAG TINY" },
  
  { 0, 0, NULL }
};

AsyncConnectionThread ofxMPSSE::asyncConnectionThread;

//--------------------------------------------------------------
ofxMPSSE::ofxMPSSE()
: ftdi(NULL)
, connected(false)
//, maxGPIOAddressMask((1<<4)-1)
, mode(SPI0)
, freq(ONE_MHZ)
, endianess(MSB)
, iface(IFACE_A)
, description(NULL)
, serial(NULL)
, index(0)
{
  setGPIOAddresses(16);
}

//--------------------------------------------------------------
ofxMPSSE::~ofxMPSSE()
{
  if (connected)
    Close(ftdi);
}

//--------------------------------------------------------------
bool
ofxMPSSE::connect(enum modes _mode, int _freq, int _endianess, int _index, interface _iface)
{
  return connect(_mode, _freq, _endianess, _iface, NULL, NULL, _index);
}

//--------------------------------------------------------------
bool
ofxMPSSE::connect(enum modes _mode, int _freq, int _endianess, interface _iface, const char* _description, const char* _serial, int _index, bool verbose)
{
  mode        = _mode;
  freq        = _freq;
  endianess   = _endianess;
  index       = _index;
  iface       = _iface;
  description = _description;
  serial      = _serial;

	for(size_t i=0; my_supported_devices[i].vid != 0; i++)
	{
		if((ftdi = OpenIndex(my_supported_devices[i].vid, my_supported_devices[i].pid, mode, freq, endianess, iface, description, serial, index)) != NULL)
		{
			if(ftdi->open)
			{
				ftdi->description = my_supported_devices[i].description;
				break;
			}
			/* If there is another device still left to try, free the context pointer and try again */
			else if(my_supported_devices[i+1].vid != 0)
			{
				Close(ftdi);
				ftdi = NULL;
			}
		}
	}

  if (ftdi != NULL && ftdi->open)
  {
//    if (verbose)
    {
      std::stringstream debugStream;
      debugStream
      << "FTDI device '" << GetDescription(ftdi) << "'";

      if (serial)
        debugStream
        << " (serial: '" << serial << "')";

      debugStream
      << " initialized at " << GetClock(ftdi) << "Hz";

      std::cout << debugStream.str() << std::endl;
    }

    connected = true;
  }
  else {
    if (verbose)
    {
      std::stringstream errorStream;
      errorStream
      << "Unable to connect to FTDI device";

      if (serial)
        errorStream
        << " (serial: '" << serial << "'), ";

      errorStream << "(" << ErrorString(ftdi) << ")";
#ifdef __APPLE__
      errorStream
      << "; "
      << std::endl
      << "don't forget to disable (or uninstall) the FTDI official/VCP drivers:"
      << std::endl;

      char str[256];
      size_t size = sizeof(str);
      int ret = sysctlbyname("kern.osrelease", str, &size, NULL, 0);

      if ((ret == 0 && size >= 2) && (str[0] == '1' && str[1] >= '3')) // OS X 10.9 (kernel version 13.x.x)
        errorStream << "sudo kextunload /System/Library/Extensions/IOUSBFamily.kext/Contents/PlugIns/AppleUSBFTDI.kext;" << std::endl;

      errorStream << "sudo kextunload /System/Library/Extensions/FTDIUSBSerialDriver.kext";
//#else
#endif

      std::cout << errorStream.str() << std::endl;
    }
  }

  return connected;
}

//--------------------------------------------------------------
bool
ofxMPSSE::reconnect(bool verbose)
{
  return connect(mode, freq, endianess, iface, description, serial, index, verbose);
}

//--------------------------------------------------------------
bool
ofxMPSSE::send(const std::vector<uint8_t>& data)
{
  if (connected)
  {
    connected = (FastWrite(ftdi, (char*)data.data(), data.size()) == MPSSE_OK);
//    connected = (Write(ftdi, (char*)data.data(), data.size()) == MPSSE_OK);
  }

  return connected;
}

//--------------------------------------------------------------
bool
ofxMPSSE::transfer(const std::vector<uint8_t>& dataOut,
                   std::vector<uint8_t>& dataIn)
{
  if (connected)
  {
    connected = (FastTransfer(ftdi, (char*)dataOut.data(), (char*)dataIn.data(), dataOut.size()) == MPSSE_OK);
    //    connected = (Transfer(ftdi, (char*)dataOut.data(), (char*)dataIn.data(), dataOut.size()) == MPSSE_OK);
  }

  return connected;
}

//--------------------------------------------------------------
bool
ofxMPSSE::read(std::vector<uint8_t>& data)
{
  if (connected)
  {
    //FastRead(ftdi, (char*)&data[0], data.size());
    char* dataPtr = Read(ftdi, data.size());
    if (dataPtr)
      memcpy(&data[0], dataPtr, data.size());
    else
      connected = false;
  }

  return connected;
}

//--------------------------------------------------------------
bool
ofxMPSSE::setGPIO(uint8_t value)
{
  if (connected)
  {
    ftdi->gpioh = value;
    connected = (set_bits_high(ftdi, ftdi->gpioh) == MPSSE_OK);
  }

  return connected;
}

//--------------------------------------------------------------
uint8_t
ofxMPSSE::getGPIO()
{
  uint8_t value = 0;
  if (connected)
  {
    value = ftdi->gpioh;
  }

  return value;
}

//--------------------------------------------------------------
bool
ofxMPSSE::setGPIOAddress(uint8_t addr)
{
  if (connected && (addr <= maxGPIOAddressMask))
  {
    connected = setGPIO((ftdi->gpioh & (~maxGPIOAddressMask)) | addr);
  }

  return connected;
}

//--------------------------------------------------------------
uint8_t
ofxMPSSE::getGPIOAddress()
{
  return (getGPIO() & maxGPIOAddressMask);
}

//--------------------------------------------------------------
void
ofxMPSSE::setGPIOAddresses(uint8_t maxAddressCount)
{
  unsigned int addressCount=1;
	while(addressCount<maxAddressCount)
    addressCount<<=1;

  maxGPIOAddressMask = (addressCount-1);
}

//--------------------------------------------------------------
bool
ofxMPSSE::isConnected()
{
#ifdef ASYNC_SUPPORT
  bool retval = false;
  if (asyncMutex.tryLock())
  {
    retval = connected;
    asyncMutex.unlock();
  }
  return retval;
#else
  return connected;
#endif
}

#ifdef ASYNC_SUPPORT
//--------------------------------------------------------------
void
ofxMPSSE::asyncConnect(enum modes _mode, int _freq, int _endianess, int _index, interface _iface, size_t _asyncConnectInterval)
{
  asyncConnect(_mode, _freq, _endianess, _iface, NULL, NULL, _index, _asyncConnectInterval);
}

//--------------------------------------------------------------
void
ofxMPSSE::asyncConnect(enum modes _mode, int _freq, int _endianess, interface _iface, const char* _description, const char* _serial, int _index, size_t _asyncConnectInterval)
{
//  ofMutex::ScopedLock lock(asyncMutex);
  if (asyncMutex.tryLock())
  {
    mode        = _mode;
    freq        = _freq;
    endianess   = _endianess;
    index       = _index;
    iface       = _iface;
    description = _description;
    serial      = _serial;

    asyncConnectInterval = _asyncConnectInterval;

    asyncConnectionThread.asyncConnect(*this);

    asyncMutex.unlock();
  }
}

//--------------------------------------------------------------
void
ofxMPSSE::asyncReconnect(size_t _asyncConnectInterval)
{
  if (asyncMutex.tryLock())
  {
    asyncConnectInterval = _asyncConnectInterval;

    asyncConnectionThread.asyncConnect(*this);

    asyncMutex.unlock();
  }
}

//--------------------------------------------------------------
void
AsyncConnectionThread::asyncConnect(ofxMPSSE& device)
{
  if (disconnectedDevices.find(&device) == disconnectedDevices.end())
  {
//    size_t now = Poco::Timestamp().epochMicroseconds();

    disconnectedDevices.insert(std::make_pair(&device, 0));
  }

  if (thread.isNull())
    thread = new Poco::Thread;

  if (!thread->isRunning())
    thread->start(*this);
}

//--------------------------------------------------------------
void
AsyncConnectionThread::run()
{
  bool verbose = false;

  while (!disconnectedDevices.empty())
  {
    size_t minRemainingWaitTime = 5e6; // wait by default

    std::map<ofxMPSSE*, size_t>::iterator deviceIter=disconnectedDevices.begin();
    while (deviceIter!=disconnectedDevices.end())
    {
      ofxMPSSE& device(*(deviceIter->first));
      size_t& lastConnectionAttemptTime(deviceIter->second);

      device.asyncMutex.lock();

      size_t now = Poco::Timestamp().epochMicroseconds();
      if (device.reconnect(lastConnectionAttemptTime? verbose : true))
      {
        std::map<ofxMPSSE*, size_t>::iterator deviceIterPrev = deviceIter;
        deviceIter++;

        disconnectedDevices.erase(deviceIterPrev);
      }
      else {
        now = Poco::Timestamp().epochMicroseconds();
        size_t elapsedSinceLastConnectionAttempt = (now - lastConnectionAttemptTime);
        if (elapsedSinceLastConnectionAttempt < device.asyncConnectInterval)
        {
          size_t remainingWaitTime = (device.asyncConnectInterval - elapsedSinceLastConnectionAttempt);
          if (remainingWaitTime < minRemainingWaitTime)
            minRemainingWaitTime = remainingWaitTime;
        }
        else {
          minRemainingWaitTime = 0;
        }

        lastConnectionAttemptTime = now;

        deviceIter++;
      }

      device.asyncMutex.unlock();
    }

    if ((!disconnectedDevices.empty()) && (minRemainingWaitTime>0))
    {
      Poco::Thread::sleep(minRemainingWaitTime / 1000);
    }
  }
}
#endif
