#include "ofxMPSSE.h"
#include <iostream>

// List of known FT2232-based devices
struct vid_pid my_supported_devices[] = {
  { 0x0403, 0x6010, "FT2232 Future Technology Devices International, Ltd" },
//  { 0x0403, 0x6011, "FT4232 Future Technology Devices International, Ltd" },
//  { 0x0403, 0x6014, "FT232H Future Technology Devices International, Ltd" },
  
  { 0, 0, NULL }
};

//--------------------------------------------------------------
ofxMPSSE::ofxMPSSE()
: ftdi(NULL)
, connected(false)
{}

//--------------------------------------------------------------
ofxMPSSE::~ofxMPSSE()
{
  if (connected)
    Close(ftdi);
}

//--------------------------------------------------------------
bool
ofxMPSSE::connect(enum modes mode, int freq, int endianess, int index, interface iface)
{
  return connect(mode, freq, endianess, iface, NULL, NULL, index);
}

//--------------------------------------------------------------
bool
ofxMPSSE::connect(enum modes mode, int freq, int endianess, interface iface, char* description, char* serial, int index)
{
//  ftdi = MPSSE(mode, freq, endianess);
//	struct mpsse_context *mpsse = NULL;
  
	for(size_t i=0; my_supported_devices[i].vid != 0; i++)
	{
    printf("connect() index = %i\n", index);

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
    std::cout
    << "FTDI device '"      << GetDescription(ftdi)
    << "' initialized at "  << GetClock(ftdi) << "Hz"
    << std::endl;

    connected = true;
  }
  else {
    std::cout
    << "Unable to connect to FTDI device (" << ErrorString(ftdi) << "); "
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
    FastWrite(ftdi, (char*)data.data(), data.size());
//    Write(ftdi, (char*)data.data(), data.size());
}
