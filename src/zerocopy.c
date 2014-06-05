#include "zerocopy.h"

//--------------------------------------------------------------
int ZeroCopyWrite(struct mpsse_context *mpsse, uint8_t* buffer, size_t size)
{
  if (mpsse != NULL && mpsse->open)
  {
    if (mpsse->mode)
    {
      if (size <= mpsse->xsize)
      {
        // Copy in the command for this block
        buffer[0] = mpsse->tx;

        if (ftdi_write_data(&mpsse->ftdi, buffer, size) == size)
        {
          return MPSSE_OK;
        }
      }
      else {
        printf("requested transfer of size %lu is greater than max frame size %d",
               size, mpsse->xsize);
      }
    }
  }

  return MPSSE_FAIL;
}
