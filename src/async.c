#include "async.h"

//--------------------------------------------------------------
struct ftdi_transfer_control* AsyncWrite(struct mpsse_context *mpsse, uint8_t* buffer, size_t size)
{
  struct ftdi_transfer_control* tc = NULL;

  if (mpsse != NULL && mpsse->open)
  {
    if (mpsse->mode)
    {
      if (size <= mpsse->xsize)
      {
        // Copy in the command for this block
        buffer[0] = mpsse->tx;

        tc = ftdi_write_data_submit(&mpsse->ftdi, buffer, size);
      }
      else {
        printf("requested transfer of size %ld is greater than max frame size %d",
               size, mpsse->xsize);
      }
    }
  }

  return tc;
}
