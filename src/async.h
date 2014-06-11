#pragma once

#include "mpsse.h"
#include <stddef.h>

struct ftdi_transfer_control* AsyncWrite(struct mpsse_context *mpsse, uint8_t* buffer, size_t size);
