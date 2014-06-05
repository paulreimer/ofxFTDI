#pragma once

#include "mpsse.h"

int ZeroCopyWrite(struct mpsse_context *mpsse, uint8_t* buffer, size_t size);
