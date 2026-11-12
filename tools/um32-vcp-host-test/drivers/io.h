#pragma once
#include "drivers/io_types.h"
#include "drivers/resource.h"
#define IO_TAG(pin) ((ioTag_t)0)
static inline IO_t IOGetByTag(ioTag_t tag) { (void)tag; return IO_NONE; }
static inline void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
    (void)io;
    (void)owner;
    (void)index;
}
