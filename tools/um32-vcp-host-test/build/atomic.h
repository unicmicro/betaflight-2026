#pragma once

#define ATOMIC_BLOCK(priority) \
    for (bool run = true; run; run = false)
