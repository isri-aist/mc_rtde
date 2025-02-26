#pragma once

#include <stdint.h>

namespace mc_rtde
{

// Called in non real-time context to initialize the application
void * init(int argc, char * argv[], uint64_t & cycle_ns, const bool & interrupt);

// Main control-loop, when waiting for the next loop one should simply call sched_yield()
void run(void * data, const bool & interrupt);

} // namespace mc_rtde
