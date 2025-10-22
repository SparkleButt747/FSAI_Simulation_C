#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum fsai_clock_mode {
    FSAI_CLOCK_MODE_REALTIME = 0,
    FSAI_CLOCK_MODE_SIMULATED = 1
} fsai_clock_mode;

typedef struct fsai_clock_config {
    fsai_clock_mode mode;
    uint64_t start_time_ns;
} fsai_clock_config;

void fsai_clock_init(fsai_clock_config config);
void fsai_clock_shutdown(void);
uint64_t fsai_clock_now(void);
uint64_t fsai_clock_last_tick(void);
uint64_t fsai_clock_advance(uint64_t delta_ns);
uint64_t fsai_clock_set(uint64_t absolute_ns);
uint64_t fsai_clock_from_seconds(double seconds);
double fsai_clock_to_seconds(uint64_t nanoseconds);
int fsai_clock_is_simulated(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace fsai {
namespace time {
inline uint64_t fromSeconds(double seconds) { return fsai_clock_from_seconds(seconds); }
inline double toSeconds(uint64_t nanoseconds) { return fsai_clock_to_seconds(nanoseconds); }
inline uint64_t now() { return fsai_clock_now(); }
inline uint64_t advance(uint64_t delta_ns) { return fsai_clock_advance(delta_ns); }
inline bool isSimulated() { return fsai_clock_is_simulated() != 0; }
}
}
#endif

