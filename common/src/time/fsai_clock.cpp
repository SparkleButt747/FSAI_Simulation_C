#include <common/time/fsai_clock.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>

namespace {
struct ClockState {
    std::atomic<bool> initialized{false};
    fsai_clock_mode mode{FSAI_CLOCK_MODE_REALTIME};
    uint64_t start_ns{0};
    std::chrono::steady_clock::time_point realtime_origin{};
    std::atomic<uint64_t> sim_now_ns{0};
    std::atomic<uint64_t> last_tick_ns{0};
    std::mutex mutex;
};

ClockState& clock_state() {
    static ClockState state;
    return state;
}

constexpr double kNsPerSec = 1e9;

void ensure_initialized() {
    ClockState& state = clock_state();
    if (!state.initialized.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(state.mutex);
        if (!state.initialized.load(std::memory_order_relaxed)) {
            state.mode = FSAI_CLOCK_MODE_REALTIME;
            state.start_ns = 0;
            state.realtime_origin = std::chrono::steady_clock::now();
            state.sim_now_ns.store(0, std::memory_order_relaxed);
            state.last_tick_ns.store(0, std::memory_order_relaxed);
            state.initialized.store(true, std::memory_order_release);
        }
    }
}

uint64_t realtime_now_ns(const ClockState& state) {
    const auto now = std::chrono::steady_clock::now();
    const auto delta = std::chrono::duration_cast<std::chrono::nanoseconds>(now - state.realtime_origin);
    return state.start_ns + static_cast<uint64_t>(delta.count());
}

}  // namespace

void fsai_clock_init(fsai_clock_config config) {
    ClockState& state = clock_state();
    std::lock_guard<std::mutex> lock(state.mutex);

    state.mode = config.mode;
    state.start_ns = config.start_time_ns;
    state.last_tick_ns.store(config.start_time_ns, std::memory_order_relaxed);

    if (config.mode == FSAI_CLOCK_MODE_REALTIME) {
        state.realtime_origin = std::chrono::steady_clock::now();
        state.sim_now_ns.store(0, std::memory_order_relaxed);
    } else {
        state.sim_now_ns.store(config.start_time_ns, std::memory_order_relaxed);
    }

    state.initialized.store(true, std::memory_order_release);
}

void fsai_clock_shutdown(void) {
    ClockState& state = clock_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.initialized.store(false, std::memory_order_release);
}

uint64_t fsai_clock_now(void) {
    ensure_initialized();
    ClockState& state = clock_state();
    uint64_t value = 0;
    if (state.mode == FSAI_CLOCK_MODE_REALTIME) {
        value = realtime_now_ns(state);
    } else {
        value = state.sim_now_ns.load(std::memory_order_acquire);
    }
    state.last_tick_ns.store(value, std::memory_order_release);
    return value;
}

uint64_t fsai_clock_last_tick(void) {
    ensure_initialized();
    ClockState& state = clock_state();
    return state.last_tick_ns.load(std::memory_order_acquire);
}

uint64_t fsai_clock_advance(uint64_t delta_ns) {
    ensure_initialized();
    ClockState& state = clock_state();
    if (state.mode == FSAI_CLOCK_MODE_REALTIME) {
        // In realtime mode we cannot advance manually; return current time.
        return fsai_clock_now();
    }
    uint64_t new_value = state.sim_now_ns.fetch_add(delta_ns, std::memory_order_acq_rel) + delta_ns;
    state.last_tick_ns.store(new_value, std::memory_order_release);
    return new_value;
}

uint64_t fsai_clock_set(uint64_t absolute_ns) {
    ensure_initialized();
    ClockState& state = clock_state();
    if (state.mode == FSAI_CLOCK_MODE_REALTIME) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.start_ns = absolute_ns;
        state.realtime_origin = std::chrono::steady_clock::now();
        state.last_tick_ns.store(absolute_ns, std::memory_order_release);
        return absolute_ns;
    }
    state.sim_now_ns.store(absolute_ns, std::memory_order_release);
    state.last_tick_ns.store(absolute_ns, std::memory_order_release);
    return absolute_ns;
}

uint64_t fsai_clock_from_seconds(double seconds) {
    if (seconds <= 0.0) {
        return 0ULL;
    }
    const double scaled = seconds * kNsPerSec;
    return static_cast<uint64_t>(std::llround(scaled));
}

double fsai_clock_to_seconds(uint64_t nanoseconds) {
    return static_cast<double>(nanoseconds) / kNsPerSec;
}

int fsai_clock_is_simulated(void) {
    ensure_initialized();
    ClockState& state = clock_state();
    return state.mode == FSAI_CLOCK_MODE_SIMULATED ? 1 : 0;
}

