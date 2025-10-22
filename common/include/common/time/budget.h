#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum fsai_budget_subsystem {
    FSAI_BUDGET_SUBSYSTEM_SIMULATION = 0,
    FSAI_BUDGET_SUBSYSTEM_VISION = 1,
    FSAI_BUDGET_SUBSYSTEM_CONTROL = 2,
    FSAI_BUDGET_SUBSYSTEM_CAN = 3,
    FSAI_BUDGET_SUBSYSTEM_COUNT
} fsai_budget_subsystem;

void fsai_budget_init(void);
void fsai_budget_set_enabled(int enabled);
void fsai_budget_configure(fsai_budget_subsystem subsystem, const char* name, uint64_t budget_ns);
void fsai_budget_mark_unimplemented(fsai_budget_subsystem subsystem, const char* note);
void fsai_budget_record(fsai_budget_subsystem subsystem, uint64_t duration_ns);
void fsai_budget_stage_record(fsai_budget_subsystem subsystem, const char* stage_name, uint64_t duration_ns);
void fsai_budget_report_all(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "fsai_clock.h"

namespace fsai {
namespace time {

class ScopedBudgetTimer {
public:
    explicit ScopedBudgetTimer(fsai_budget_subsystem subsystem)
        : subsystem_(subsystem), stage_name_(nullptr), start_ns_(fsai_clock_now()) {}

    ScopedBudgetTimer(fsai_budget_subsystem subsystem, const char* stage_name)
        : subsystem_(subsystem), stage_name_(stage_name), start_ns_(fsai_clock_now()) {}

    ~ScopedBudgetTimer() {
        const uint64_t duration_ns = fsai_clock_now() - start_ns_;
        if (stage_name_) {
            fsai_budget_stage_record(subsystem_, stage_name_, duration_ns);
        } else {
            fsai_budget_record(subsystem_, duration_ns);
        }
    }

    ScopedBudgetTimer(const ScopedBudgetTimer&) = delete;
    ScopedBudgetTimer& operator=(const ScopedBudgetTimer&) = delete;

private:
    fsai_budget_subsystem subsystem_;
    const char* stage_name_;
    uint64_t start_ns_;
};

class VisionStageTimer : public ScopedBudgetTimer {
public:
    explicit VisionStageTimer(const char* stage_name = nullptr)
        : ScopedBudgetTimer(FSAI_BUDGET_SUBSYSTEM_VISION, stage_name) {}
};

class ControlStageTimer : public ScopedBudgetTimer {
public:
    explicit ControlStageTimer(const char* stage_name = nullptr)
        : ScopedBudgetTimer(FSAI_BUDGET_SUBSYSTEM_CONTROL, stage_name) {}
};

class SimulationStageTimer : public ScopedBudgetTimer {
public:
    explicit SimulationStageTimer(const char* stage_name = nullptr)
        : ScopedBudgetTimer(FSAI_BUDGET_SUBSYSTEM_SIMULATION, stage_name) {}
};

class CanStageTimer : public ScopedBudgetTimer {
public:
    explicit CanStageTimer(const char* stage_name = nullptr)
        : ScopedBudgetTimer(FSAI_BUDGET_SUBSYSTEM_CAN, stage_name) {}
};

}  // namespace time
}  // namespace fsai
#endif

