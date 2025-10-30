#include <common/time/budget.h>
#include <common/time/fsai_clock.h>

#include <stdio.h>
#include <string.h>


typedef struct fsai_budget_entry {
    const char* name;
    uint64_t budget_ns;
    uint64_t last_duration_ns;
    uint64_t worst_duration_ns;
    uint64_t total_duration_ns;
    uint64_t sample_count;
    int configured;
    int note_printed;
    const char* unimplemented_note;
} fsai_budget_entry;

static fsai_budget_entry g_budget_entries[FSAI_BUDGET_SUBSYSTEM_COUNT];
static int g_budget_enabled = 1;

static int fsai_budget_valid_subsystem(fsai_budget_subsystem subsystem) {
    return subsystem >= 0 && subsystem < FSAI_BUDGET_SUBSYSTEM_COUNT;
}

void fsai_budget_init(void) {
    memset(g_budget_entries, 0, sizeof(g_budget_entries));
    g_budget_enabled = 1;
}

void fsai_budget_set_enabled(int enabled) {
    g_budget_enabled = enabled ? 1 : 0;
}

void fsai_budget_configure(fsai_budget_subsystem subsystem, const char* name, uint64_t budget_ns) {
    if (!fsai_budget_valid_subsystem(subsystem)) {
        return;
    }
    fsai_budget_entry* entry = &g_budget_entries[subsystem];
    entry->name = name;
    entry->budget_ns = budget_ns;
    entry->configured = 1;
    entry->last_duration_ns = 0;
    entry->worst_duration_ns = 0;
    entry->total_duration_ns = 0;
    entry->sample_count = 0;
    entry->note_printed = 0;
}

void fsai_budget_mark_unimplemented(fsai_budget_subsystem subsystem, const char* note) {
    if (!fsai_budget_valid_subsystem(subsystem)) {
        return;
    }
    fsai_budget_entry* entry = &g_budget_entries[subsystem];
    if (!entry->configured || entry->note_printed) {
        return;
    }
    entry->unimplemented_note = note;
    entry->note_printed = 1;
    if (g_budget_enabled) {
        printf("[Timing][%s] Note: %s\n", entry->name ? entry->name : "<unnamed>", note);
    }
}

void fsai_budget_record(fsai_budget_subsystem subsystem, uint64_t duration_ns) {
    if (!fsai_budget_valid_subsystem(subsystem)) {
        return;
    }
    fsai_budget_entry* entry = &g_budget_entries[subsystem];
    if (!entry->configured) {
        return;
    }

    entry->last_duration_ns = duration_ns;
    if (duration_ns > entry->worst_duration_ns) {
        entry->worst_duration_ns = duration_ns;
    }
    entry->total_duration_ns += duration_ns;
    entry->sample_count += 1;

}

void fsai_budget_stage_record(fsai_budget_subsystem subsystem, const char* stage_name,
                              uint64_t duration_ns) {
    if (!fsai_budget_valid_subsystem(subsystem)) {
        return;
    }
    const fsai_budget_entry* entry = &g_budget_entries[subsystem];
    if (!entry->configured) {
        return;
    }
}


