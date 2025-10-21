#include "budget.h"

#include <stdio.h>
#include <string.h>

#include "fsai_clock.h"

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

static void fsai_budget_emit_line(const fsai_budget_entry* entry, const char* stage_name,
                                  uint64_t duration_ns) {
    if (!g_budget_enabled || !entry || !entry->configured) {
        return;
    }

    const double duration_ms = fsai_clock_to_seconds(duration_ns) * 1000.0;
    const double budget_ms = fsai_clock_to_seconds(entry->budget_ns) * 1000.0;
    const double usage_pct = (entry->budget_ns > 0)
                                 ? (static_cast<double>(duration_ns) * 100.0) / entry->budget_ns
                                 : 0.0;
    const int over_budget = (entry->budget_ns > 0 && duration_ns > entry->budget_ns) ? 1 : 0;

    if (stage_name && stage_name[0] != '\0') {
        printf("[Timing][%s:%s] %.3f ms (budget %.3f ms, %.1f%%)%s\n",
               entry->name ? entry->name : "<unnamed>", stage_name,
               duration_ms, budget_ms, usage_pct,
               over_budget ? " !! over budget" : "");
    } else {
        printf("[Timing][%s] %.3f ms (budget %.3f ms, %.1f%%)%s\n",
               entry->name ? entry->name : "<unnamed>",
               duration_ms, budget_ms, usage_pct,
               over_budget ? " !! over budget" : "");
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

    fsai_budget_emit_line(entry, NULL, duration_ns);
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
    fsai_budget_emit_line(entry, stage_name, duration_ns);
}

void fsai_budget_report_all(void) {
    if (!g_budget_enabled) {
        return;
    }

    printf("========== Timing Summary =========\n");
    for (int i = 0; i < FSAI_BUDGET_SUBSYSTEM_COUNT; ++i) {
        const fsai_budget_entry* entry = &g_budget_entries[i];
        if (!entry->configured) {
            continue;
        }
        printf("[%s] ", entry->name ? entry->name : "<unnamed>");
        if (entry->sample_count == 0) {
            if (entry->unimplemented_note) {
                printf("no samples recorded (%s)\n", entry->unimplemented_note);
            } else {
                printf("no samples recorded\n");
            }
            continue;
        }
        const double avg_ms = fsai_clock_to_seconds(entry->total_duration_ns) * 1000.0 /
                              (entry->sample_count > 0 ? entry->sample_count : 1);
        const double worst_ms = fsai_clock_to_seconds(entry->worst_duration_ns) * 1000.0;
        const double budget_ms = fsai_clock_to_seconds(entry->budget_ns) * 1000.0;
        printf("avg %.3f ms, worst %.3f ms, budget %.3f ms\n", avg_ms, worst_ms, budget_ms);
    }
    printf("====================================\n");
}

