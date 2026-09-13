#include "stats_domain.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstring>
#include <cstdlib>

namespace stats {

CpuLoadTracker::CpuLoadTracker(float ema_alpha) noexcept
    : m_ema_load(ema_alpha) {}

int CpuLoadTracker::sample() {
    int cpu_load_pct = m_last_pct;

#if (configGENERATE_RUN_TIME_STATS == 1 && configUSE_TRACE_FACILITY == 1)
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    if (task_count > 0) {
        TaskStatus_t* task_status_array = static_cast<TaskStatus_t*>(pvPortMalloc(task_count * sizeof(TaskStatus_t)));
        if (task_status_array) {
            uint32_t total_runtime_dummy = 0;
            UBaseType_t num_tasks = uxTaskGetSystemState(task_status_array, task_count, &total_runtime_dummy);
            uint32_t total_tasks_runtime = 0;
            uint32_t idle_runtime = 0;

            for (UBaseType_t i = 0; i < num_tasks; ++i) {
                total_tasks_runtime += task_status_array[i].ulRunTimeCounter;
                if (strncmp(task_status_array[i].pcTaskName, "IDLE", 4) == 0) {
                    idle_runtime += task_status_array[i].ulRunTimeCounter;
                }
            }
            vPortFree(task_status_array);

            if (m_has_prev_runtime) {
                uint32_t delta_total = total_tasks_runtime - m_last_total_runtime;
                uint32_t delta_idle = idle_runtime - m_last_idle_runtime;

                if (delta_total > 0 && delta_idle <= delta_total) {
                    uint32_t active_time = delta_total - delta_idle;
                    cpu_load_pct = static_cast<int>((static_cast<uint64_t>(active_time) * 100ULL + (delta_total / 2)) / delta_total);
                    if (cpu_load_pct > 100) cpu_load_pct = 100;
                    if (cpu_load_pct < 0) cpu_load_pct = 0;
                }
            } else {
                m_has_prev_runtime = true;
            }
            m_last_total_runtime = total_tasks_runtime;
            m_last_idle_runtime = idle_runtime;
        }
    }
#endif

    m_last_pct = cpu_load_pct;
    m_ema_load.update(static_cast<float>(cpu_load_pct));
    m_history.push(static_cast<float>(cpu_load_pct));
    return cpu_load_pct;
}

void CpuLoadTracker::reset() {
    m_last_total_runtime = 0;
    m_last_idle_runtime = 0;
    m_has_prev_runtime = false;
    m_last_pct = 0;
    m_ema_load.reset(0.0f);
    m_history.clear();
}

} // namespace stats

