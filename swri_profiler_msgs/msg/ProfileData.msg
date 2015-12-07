uint32 key
# The corresponding key for this block reported in the profiler's index.

uint64 abs_call_count
# The number of times this block has been started since the profiler
# started.

duration abs_total_duration
# The total amount of time spent in this block since the profiler
# started, including any current calls.

duration rel_total_duration
# The amount of time spent in this block since the previous report.

duration rel_max_duration
# The maximum amount of time spent in this call since the last report.

