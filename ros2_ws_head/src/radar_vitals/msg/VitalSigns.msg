# Vital signs estimated from radar phase. Research prototype, not a medical measurement.
std_msgs/Header header

bool warming_up              # Not enough data collected yet
bool person_detected         # A range bin shows much more motion than the rest of the scene
bool motion_detected         # Body movement is too large to measure; estimates are being held

float32 range_m              # Distance of the selected range bin; NaN when there is no person

# Rates are NaN until an estimate exists. Standard deviations come from the belief filter.
float32 heart_rate_bpm
float32 heart_rate_std_bpm
float32 heart_confidence     # 0 to 1: share of the belief close to the estimate

float32 breathing_rate_bpm
float32 breathing_rate_std_bpm
float32 breathing_confidence # 0 to 1

float32 breathing_amplitude_mm  # Recent motion in the breathing band; small during a breath-hold
float32 heart_amplitude_mm      # Recent motion in the heart band

uint32 missed_frames         # Radar frames lost since the node started
