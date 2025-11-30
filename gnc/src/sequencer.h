#ifndef CLOVER_SEQUENCER_H
#define CLOVER_SEQUENCER_H

#include <vector>

int sequencer_prepare(int gap, std::vector<float> bps, bool motor_only);

int sequencer_prepare_sine(int total_time, float offset, float amplitude, float period, float phase);

int sequencer_start_trace();

void sequencer_set_data_recipient(int sock);

#endif //CLOVER_SEQUENCER_H
