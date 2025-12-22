#ifndef ARTY_SEQUENCER_H
#define ARTY_SEQUENCER_H

#include <vector>

int sequencer_prepare(int gap, std::vector<float> fuel_bps, std::vector<float> lox_bps, bool mot_only);

int sequencer_prepare_sine(int total_time, float offset, float amplitude, float period, float phase);

int sequencer_prepare_combo(
    int gap,
    const std::vector<float> &fuel_bps,
    const std::vector<float> &lox_bps,
    const std::vector<float> &seq_sine_offsets_fuel,
    const std::vector<float> &seq_sine_amps_fuel,
    const std::vector<float> &seq_sine_periods_fuel,
    const std::vector<float> &seq_sine_phases_fuel,
    const std::vector<float> &seq_sine_offsets_lox,
    const std::vector<float> &seq_sine_amps_lox,
    const std::vector<float> &seq_sine_periods_lox,
    const std::vector<float> &seq_sine_phases_lox,
    bool mot_only);
int sequencer_start_trace();

void sequencer_set_data_recipient(int sock);

int sequencer_get_data_recipient();

void sequencer_halt();

#endif //ARTY_SEQUENCER_H
