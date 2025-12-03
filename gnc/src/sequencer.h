#ifndef CLOVER_SEQUENCER_H
#define CLOVER_SEQUENCER_H

#include <vector>

int sequencer_prepare(int gap, std::vector<float> fuel_bps, std::vector<float> lox_bps, bool mot_only);

int sequencer_prepare_sine(int total_time, float offset, float amplitude, float period, float phase);

int sequencer_prepare_combo(int gap, std::vector<float> fuel_bps, std::vector<float> lox_bps, std::vector<float> seq_sine_offsets_fuel, std::vector<float> seq_sine_amps_fuel, std::vector<float> seq_sine_periods_fuel,std::vector<float> seq_sine_phase_fuel,  std::vector<float> seq_sine_offsets_lox, std::vector<float> seq_sine_amps_lox, std::vector<float> seq_sine_periods_lox,std::vector<float> seq_sine_phase_lox, bool mot_only);

int sequencer_start_trace();

void sequencer_set_data_recipient(int sock);

void sequencer_halt();

#endif //CLOVER_SEQUENCER_H
