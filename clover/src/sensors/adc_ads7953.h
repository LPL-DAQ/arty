#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum adc_ads7953_channel {
    ADC_ADS7953_BANK1_AIN0 = 0,
    ADC_ADS7953_BANK1_AIN1,
    ADC_ADS7953_BANK2_AIN0,
    ADC_ADS7953_BANK2_AIN1,
    ADC_ADS7953_CHANNEL_COUNT,
};

int adc_ads7953_init(void);
int adc_ads7953_read(enum adc_ads7953_channel ch, int16_t *sample);
int adc_ads7953_read_all(int16_t samples[ADC_ADS7953_CHANNEL_COUNT]);
const char *adc_ads7953_channel_name(enum adc_ads7953_channel ch);

#ifdef __cplusplus
}
#endif
