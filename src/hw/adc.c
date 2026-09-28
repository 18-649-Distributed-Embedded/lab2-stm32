#include "hw/adc.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include "core/log.h"

#define ADC_ACQ_TIME_SLOW ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 480)

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

static const struct adc_channel_cfg l_isense_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL, // STM32 uses internal VDDA (3.3V)
    .acquisition_time = ADC_ACQ_TIME_SLOW,
    .channel_id       = 10,               // PC0 = IN10
};

static const struct adc_channel_cfg r_isense_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_SLOW,
    .channel_id       = 11,               // PC1 = IN11
};

static const struct adc_channel_cfg s_isense_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_SLOW,
    .channel_id       = 8,                // PB0 = IN8
};

int adc_init(void)
{
    if (!device_is_ready(adc_dev)) {
        DEBUG_PRINT("Error: ADC device is not ready\n");
        return -1;
    }
    adc_channel_setup(adc_dev, &l_isense_cfg);
    adc_channel_setup(adc_dev, &r_isense_cfg);
    adc_channel_setup(adc_dev, &s_isense_cfg);
    return 0;
}

static int32_t read_voltage(const struct adc_channel_cfg *cfg)
{
    int err;
    uint16_t buf;
    int32_t val_mv;

    struct adc_sequence seq = {
        .channels    = BIT(cfg->channel_id),
        .buffer      = &buf,
        .buffer_size = sizeof(buf),
        .resolution  = 12, // STM32F4 max resolution is 12-bit
    };

    err = adc_read(adc_dev, &seq);
    if (err < 0) {
        return -1;
    }

    val_mv = buf;
    adc_raw_to_millivolts(3300, cfg->gain, seq.resolution, &val_mv);
    return val_mv;
}

int32_t adc_get_left_current_mv(void)
{
    return read_voltage(&l_isense_cfg);
}

int32_t adc_get_right_current_mv(void)
{
    return read_voltage(&r_isense_cfg);
}

int32_t adc_get_servo_current_mv(void)
{
    return read_voltage(&s_isense_cfg);
}
