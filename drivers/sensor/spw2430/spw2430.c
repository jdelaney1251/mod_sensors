#include <stdlib.h>
#include <kernel.h>
#include <device.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/adc.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>

#include <logging/log.h>

#include "spw2430.h"
#include <drivers/sensor_chn.h>

LOG_MODULE_REGISTER(spw2430, CONFIG_SENSOR_LOG_LEVEL);

struct spw2430_config {
    const struct device *adc;
    uint8_t channel;
};

struct spw2430_data {
    struct adc_channel_cfg channel_cfg;
    struct adc_sequence sequence;
    int32_t data_buffer;
    uint32_t spl_dB;
};

static void spw2430_calc_spl(struct spw2430_data *data)
{
    LOG_INF("data val= %d", data->data_buffer);
    data->spl_dB = data->data_buffer;
}

static int spw2430_sample_fetch(const struct device *dev, 
                                enum sensor_channel chan)
{
    struct spw2430_data *data = dev->data;
    struct spw2430_config *config = dev->config;
    int err;

    err = adc_read(config->adc, &data->sequence);
    if (err != 0)
    {
        LOG_ERR("ADC reading failed with err %d", err);
        return err;
    }
    //data->sequence.buffer = &data->data_buffer;
    //data->data_buffer = 30;
    //*data->sequence.buffer = 40;
    //LOG_INF("seq buf: %d", *(uint32_t *)data->sequence.buffer);

    spw2430_calc_spl(data);
    return 0;
}

static int spw2430_channel_get(const struct device *dev, 
                                enum sensor_channel chan,  
                                struct sensor_value *val)
{
    struct spw2430_data *data = dev->data;

    if (chan == SENSOR_CHAN_SPL)
    {
        val->val1 = data->spl_dB;
    }
    else
    {
        return -EINVAL;
    }
    return 0;
}

static const struct sensor_driver_api spw2430_driver_api = {
    .sample_fetch = spw2430_sample_fetch,
    .channel_get = spw2430_channel_get,
};

static int spw2430_init(const struct device *dev)
{
    struct spw2430_data *data = dev->data;
    struct spw2430_config *config = dev->config;
    //const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

    if (!device_is_ready(config->adc))
    {
        LOG_ERR("specified adc device not found");
        return -EIO;
    }

    data->channel_cfg.channel_id = config->channel;
    data->channel_cfg.gain = ADC_GAIN_1;
    data->channel_cfg.reference = ADC_REF_INTERNAL;
    data->channel_cfg.differential = 0;
    data->channel_cfg.acquisition_time = 0;

    adc_channel_setup(config->adc, &data->channel_cfg);

    data->sequence.resolution = 18; // 18 bit resolution
    data->sequence.channels = config->channel;
    data->sequence.buffer = &data->data_buffer;
    data->sequence.buffer_size = sizeof(data->data_buffer);

    return 0;
}

#define INST_DT_SPW2430(inst) DT_INST(inst, knowles_spw2430)

#define SPW2430_DEVICE(n) \
    static struct spw2430_data spw2430_data_##n; \
    static const struct spw2430_config spw2430_config_##n = { \
        .adc = DEVICE_DT_GET(DT_PHANDLE(INST_DT_SPW2430(n), io_channels)), \
        .channel = DT_PHA(INST_DT_SPW2430(n), io_channels, channel), \
    }; \
    DEVICE_DT_DEFINE(INST_DT_SPW2430(n), \
            &spw2430_init, NULL, \
            &spw2430_data_##n, \
            &spw2430_config_##n, POST_KERNEL, \
            CONFIG_SENSOR_INIT_PRIORITY, \
            &spw2430_driver_api)

#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_SPW2430_FOREACH(inst_expr) \
    UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(knowles_spw2430), CALL_WITH_ARG, inst_expr)

INST_DT_SPW2430_FOREACH(SPW2430_DEVICE);
