
#include <stdlib.h>
#include <drivers/adc.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include <../drivers/adc/adc_context.h>

// do we need to use this adc_context api???
//#include <drivers/adc/adc_context.h>

LOG_MODULE_REGISTER(adc_mcp3424, CONFIG_ADC_LOG_LEVEL);

#define MCP3424_NUM_CHANNELS    4

#define MCP3424_CFG_BIT_RDY         7
#define MCP3424_CFG_BIT_CHN_SEL     6
#define MCP3424_CFG_BIT_CONV_MODE   4
#define MCP3424_CFG_BIT_SMPL_RATE   3
#define MCP3424_CFG_BIT_PGA_GAIN    1

#define MCP3424_CFG_CONV_MODE_CONT                  1
#define MCP3424_CFG_CONV_MODE_ONE_SHOT              0

#define MCP3424_CFG_SMPL_RATE_240                   0x0  // 12 bit resolution
#define MCP3424_CFG_SMPL_RATE_60                    0x1  // 14 bit resolution
#define MCP3424_CFG_SMPL_RATE_15                    0x2  // 16 bit resolution
#define MCP3424_CFG_SMPL_RATE_3_75                  0x3  // 18 bit resolution

#define MCP3424_CFG_PGA_GAIN_1x                     0x0
#define MCP3424_CFG_PGA_GAIN_2x                     0x1
#define MCP3424_CFG_PGA_GAIN_4x                     0x2
#define MCP3424_CFG_PGA_GAIN_8x                     0x3



struct mcp3424_config {
    const struct device *i2c_dev;
    //uint8_t channels;
    uint8_t i2c_addr;
};

struct mcp3424_data {
    struct adc_context ctx;
    const struct device *dev;
    int32_t *buffer;
    int32_t *repeat_buffer;
    uint8_t channels;
    uint8_t resolution;
    struct k_thread thread;
    struct k_sem sem;
    struct adc_channel_cfg channel_cfg[4];

    K_KERNEL_STACK_MEMBER(stack,
			CONFIG_ADC_MCP3424_ACQUISITION_THREAD_STACK_SIZE);
};

static int mcp3424_channel_setup(const struct device *dev, 
                                const struct adc_channel_cfg *channel_cfg)
{
    return 0;
}

static int mcp3424_start_read(const struct device *dev, 
                            const struct adc_sequence *sequence)
{
    const struct mcp3424_config *config = dev->config;
    struct mcp3424_data *data = dev->data;
    int err;

    data->resolution = sequence->resolution;
    data->buffer = sequence->buffer;

    if (find_msb_set(sequence->channels) > MCP3424_NUM_CHANNELS)
    {
        LOG_ERR("unsupported channels in mask");
        return -ENOTSUP;
    }

    adc_context_start_read(&data->ctx, sequence);

    return adc_context_wait_for_completion(&data->ctx);
}

static int mcp3424_read_async(const struct device *dev, 
                            const struct adc_sequence *sequence, 
                            struct k_poll_signal *async)
{
    struct mcp3424_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = mcp3424_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int mcp3424_read(const struct device *dev, 
                    const struct adc_sequence *sequence)
{
    return mcp3424_read_async(dev, sequence, NULL);
}

static int mcp3424_read_channel(const struct device *dev, 
                                uint8_t channel, 
                                int32_t *result)
{
    const struct mcp3424_config *config = dev->config;
    struct mcp3424_data *data = dev->data;
    int err;
    uint8_t cfg_byte = 0;
    uint8_t *rx_bytes;
    uint8_t rx_len = 0;

    switch (data->resolution)
    {
        case 12: 
            rx_len = 3;
            cfg_byte |= (MCP3424_CFG_SMPL_RATE_240 << MCP3424_CFG_BIT_SMPL_RATE);
            break;
        case 14:
            rx_len = 3;
            cfg_byte |= (MCP3424_CFG_SMPL_RATE_60 << MCP3424_CFG_BIT_SMPL_RATE);
            break;
        case 16:
            rx_len = 3;
            cfg_byte |= (MCP3424_CFG_SMPL_RATE_15 << MCP3424_CFG_BIT_SMPL_RATE);
            break;
        case 18:
            rx_len = 4;
            cfg_byte |= (MCP3424_CFG_SMPL_RATE_3_75 << MCP3424_CFG_BIT_SMPL_RATE);
            break;
    }

    cfg_byte |= ((channel-1) << MCP3424_CFG_BIT_CHN_SEL);
    cfg_byte &= ~(BIT(MCP3424_CFG_BIT_CONV_MODE));
    
    switch (data->channel_cfg[channel].gain)
    {
        case ADC_GAIN_1: cfg_byte |= (MCP3424_CFG_PGA_GAIN_1x << MCP3424_CFG_BIT_PGA_GAIN);
            break;
        case ADC_GAIN_2: cfg_byte |= (MCP3424_CFG_PGA_GAIN_2x << MCP3424_CFG_BIT_PGA_GAIN);
            break;
        case ADC_GAIN_4: cfg_byte |= (MCP3424_CFG_PGA_GAIN_4x << MCP3424_CFG_BIT_PGA_GAIN);
            break;
        case ADC_GAIN_8: cfg_byte |= (MCP3424_CFG_PGA_GAIN_8x << MCP3424_CFG_BIT_PGA_GAIN);
            break;
            
    }

    cfg_byte |= (1 << MCP3424_CFG_BIT_RDY);

    rx_bytes = malloc(sizeof(uint8_t)*rx_len);

    err = i2c_write(config->i2c_dev, &cfg_byte, 1, config->i2c_addr);

    while ((cfg_byte & BIT(MCP3424_CFG_BIT_RDY)))
    {
        err = i2c_read(config->i2c_dev, rx_bytes, rx_len, config->i2c_addr);

        cfg_byte = rx_bytes[rx_len - 1];
    }

    if (data->resolution > 16)
    {
        *result = sys_get_be24(rx_bytes);
    }
    else {
        *result = sys_get_be16(rx_bytes);
    }

    *result &= 0x0003FFFF; // zero out bits 19-24
    // check if sign bit (MSB) is set
    if (*result & (1 << (data->resolution-1)))
    {
        *result &= 0x0001FFFF;
        *result = -1 * (int32_t)*result;
    }

    free(rx_bytes);

    return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct mcp3424_data *data = CONTAINER_OF(ctx, struct mcp3424_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	k_sem_give(&data->sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct mcp3424_data *data = CONTAINER_OF(ctx, struct mcp3424_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void mcp3424_acquisition_thread(struct mcp3424_data *data)
{
    int32_t result = 0;
    uint8_t channel;
    int err;

    while (true)
    {
        k_sem_take(&data->sem, K_FOREVER);

        while (data->channels)
        {
            channel = find_lsb_set(data->channels) - 1;

            LOG_DBG("reading channel %d", channel);

			err = mcp3424_read_channel(data->dev, channel, &result);
			if (err) {
				LOG_ERR("failed to read channel %d (err %d)",
					channel, err);
				adc_context_complete(&data->ctx, err);
				break;
			}

			//LOG_DBG("read channel %d, result = %d", channel, result);

			*data->buffer++ = result;
			WRITE_BIT(data->channels, channel, 0);
		}

		adc_context_on_sampling_done(&data->ctx, data->dev);
    }
}

static int mcp3424_init(const struct device *dev)
{
    struct mcp3424_config *config = dev->config;
    struct mcp3424_data *data = dev->data;

    data->dev = dev;

    k_sem_init(&data->sem, 0, 1);

    if (!device_is_ready(config->i2c_dev))
    {
        LOG_ERR("I2C device not ready");
        return -EINVAL;
    }

    // put the device into one-shot conversion mode (i.e. no continuous conversion)
    uint8_t cfg_byte[1];
    int err;

    cfg_byte[0] = 0x06;
    err = i2c_write(config->i2c_dev, cfg_byte, 1, 0x00);
    if (err != 0)
    {
        LOG_ERR("error writing to i2c device: %d", err);
    }

    cfg_byte[0] = 0x00;
    err = i2c_write(config->i2c_dev, &cfg_byte, 1, config->i2c_addr);
    if (err != 0)
    {
        LOG_ERR("error writing to i2c device: %d", err);
    }

    k_thread_create(&data->thread, data->stack, 
                    CONFIG_ADC_MCP3424_ACQUISITION_THREAD_STACK_SIZE,
                    (k_thread_entry_t)mcp3424_acquisition_thread,
                    data, NULL, NULL,
                    CONFIG_ADC_MCP3424_ACQUISITION_THREAD_PRIO,
                    0, K_NO_WAIT);
    
    adc_context_unlock_unconditionally(&data->ctx);

    return 0;
}

struct adc_driver_api mcp3424_adc_api = {
    .channel_setup = mcp3424_channel_setup,
    .read = mcp3424_read
};

#define INST_DT_MCP3424(inst) DT_INST(inst, microchip_mcp3424)

#define MCP3424_DEVICE(n) \
    static struct mcp3424_data mcp3424_data_##n = { \
        ADC_CONTEXT_INIT_TIMER(mcp3424_data_##n, ctx), \
        ADC_CONTEXT_INIT_LOCK(mcp3424_data_##n, ctx), \
        ADC_CONTEXT_INIT_SYNC(mcp3424_data_##n, ctx), \
    }; \
    static const struct mcp3424_config mcp3424_config_##n = { \
        .i2c_dev = DEVICE_DT_GET(DT_BUS(INST_DT_MCP3424(n))), \
        .i2c_addr = DT_REG_ADDR(INST_DT_MCP3424(n)), \
    }; \
    DEVICE_DT_DEFINE(INST_DT_MCP3424(n), \
            &mcp3424_init, NULL, \
            &mcp3424_data_##n, \
            &mcp3424_config_##n, POST_KERNEL, \
            CONFIG_ADC_MCP3424_INIT_PRIORITY, \
            &mcp3424_adc_api)

#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_MCP3424_FOREACH(inst_expr) \
    UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(microchip_mcp3424), CALL_WITH_ARG, inst_expr)

INST_DT_MCP3424_FOREACH(MCP3424_DEVICE);
