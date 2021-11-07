#include <kernel.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>

#include <logging/log.h>

#include "bh1750.h"

LOG_MODULE_REGISTER(BH1750, CONFIG_SENSOR_LOG_LEVEL);

struct bh1750_data {
    const struct device *i2c;
    uint16_t calc_lux_value;
    uint8_t data_h_byte;
    uint8_t data_l_byte;
};

static void bh1750_calc_light_meas(struct bh1750_data *data)
{
    uint16_t tmp;
    tmp = (data->data_h_byte << 8) + (data->data_l_byte);
    tmp = tmp / 1.2;
    data->calc_lux_value = tmp;
}

static int bh1750_sample_fetch(const struct device *dev, 
                                enum sensor_channel chan)
{
    struct bh1750_data *drv_data = dev->data;
    int err;

    uint8_t cmd = BH1750_MODE | BH1750_MEAS_RES;
    uint8_t buf[2];
    err = i2c_write(drv_data->i2c, &cmd, 1, BH1750_I2C_ADDRESS);
    if (err < 0)
    {
        LOG_DBG("Failed to send measurement cmd");
        return err;
    }

    k_sleep(K_MSEC(BH1750_MEAS_WAIT_TIME_MS));

    err = i2c_read(drv_data->i2c, buf, 2, BH1750_I2C_ADDRESS);
    if (err < 0)
    {
        LOG_DBG("Read data failed");
        return err;
    }

    drv_data->data_h_byte = buf[0];
    drv_data->data_l_byte = buf[1];

    bh1750_calc_light_meas(drv_data);
    return 0;
}

static int bh1750_channel_get(const struct device *dev, 
                                enum sensor_channel chan,  
                                struct sensor_value *val)
{
    struct bh1750_data *data = dev->data;
    
    if (chan == SENSOR_CHAN_LIGHT)
    {
        val->val1 = data->calc_lux_value;
        val->val2 = 0;
    }
    else 
    {
        return  -EINVAL;
    }
    return 0;
}

static const struct sensor_driver_api bh1750_driver_api = {
    .sample_fetch = bh1750_sample_fetch,
    .channel_get = bh1750_channel_get,
};

int bh1750_init(const struct device *dev)
{
    struct bh1750_data *drv_data = dev->data;
    int err;

    drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
    if (drv_data->i2c == NULL) {
        LOG_DBG("Could not get pointer to %s device",
                DT_INST_BUS_LABEL(0));
        return -EINVAL;
    }
    uint8_t data = BH1750_CMD_PWR_DN;
    err = i2c_write(drv_data->i2c, &data, 1, BH1750_I2C_ADDRESS);
    if (err < 0)
    {
        LOG_DBG("Set device power down failed");
        return err;
    }

    LOG_DBG("\"%s\" OK", dev->name);
	return 0;
}

struct bh1750_data bh1750_driver;

DEVICE_DT_INST_DEFINE(0, bh1750_init, NULL, &bh1750_driver,
            NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
            &bh1750_driver_api);