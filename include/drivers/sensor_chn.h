#ifndef _MOD_SENSORS_DRIVERS_SENSOR_CHN_H_
#define _MOD_SENSORS_DRIVERS_SENSOR_CHN_H_

#include <drivers/sensor.h>

enum mod_sensors_cust_channel {
    /* sound pressure level in dB */
    SENSOR_CHAN_SPL = SENSOR_CHAN_PRIV_START,
};

#endif