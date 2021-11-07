#ifndef _SW_DCMI_H_
#define _SW_DCMI_H_

#include <zephyr/types.h>
#include <device.h>
#include <errno.h>

typedef int (*sw_dcmi_api_configure_cam_t)(const struct device *dev, 
                                        const struct device *cam_dev);

typedef int (*sw_dcmi_api_capture_image_t)(const struct device *dev);

__subsystem struct sw_dcmi_driver_api {
    sw_dcmi_api_configure_cam_t configure_cam;
    sw_dcmi_api_capture_image_t capture_image;
};

__syscall int sw_dcmi_configure_cam(const struct device *dev,
                                        const struct device *cam_dev);

static inline int z_impl_sw_dcmi_configure_cam(const struct device *dev,
                                                const struct device *cam_dev)
{
    const struct sw_dcmi_driver_api *api = (const struct sw_dcmi_driver_api *)dev->api;
    return api->configure_cam;
}

__syscall int sw_dcmi_capture_image(const struct device *dev);

static inline int z_impl_sw_dcmi_capture_image(const struct device *dev)
{
    const struct sw_dcmi_driver_api *api = (const struct sw_dcmi_driver_api *)dev->api;
    return api->capture_image;
}

#include <syscalls/qspi_ram.h>
#endif