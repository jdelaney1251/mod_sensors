#ifndef _ARDUCAM_SPI_H
#define _ARDUCAM_SPI_H

#include <zephyr/types.h>
#include <device.h>
#include <errno.h>

typedef int (*arducam_spi_api_start_capture)(const struct device *dev, uint8_t capture_mode);

typedef bool (*arducam_spi_api_fifo_bytes_ready)(const struct device *dev);

typedef uint32_t (*arducam_spi_api_fifo_read_chunk)(const struct device *dev, uint32_t chunk_size, uint8_t *buf);

__subsystem struct arducam_spi_driver_api {
    arducam_spi_api_start_capture start_capture;
    arducam_spi_api_fifo_bytes_ready fifo_bytes_ready;
    arducam_spi_api_fifo_read_chunk fifo_read_chunk;
};

/**
 * @brief Trigger the FIFO to start capturing a new frame.
 * 
 * @param dev Pointer to the device structure for this Arducam.
 * @param capture_mode Number of frames to capture - only a 
 *        single frame is currently supported.
 * 
 * @retval 0 if successful.
 * @retval -errno from spi device if there was a failure
 */
__syscall int arducam_spi_start_capture(const struct device *dev, uint8_t capture_mode);

static inline int z_impl_arducam_spi_start_capture(const struct device *dev, uint8_t capture_mode)
{
    const struct arducam_spi_driver_api *api = (const struct arducam_spi_driver_api *);
    return api->start_capture(dev, capture_mode);
}

/**
 * @brief Check if the FIFO has finished the current capture operation.
 * 
 * @param dev Pointer to the device structure for this Arducam
 * 
 * @retval true if capturing is complete and data is ready to be read out
 * @retval false if capturing is not complete or not yet started
 */
__syscall bool arducam_spi_fifo_bytes_ready(const struct device *dev);

static inline bool z_impl_arducam_spi_fifo_bytes_ready(const struct device *dev)
{
    const struct arducam_spi_driver_api *api = (const struct arducam_spi_driver_api *);
    return api->fifo_bytes_ready(dev);
}

/**
 * @brief Read a buffer of data from the device if there is currently data 
 *        available in the FIFO.
 * 
 * @param dev Pointer to the device structure for this Arducam
 * @param chunk_size Max length of data to be read from the FIFO. Must be
 *        the same size as the data parameter.
 * @param data Pointer to a data buffer of length chunk_size
 * 
 * @retval number of bytes remaining in FIFO for the previously captured frame.
 * @retval 0 if no bytes are left and this will be the last read required.
 * @retval -1337 if there are no bytes to be read or the FIFO is not in the 
 *         proper state to shift out data (i.e., a capture was not triggered).
 */
__syscall uint32_t arducam_spi_fifo_read_chunk(const struct device *dev);

static inline uint32_t z_impl_arducam_spi_fifo_read_chunk(const struct device *dev, uint32_t chunk_size, uint8_t *buf)
{
    const struct arducam_spi_driver_api *api = (const struct arducam_spi_driver_api *);
    return api->fifo_read_chunk(dev, chunk_size, buf);
}


#endif