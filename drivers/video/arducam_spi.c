#include <zephyr.h>
#include <device.h>
#include <sys/util.h>
#include <drivers/video/arducam_spi.h>
#include <drivers/spi.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(arducam_spi_cam, CONFIG_SPI_CAM_LOG_LEVEL);

#define DT_DRV_COMPAT arducam_spi_cam

// Register definitions
#define RWBIT_MASK                  0x80
#define CMD_RD_REG                  (0 << RWBIT_MASK)
#define CMD_WR_REG                  (1 << RWBIT_MASK)

#define REG_TEST                    0x00
#define REG_BUS_MODE                0x02
#define REG_SENSOR_TIMING           0x03
#define REG_FIFO_CTRL               0x04
#define REG_GPIO_DIR                0x05
#define REG_GPIO_WR                 0x06
#define REG_BURST_FIFIO_RD          0x3C
#define REG_SINGLE_FIFO_RD          0x3D
#define REG_LCD_CTRL_RS_0           0x3E
#define REG_LCD_CTRL_RS_1           0x3F
#define REG_ARDUCHIP_VER            0x40
#define REG_TRIG_SRC                0x41
#define REG_FIFO_SIZE_1             0x42
#define REG_FIFO_SIZE_2             0x43
#define REG_FIFO_SIZE_3             0x44
#define REG_GPIO_RD                 0x45

#define BUS_MODE_MCU2LCD            0x00
#define BUS_MODE_CAM2LCD            0x01
#define BUS_MODE_LCD2MCU            0x02

#define SENSOR_TIMING_HREF_MASK     0x01
#define SENSOR_TIMING_VREF_MASK     0x02
#define SENSOR_TIMING_LCD_BKEN      0x04
#define SENSOR_TIMING_PCLK_REV      0x08

#define FIFO_CTRL_CLEAR_MASK        0x01
#define FIFO_CTRL_START_MASK        0x02
#define FIFO_CTRL_WRITE
#define FIFO_CTRL_WRPTR_RST_MASK    0x10
#define FIFO_CTRL_RDPTR_RST_MASK    0x20

#define GPIO_RST_MASK               0x01
#define GPIO_PWDN_MASK              0x02
#define GPIO_PWREN_MASK             0x04

#define TRIG_SRC_VSYNC_STAT_MASK    0x01
#define TRIG_SRC_WR_FIFO_DONE_MASK  0x08


#define FIFO_STATE_IDLE             0x00
#define FIFO_STATE_CAPTURING        0x01
#define FIFO_STATE_BYTES_READY      0x02


struct arducam_spi_data {
    const struct device *spi_dev;
    const struct spi_config *spi_cfg;

    struct spi_buf cmd_wr;
    struct spi_buf cmd_rd;

    uint8_t fifo_state;
    uint32_t fifo_bytes;
};

static int reg_write(const struct device *dev, uint8_t reg, uint8_t wr_data)
{
    struct arducam_spi_data *data = dev->data;
    uint8_t ret;

    data->cmd_wr.buf[0] = CMD_WR_REG | reg;
    data->cmd_wr.buf[1] = data;
    ret = spi_transceive(data->spi_dev, data->spi_cfg, data->cmd_wr, NULL);
    if (ret != 0)
    {
        LOG_ERR("reg read err");
        return ret;
    }
    return ret;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *rd_data)
{
    struct arducam_spi_data *data = dev->data;
    uint8_t ret;

    data->cmd_rd.buf[0] = CMD_RD_REG | reg;
    data->cmd_rd.buf[1] = 0x00;
    ret = spi_transceive(data->spi_dev, data->spi_cfg, data->cmd_wr, data->cmd_rd);
    if (ret != 0)
    {
        LOG_ERR("reg read err");
        return ret;
    }
    *rd_data = data->cmd_rd.buf[1];
    return ret;
}

static int _start_capture(const struct device *dev, uint8_t capture_mode)
{
    struct arducam_spi_data *data = dev->data;
    uint8_t ret;

    ret = write_reg(REG_FIFO_CTRL, FIFO_CTRL_CLEAR_MASK);
    if (ret != 0)
    {
        LOG_ERR("error flushing fifo");
        return ret;
    }

    ret = write_reg(REG_FIFO_CTRL, FIFO_CTRL_WRPTR_RST_MASK | FIFO_CTRL_WRPTR_RST_MASK);
    if (ret != 0)
    {
        LOG_ERR("error resetting fifo pointers");
        return ret;
    }
    
    ret = write_reg(REG_FIFO_CTRL, FIFO_CTRL_START_MASK);
    if (ret != 0)
    {
        LOG_ERR("error triggering capture start");
        return ret;
    }

    data->fifo_state = FIFO_STATE_CAPTURING;
    
    return 0;
}

static bool _fifo_bytes_ready(const struct device *dev)
{
    struct arducam_spi_data *data = dev->data;
    uint8_t ret;

    uint8_t trigger_reg = 0;
    ret = read_reg(dev, REG_TRIG_SRC, trigger_reg);
    if (ret != 0)
    {
        LOG_ERR("failed to read trig src register");
        return false;
    }   

    if (trigger_reg & TRIG_SRC_WR_FIFO_DONE_MASK)
    {
        data->fifo_state = FIFO_STATE_BYTES_READY;
        return true;
    }

    data->fifo_state = FIFO_STATE_BYTES_READY;
    return false;
}

static uint32_t _fifo_read_chunk(const struct device *dev, uint32_t chunk_size, uint8_t *data)
{
    struct arducam_spi_data *data = dev->data;
    uint8_t ret;

    if (data->fifo_state == FIFO_STATE_BYTES_READY)
    {
        uint32_t rem_bytes = 0;
        uint8_t b0, b1, b2 = 0;
        struct spi_buf rx_buf = {.buf = data, .len = chunk_size};

        ret = spi_transceive(data->spi_dev, data->spi_cfg, NULL, rx_buf);
        if (ret != 0)
        {
            LOG_ERR("failed to read chunk of data from fifo");
            return ret;
        }

        ret = read_reg(REG_FIFO_SIZE_1, b0);
        ret = read_reg(REG_FIFO_SIZE_1, b1);
        ret = read_reg(REG_FIFO_SIZE_1, b2);
        b2 &= 0x7f;
        rem_bytes = ((b0) | (b1 << 8) | (b2 << 16)) & 0x007ffff;
        return rem_bytes;
    }

    // FIFO isn't in the necessary state to shift out data.
    return -1337;
}



static const struct arducam_spi_driver_api arducam_spi_api = {
    .start_capture = _start_capture,
    .fifo_bytes_ready = _fifo_bytes_ready,
    .fifo_read_chunk = _fifo_read_chunk,
};

static int arducam_spi_init(const struct device *dev)
{
    struct arducam_spi_data *data = dev->data;

    data->cmd_wr.buf = malloc(sizeof(uint8_t) * 2);
    data->cmd_wr.len = 2;
    data->cmd_rd.buf = malloc(sizeof(uint8_t) * 2);
    data->cmd_rd.len = 2;

    return 0;
}

#define INST_DT_SPI_CAM(inst) DT_INST(inst, DT_DRV_COMPAT)

#define SPI_CAM_DEVICE(n) \
    static struct arducam_spi_data arducam_spi_data_##n = { \
        .spi_dev = DEVICE_DT_GET(DT_INST_BUS(inst)), \
        .spi_cfg = SPI_CONFIG_DT_INST(DT_INST_BUS(inst), SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB | , 0), \
    }; \
    DEVICE_DT_DEFINE(INST_DT_SPI_CAM(n), \
            &arducam_spi_init, NULL, \
            &arducam_spi_data_##n, \
            NULL, POST_KERNEL, \
            CONFIG_ARDUCAM_SPI_INIT_PRIORITY, \
            &arducam_spi_api)

    // static const struct mcp3424_config mcp3424_config_##n = { \
    //     .i2c_dev = DEVICE_DT_GET(DT_BUS(INST_DT_MCP3424(n))), \
    //     .i2c_addr = DT_REG_ADDR(INST_DT_MCP3424(n)), \
    // }; \

#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_SPI_CAM_FOREACH(inst_expr) \
    UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT), CALL_WITH_ARG, inst_expr)

INST_DT_SPI_CAM_FOREACH(SPI_CAM_DEVICE);