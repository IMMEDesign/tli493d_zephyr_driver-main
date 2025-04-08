#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#define DT_DRV_COMPAT infineon_tli493d

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "tli493d hall sensor driver enabled without any devices"
#endif

// #define ADDR 0x35

struct tli493d_data {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temp;

};

struct tli493d_config {
    struct i2c_dt_spec i2c;
};

static int tli493d_init(const struct device *dev)
{
    struct tli493d_data *data = dev->data;
    struct tli493d_config *cfg = dev->config;
    uint8_t write_buf[3];
    uint8_t read_buf[1];
    uint8_t dbg_unused1;        // !dbg!
    
    write_buf[0] = 0x10;
    write_buf[1] = 0x00;
    write_buf[2] = 0x19;

    i2c_write_dt(&cfg->i2c, write_buf, 3);
    
    return 0;
}

static int tli493d_sample_fetch(const struct device *dev, enum sensor_channel chan) 
{
    int ret;
    struct tli493d_data *data = dev->data;
    struct tli493d_config *cfg = dev->config;
    uint8_t buf[6];
    uint8_t w_buf[1];

    w_buf[0] = 0x20;

    ret = i2c_write_dt(&cfg->i2c, w_buf, 1);
    if(ret < 0)
    {
        return -1;
    }
    ret = i2c_read_dt(&cfg->i2c, buf, 6);
    if(ret < 0)
    {
        return -1;
    }
    data->x = (int16_t)((buf[0] << 8) | (buf[4] & 0xF0)) >> 4;
    data->y = (int16_t)((buf[1] << 8) | ((buf[4] & 0x0F) << 4)) >> 4;
    data->z = (int16_t)((buf[2] << 8) | ((buf[5] & 0x0F) << 4)) >> 4;
    data->temp = (int16_t)((buf[3] << 4) | (buf[5] >> 4));

    // convert data in buffer to signed 12-bit integers
    return 0;

}

static int tli493d_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct tli493d_data *data = dev->data;
	switch(chan)
    {
        case SENSOR_CHAN_MAGN_X:
            val->val1 = data->x;
            break;
        case SENSOR_CHAN_MAGN_Y:
            val->val1 = data->y;
            break;
        case SENSOR_CHAN_MAGN_Z:
            val->val1 = data->z;
            break;
        case SENSOR_CHAN_GAUGE_TEMP:
            val->val1 = data->temp;
            break;
        default:
            return -1;
    }    
	return 0;
}

static const struct sensor_driver_api tli493d_api = 
{
    .sample_fetch = &tli493d_sample_fetch,
    .channel_get = &tli493d_channel_get,
};

#define tli493d_DEFINE(inst)												\
	static struct tli493d_data tli493d_data##inst;                    		\
																			\
	static const struct tli493d_config tli493d_config##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),									\
	};  																	\
	DEVICE_DT_INST_DEFINE(inst,												\
				tli493d_init,												\
				NULL,														\
				&tli493d_data##inst,										\
				&tli493d_config##inst,										\
				POST_KERNEL, 												\
				CONFIG_SENSOR_INIT_PRIORITY, 								\
				&tli493d_api);

DT_INST_FOREACH_STATUS_OKAY(tli493d_DEFINE)

