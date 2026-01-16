#define DT_DRV_COMPAT solomon_ssd1362

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ssd1362, CONFIG_DISPLAY_LOG_LEVEL);

#define SSD1362_SET_COLUMN_ADDR        0x15
#define SSD1362_SET_FADE_MODE          0x23
#define SSD1362_SET_ROW_ADDR           0x75
#define SSD1362_SET_CONTRAST           0x81
#define SSD1362_SET_REMAP              0xA0
#define SSD1362_SET_START_LINE         0xA1
#define SSD1362_SET_DISPLAY_OFFSET     0xA2
#define SSD1362_MODE_NORMAL            0xA4
#define SSD1362_MODE_ALL_ON            0xA5
#define SSD1362_MODE_ALL_OFF           0xA6
#define SSD1362_MODE_INVERSE           0xA7
#define SSD1362_EXIT_PARTIAL           0xA9
#define SSD1362_SET_VDD                0xAB
#define SSD1362_SET_IREF               0xAD
#define SSD1362_DISPLAY_OFF            0xAE
#define SSD1362_DISPLAY_ON             0xAF
#define SSD1362_SET_PHASE_LENGTH       0xB1
#define SSD1362_SET_CLOCK_DIV          0xB3
#define SSD1362_SET_SECOND_PRECHARGE   0xB6
#define SSD1362_DEFAULT_GREYSCALE      0xB9
#define SSD1362_SET_PRECHARGE_VOLTAGE  0xBC
#define SSD1362_SET_PRECHARGE_CAP      0xBD
#define SSD1362_SET_VCOMH              0xBE
#define SSD1362_SET_MUX_RATIO          0xCA
#define SSD1362_COMMAND_LOCK           0xFD

#define SSD1362_COMMAND_LOCK_UNLOCK    0x12

#define SSD1362_BITS_PER_SEGMENT       4
#define SSD1362_SEGMENTS_PER_BYTE      (8 / SSD1362_BITS_PER_SEGMENT)

struct ssd1362_data {
	enum display_pixel_format current_pf;
};

struct ssd1362_config {
	const struct device *mipi_dev;
	struct mipi_dbi_config dbi_config;
	uint16_t height;
	uint16_t width;
	uint16_t column_offset;
	uint8_t row_offset;
	uint8_t start_line;
	uint8_t mux_ratio;
	bool remap_row_first;
	bool remap_columns;
	bool remap_rows;
	bool remap_nibble;
	bool remap_com_odd_even_split;
	bool remap_com_dual;
	uint8_t segments_per_pixel;
	uint8_t contrast;
	bool inversion_on;
	bool iref_external;
	uint8_t *conversion_buf;
	size_t conversion_buf_size;
};

static inline int ssd1362_write_command(const struct device *dev, uint8_t cmd, const uint8_t *buf,
					size_t len)
{
	const struct ssd1362_config *config = dev->config;
	int ret;

	ret = mipi_dbi_command_write(config->mipi_dev, &config->dbi_config, cmd, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	for (size_t i = 0; i < len; i++) {
		ret = mipi_dbi_command_write(config->mipi_dev, &config->dbi_config, buf[i], NULL, 0);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int ssd1362_blanking_on(const struct device *dev)
{
	return ssd1362_write_command(dev, SSD1362_MODE_ALL_OFF, NULL, 0);
}

static int ssd1362_blanking_off(const struct device *dev)
{
	const struct ssd1362_config *config = dev->config;
	uint8_t mode = config->inversion_on ? SSD1362_MODE_INVERSE : SSD1362_MODE_NORMAL;

	return ssd1362_write_command(dev, mode, NULL, 0);
}

static int ssd1362_conv_mono01_grayscale(const uint8_t **buf_in, uint32_t *pixel_count,
					 uint8_t *buf_out, size_t buf_out_size,
					 uint8_t segments_per_pixel)
{
	size_t out_idx = 0;
	size_t in_idx = 0;
	size_t max_input_bytes = *pixel_count / 8;

	while (in_idx < max_input_bytes && (out_idx + 4 * segments_per_pixel) <= buf_out_size) {
		uint8_t b = (*buf_in)[in_idx];

		for (size_t s = 0; s < segments_per_pixel; s++) {
			buf_out[out_idx++] = ((b & BIT(0)) ? 0x00 : 0xF0) | ((b & BIT(1)) ? 0x00 : 0x0F);
		}
		for (size_t s = 0; s < segments_per_pixel; s++) {
			buf_out[out_idx++] = ((b & BIT(2)) ? 0x00 : 0xF0) | ((b & BIT(3)) ? 0x00 : 0x0F);
		}
		for (size_t s = 0; s < segments_per_pixel; s++) {
			buf_out[out_idx++] = ((b & BIT(4)) ? 0x00 : 0xF0) | ((b & BIT(5)) ? 0x00 : 0x0F);
		}
		for (size_t s = 0; s < segments_per_pixel; s++) {
			buf_out[out_idx++] = ((b & BIT(6)) ? 0x00 : 0xF0) | ((b & BIT(7)) ? 0x00 : 0x0F);
		}

		in_idx++;
	}

	*buf_in += in_idx;
	*pixel_count -= in_idx * 8;
	return out_idx;
}

static int ssd1362_conv_rgb565_grayscale(const uint8_t **buf_in, uint32_t *pixel_count,
					 uint8_t *buf_out, size_t buf_out_size,
					 uint8_t segments_per_pixel)
{
	const uint16_t *rgb_in = (const uint16_t *)*buf_in;
	size_t out_idx = 0;
	size_t in_idx = 0;

	while (in_idx + 1 < *pixel_count && (out_idx + segments_per_pixel) <= buf_out_size) {
		uint16_t px1_raw = rgb_in[in_idx];
		uint16_t px2_raw = rgb_in[in_idx + 1];
#ifdef CONFIG_LV_COLOR_16_SWAP
		uint16_t px1 = (px1_raw >> 8) | (px1_raw << 8);
		uint16_t px2 = (px2_raw >> 8) | (px2_raw << 8);
#else
		uint16_t px1 = px1_raw;
		uint16_t px2 = px2_raw;
#endif

		uint8_t r1 = (px1 >> 11) & 0x1F;
		uint8_t g1 = (px1 >> 5) & 0x3F;
		uint8_t b1 = px1 & 0x1F;
		uint8_t y1 = (r1 * 3 + g1 * 6 + b1) >> 5;

		uint8_t r2 = (px2 >> 11) & 0x1F;
		uint8_t g2 = (px2 >> 5) & 0x3F;
		uint8_t b2 = px2 & 0x1F;
		uint8_t y2 = (r2 * 3 + g2 * 6 + b2) >> 5;

		for (size_t s = 0; s < segments_per_pixel; s++) {
			buf_out[out_idx++] = (y1 << 4) | y2;
		}

		in_idx += 2;
	}

	*buf_in += in_idx * 2;
	*pixel_count -= in_idx;
	return out_idx;
}

static int ssd1362_write_pixels(const struct device *dev, const uint8_t *buf,
				uint32_t pixel_count, enum display_pixel_format pf)
{
	const struct ssd1362_config *config = dev->config;
	struct display_buffer_descriptor mipi_desc;

	while (pixel_count > 0) {
		size_t len;
		int ret;

		if (pf == PIXEL_FORMAT_RGB_565) {
			len = ssd1362_conv_rgb565_grayscale(&buf, &pixel_count,
							   config->conversion_buf,
							   config->conversion_buf_size,
							   config->segments_per_pixel);
		} else {
			len = ssd1362_conv_mono01_grayscale(&buf, &pixel_count,
							   config->conversion_buf,
							   config->conversion_buf_size,
							   config->segments_per_pixel);
		}

		if (len == 0) {
			break;
		}

		mipi_desc.buf_size = len;
		mipi_desc.width = len * 8;
		mipi_desc.height = 1;
		mipi_desc.pitch = mipi_desc.width;
		ret = mipi_dbi_write_display(config->mipi_dev, &config->dbi_config,
					     config->conversion_buf, &mipi_desc,
					     PIXEL_FORMAT_MONO01);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static size_t ssd1362_conv_row_rgb565(const uint16_t *rgb_in, uint16_t width,
				      uint8_t *buf_out, uint8_t x_start_odd)
{
	size_t out_idx = 0;
	size_t in_idx = 0;

	if (x_start_odd && width > 0) {
		uint16_t px = rgb_in[in_idx++];
#ifdef CONFIG_LV_COLOR_16_SWAP
		px = (px >> 8) | (px << 8);
#endif
		uint8_t r = (px >> 11) & 0x1F;
		uint8_t g = (px >> 5) & 0x3F;
		uint8_t b = px & 0x1F;
		uint8_t y = (r * 3 + g * 6 + b) >> 5;

		buf_out[out_idx++] = y;
		width--;
	}

	while (width >= 2) {
		uint16_t px1 = rgb_in[in_idx++];
		uint16_t px2 = rgb_in[in_idx++];
#ifdef CONFIG_LV_COLOR_16_SWAP
		px1 = (px1 >> 8) | (px1 << 8);
		px2 = (px2 >> 8) | (px2 << 8);
#endif
		uint8_t r1 = (px1 >> 11) & 0x1F;
		uint8_t g1 = (px1 >> 5) & 0x3F;
		uint8_t b1 = px1 & 0x1F;
		uint8_t y1 = (r1 * 3 + g1 * 6 + b1) >> 5;

		uint8_t r2 = (px2 >> 11) & 0x1F;
		uint8_t g2 = (px2 >> 5) & 0x3F;
		uint8_t b2 = px2 & 0x1F;
		uint8_t y2 = (r2 * 3 + g2 * 6 + b2) >> 5;

		buf_out[out_idx++] = (y1 << 4) | y2;
		width -= 2;
	}

	if (width == 1) {
		uint16_t px = rgb_in[in_idx];
#ifdef CONFIG_LV_COLOR_16_SWAP
		px = (px >> 8) | (px << 8);
#endif
		uint8_t r = (px >> 11) & 0x1F;
		uint8_t g = (px >> 5) & 0x3F;
		uint8_t b = px & 0x1F;
		uint8_t y = (r * 3 + g * 6 + b) >> 5;

		buf_out[out_idx++] = (y << 4);
	}

	return out_idx;
}

static int ssd1362_write(const struct device *dev, const uint16_t x, const uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct ssd1362_config *config = dev->config;
	struct ssd1362_data *data = dev->data;
	int ret;
	uint8_t cmd_data[2];
	enum display_pixel_format pf;

	if (desc->pitch < desc->width) {
		LOG_ERR("Pitch is smaller than width");
		return -EINVAL;
	}

	if (buf == NULL || desc->buf_size == 0U) {
		LOG_ERR("Display buffer is not available");
		return -EINVAL;
	}

	pf = (data != NULL) ? data->current_pf : PIXEL_FORMAT_MONO01;

	if (pf == PIXEL_FORMAT_RGB_565) {
		const uint16_t *rgb_buf = (const uint16_t *)buf;
		uint16_t x_aligned = x & ~1;
		uint16_t x_end = x + desc->width;
		uint16_t x_end_aligned = (x_end + 1) & ~1;
		uint8_t x_start_odd = x & 1;

		cmd_data[0] = y;
		cmd_data[1] = y + desc->height - 1;
		ret = ssd1362_write_command(dev, SSD1362_SET_ROW_ADDR, cmd_data, 2);
		if (ret < 0) {
			return ret;
		}

		cmd_data[0] = config->column_offset + (x_aligned >> 1);
		cmd_data[1] = config->column_offset + (x_end_aligned >> 1) - 1;
		ret = ssd1362_write_command(dev, SSD1362_SET_COLUMN_ADDR, cmd_data, 2);
		if (ret < 0) {
			return ret;
		}

		for (uint16_t row = 0; row < desc->height; row++) {
			size_t len = ssd1362_conv_row_rgb565(
				&rgb_buf[row * desc->pitch],
				desc->width,
				config->conversion_buf,
				x_start_odd);

			struct display_buffer_descriptor mipi_desc = {
				.buf_size = len,
				.width = len * 8,
				.height = 1,
				.pitch = len * 8,
			};
			ret = mipi_dbi_write_display(config->mipi_dev, &config->dbi_config,
						     config->conversion_buf, &mipi_desc,
						     PIXEL_FORMAT_MONO01);
			if (ret < 0) {
				return ret;
			}
		}
		return 0;
	}

	if (desc->pitch > desc->width) {
		LOG_ERR("Unsupported mode");
		return -EINVAL;
	}

	cmd_data[0] = y;
	cmd_data[1] = y + desc->height - 1;
	ret = ssd1362_write_command(dev, SSD1362_SET_ROW_ADDR, cmd_data, 2);
	if (ret < 0) {
		return ret;
	}

	cmd_data[0] = config->column_offset + (x >> 1) * config->segments_per_pixel;
	cmd_data[1] = config->column_offset + ((x + desc->width) >> 1) * config->segments_per_pixel - 1;
	ret = ssd1362_write_command(dev, SSD1362_SET_COLUMN_ADDR, cmd_data, 2);
	if (ret < 0) {
		return ret;
	}

	uint32_t pixel_count = desc->width * desc->height;
	return ssd1362_write_pixels(dev, buf, pixel_count, pf);
}

static int ssd1362_set_contrast(const struct device *dev, const uint8_t contrast)
{
	return ssd1362_write_command(dev, SSD1362_SET_CONTRAST, &contrast, 1);
}

static void ssd1362_get_capabilities(const struct device *dev, struct display_capabilities *caps)
{
	const struct ssd1362_config *config = dev->config;
	struct ssd1362_data *data = dev->data;

	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = config->width;
	caps->y_resolution = config->height;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO01 | PIXEL_FORMAT_RGB_565;
	caps->current_pixel_format = (data != NULL) ? data->current_pf : PIXEL_FORMAT_MONO01;
	caps->screen_info = 0;
}

static int ssd1362_set_pixel_format(const struct device *dev, enum display_pixel_format pf)
{
	struct ssd1362_data *data = dev->data;

	if (data == NULL) {
		return -ENODEV;
	}

	if (pf == PIXEL_FORMAT_MONO01 || pf == PIXEL_FORMAT_RGB_565) {
		data->current_pf = pf;
		return 0;
	}

	return -ENOTSUP;
}

static int ssd1362_init_device(const struct device *dev)
{
	int ret;
	uint8_t data[2];
	const struct ssd1362_config *config = dev->config;

	ret = mipi_dbi_reset(config->mipi_dev, 1);
	if (ret < 0) {
		LOG_ERR("Reset failed: %d", ret);
		return ret;
	}
	k_usleep(100);

	data[0] = SSD1362_COMMAND_LOCK_UNLOCK;
	ret = ssd1362_write_command(dev, SSD1362_COMMAND_LOCK, data, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_DISPLAY_OFF, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0x00;
	ret = ssd1362_write_command(dev, SSD1362_SET_FADE_MODE, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = config->mux_ratio - 1;
	ret = ssd1362_write_command(dev, SSD1362_SET_MUX_RATIO, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = config->start_line;
	ret = ssd1362_write_command(dev, SSD1362_SET_START_LINE, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = config->row_offset;
	ret = ssd1362_write_command(dev, SSD1362_SET_DISPLAY_OFFSET, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0xC3;
	ret = ssd1362_write_command(dev, SSD1362_SET_REMAP, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = config->contrast;
	ret = ssd1362_write_command(dev, SSD1362_SET_CONTRAST, data, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_DEFAULT_GREYSCALE, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0x22;
	ret = ssd1362_write_command(dev, SSD1362_SET_PHASE_LENGTH, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0xA0;
	ret = ssd1362_write_command(dev, SSD1362_SET_CLOCK_DIV, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0x04;
	ret = ssd1362_write_command(dev, SSD1362_SET_SECOND_PRECHARGE, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0x01;
	ret = ssd1362_write_command(dev, SSD1362_SET_VDD, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = config->iref_external ? 0x8E : 0x9E;
	ret = ssd1362_write_command(dev, SSD1362_SET_IREF, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0x1F;
	ret = ssd1362_write_command(dev, SSD1362_SET_PRECHARGE_VOLTAGE, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0x01;
	ret = ssd1362_write_command(dev, SSD1362_SET_PRECHARGE_CAP, data, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = 0x07;
	ret = ssd1362_write_command(dev, SSD1362_SET_VCOMH, data, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev,
				    config->inversion_on ? SSD1362_MODE_INVERSE : SSD1362_MODE_NORMAL,
				    NULL, 0);
	if (ret < 0) {
		return ret;
	}

	return ssd1362_write_command(dev, SSD1362_DISPLAY_ON, NULL, 0);
}

static int ssd1362_init(const struct device *dev)
{
	const struct ssd1362_config *config = dev->config;

	if (!device_is_ready(config->mipi_dev)) {
		LOG_ERR("MIPI not ready!");
		return -ENODEV;
	}

	int ret = ssd1362_init_device(dev);

	if (ret < 0) {
		LOG_ERR("Failed to initialize device, err = %d", ret);
		return -EIO;
	}

	return 0;
}

static DEVICE_API(display, ssd1362_driver_api) = {
	.blanking_on = ssd1362_blanking_on,
	.blanking_off = ssd1362_blanking_off,
	.write = ssd1362_write,
	.set_contrast = ssd1362_set_contrast,
	.get_capabilities = ssd1362_get_capabilities,
	.set_pixel_format = ssd1362_set_pixel_format,
};

#define SSD1362_CONV_BUFFER_SIZE(node_id)                                                          \
	DIV_ROUND_UP(DT_PROP(node_id, width) * DT_PROP(node_id, height) *                          \
			     DT_PROP(node_id, segments_per_pixel),                                 \
		     SSD1362_SEGMENTS_PER_BYTE)

#define SSD1362_DEFINE(node_id)                                                                    \
	static uint8_t conversion_buf##node_id[SSD1362_CONV_BUFFER_SIZE(node_id)];                 \
	static struct ssd1362_data data##node_id = {                                               \
		.current_pf = PIXEL_FORMAT_MONO01,                                                 \
	};                                                                                         \
	static const struct ssd1362_config config##node_id = {                                     \
		.height = DT_PROP(node_id, height),                                                \
		.width = DT_PROP(node_id, width),                                                  \
		.column_offset = DT_PROP(node_id, column_offset),                                  \
		.row_offset = DT_PROP(node_id, row_offset),                                        \
		.start_line = DT_PROP(node_id, start_line),                                        \
		.mux_ratio = DT_PROP(node_id, mux_ratio),                                          \
		.remap_row_first = DT_PROP(node_id, remap_row_first),                              \
		.remap_columns = DT_PROP(node_id, remap_columns),                                  \
		.remap_rows = DT_PROP(node_id, remap_rows),                                        \
		.remap_nibble = DT_PROP(node_id, remap_nibble),                                    \
		.remap_com_odd_even_split = DT_PROP(node_id, remap_com_odd_even_split),            \
		.remap_com_dual = DT_PROP(node_id, remap_com_dual),                                \
		.segments_per_pixel = DT_PROP(node_id, segments_per_pixel),                        \
		.contrast = DT_PROP(node_id, contrast),                                            \
		.inversion_on = DT_PROP(node_id, inversion_on),                                    \
		.iref_external = DT_PROP(node_id, iref_external),                                  \
		.mipi_dev = DEVICE_DT_GET(DT_PARENT(node_id)),                                     \
		.dbi_config = {.mode = MIPI_DBI_MODE_SPI_4WIRE,                                    \
			       .config = MIPI_DBI_SPI_CONFIG_DT(                                   \
				       node_id, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0)},         \
		.conversion_buf = conversion_buf##node_id,                                         \
		.conversion_buf_size = sizeof(conversion_buf##node_id),                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_DEFINE(node_id, ssd1362_init, NULL, &data##node_id, &config##node_id,            \
			 POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &ssd1362_driver_api);

DT_FOREACH_STATUS_OKAY(solomon_ssd1362, SSD1362_DEFINE)
