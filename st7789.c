#include "st7789.h"

static const uint32_t SPI_Command_Mode = 0;
static const uint32_t SPI_Data_Mode = 1;

void spi_master_init(st7789_conf_t *conf);
esp_err_t spi_master_write_command(st7789_conf_t *conf, uint8_t cmd);
esp_err_t spi_master_write_data_byte(st7789_conf_t *conf, uint8_t data);
esp_err_t spi_master_write_addr(st7789_conf_t *conf, uint16_t addr1, uint16_t addr2);
esp_err_t spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t *data, size_t size);

void st7789_off(st7789_conf_t *conf)
{
	spi_master_write_command(conf, ST7789_DISPOFF);
}

void st7789_on(st7789_conf_t *conf)
{
	spi_master_write_command(conf, ST7789_DISPON);
}

void st7789_init(st7789_conf_t *conf)
{
	/*---------------
	 * SPI init
	 * -------------*/

	spi_master_init(conf);

	/*---------------
	 * Display init
	 * -------------*/

	spi_master_write_command(conf, ST7789_SWRESET); // Software Reset
	vTaskDelay(pdMS_TO_TICKS(150));

	spi_master_write_command(conf, ST7789_SLPOUT); // Sleep Out
	vTaskDelay(pdMS_TO_TICKS(255));

	spi_master_write_command(conf, ST7789_COLMOD); // Interface Pixel Format
	spi_master_write_data_byte(conf, ST7789_COLOR_MODE_16bit);
	vTaskDelay(pdMS_TO_TICKS(10));

	spi_master_write_command(conf, 0xB2); //	Porch control
	{
		uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		st7789_write_data(conf, (uint16_t *)data, sizeof(data));
	}

	spi_master_write_command(conf, ST7789_MADCTL);
	spi_master_write_data_byte(conf, conf->display.rotation);

	spi_master_write_command(conf, ST7789_CASET); // Column Address Set
	spi_master_write_data_byte(conf, 0x00);
	spi_master_write_data_byte(conf, 0x00);
	spi_master_write_data_byte(conf, 0x00);
	spi_master_write_data_byte(conf, 0xF0);

	spi_master_write_command(conf, ST7789_RASET); // Row Address Set
	spi_master_write_data_byte(conf, 0x00);
	spi_master_write_data_byte(conf, 0x00);
	spi_master_write_data_byte(conf, 0x00);
	spi_master_write_data_byte(conf, 0xF0);

	/* Internal LCD Voltage generator settings */
	spi_master_write_command(conf, 0XB7);	//	Gate Control
	spi_master_write_data_byte(conf, 0x35); //	Default value
	spi_master_write_command(conf, 0xBB);	//	VCOM setting
	spi_master_write_data_byte(conf, 0x19); //	0.725v (default 0.75v for 0x20)
	spi_master_write_command(conf, 0xC0);	//	LCMCTRL
	spi_master_write_data_byte(conf, 0x2C); //	Default value
	spi_master_write_command(conf, 0xC2);	//	VDV and VRH command Enable
	spi_master_write_data_byte(conf, 0x01); //	Default value
	spi_master_write_command(conf, 0xC3);	//	VRH set
	spi_master_write_data_byte(conf, 0x12); //	+-4.45v (defalut +-4.1v for 0x0B)
	spi_master_write_command(conf, 0xC4);	//	VDV set
	spi_master_write_data_byte(conf, 0x20); //	Default value
	spi_master_write_command(conf, 0xC6);	//	Frame rate control in normal mode
	spi_master_write_data_byte(conf, 0x0F); //	Default value (60HZ)
	spi_master_write_command(conf, 0xD0);	//	Power control
	spi_master_write_data_byte(conf, 0xA4); //	Default value
	spi_master_write_data_byte(conf, 0xA1); //	Default value
	/**************** Division line ****************/

	spi_master_write_command(conf, 0xE0);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
		st7789_write_data(conf, (uint16_t *)data, sizeof(data));
	}

	spi_master_write_command(conf, 0xE1);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
		st7789_write_data(conf, (uint16_t *)data, sizeof(data));
	}
	spi_master_write_command(conf, ST7789_INVON);  // Display Inversion On
	spi_master_write_command(conf, ST7789_NORON);  // Normal Display Mode On
	spi_master_write_command(conf, ST7789_DISPON); // Display ON

	if (conf->pins.bl >= 0)
	{
		gpio_set_level(conf->pins.bl, 1);
	}
}

void st7789_set_window(st7789_conf_t *conf, int32_t x1, int32_t x2, int32_t y1, int32_t y2)
{
	x1 = x1 + conf->display.offset_x;
	y1 = y1 + conf->display.offset_y;
	x2 = x2 + conf->display.offset_x;
	y2 = y2 + conf->display.offset_y;

	spi_master_write_command(conf, ST7789_CASET);
	spi_master_write_addr(conf, x1, x2);
	spi_master_write_command(conf, ST7789_RASET);
	spi_master_write_addr(conf, y1, y2);
}

void st7789_write_data(st7789_conf_t *conf, uint16_t *data, uint32_t size)
{
	spi_master_write_command(conf, ST7789_RAMWR);
	gpio_set_level(conf->pins.dc, SPI_Data_Mode);
	while (size > 0)
	{
		uint16_t buff_size = (size > 512) ? 512 : size;
		spi_master_write_byte(conf->_SPIHandle, (uint8_t *)data, buff_size * 2);
		size -= buff_size;
		data += buff_size;
	}
}

/*------------------------------------
 * Privat API
 * -----------------------------------*/

void spi_master_init(st7789_conf_t *conf)
{

	if (conf->pins.cs >= 0)
	{
		gpio_reset_pin(conf->pins.cs);
		gpio_set_direction(conf->pins.cs, GPIO_MODE_OUTPUT);
		gpio_set_level(conf->pins.cs, 0);
	}

	gpio_reset_pin(conf->pins.dc);
	gpio_set_direction(conf->pins.dc, GPIO_MODE_OUTPUT);
	gpio_set_level(conf->pins.dc, 0);

	if (conf->pins.reset >= 0)
	{
		gpio_reset_pin(conf->pins.reset);
		gpio_set_direction(conf->pins.reset, GPIO_MODE_OUTPUT);
		gpio_set_level(conf->pins.reset, 1);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(conf->pins.reset, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(conf->pins.reset, 1);
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	if (conf->pins.bl >= 0)
	{
		gpio_reset_pin(conf->pins.bl);
		gpio_set_direction(conf->pins.bl, GPIO_MODE_OUTPUT);
		gpio_set_level(conf->pins.bl, 0);
	}

	spi_bus_config_t buscfg = {
		.mosi_io_num = conf->pins.mosi,
		.miso_io_num = -1,
		.sclk_io_num = conf->pins.sclk,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0,
	};

	ESP_ERROR_CHECK(spi_bus_initialize(conf->spi_host, &buscfg, SPI_DMA_CH_AUTO));

	spi_device_interface_config_t devcfg = {0};

	devcfg.clock_speed_hz = conf->clock_speed_hz;
	devcfg.queue_size = 7;
	devcfg.mode = 3;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

	if (conf->pins.cs >= 0)
	{
		devcfg.spics_io_num = conf->pins.cs;
	}
	else
	{
		devcfg.spics_io_num = -1;
	}

	spi_device_handle_t handle;
	ESP_ERROR_CHECK(spi_bus_add_device(conf->spi_host, &devcfg, &handle));
	conf->_SPIHandle = handle;
}

esp_err_t spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t *data, size_t size)
{
	spi_transaction_t SPITransaction = {0};

	if (size > 0)
	{
		SPITransaction.length = size * 8;
		SPITransaction.tx_buffer = data;
		return spi_device_transmit(SPIHandle, &SPITransaction);
	}

	return ESP_ERR_NOT_FOUND;
}

esp_err_t spi_master_write_command(st7789_conf_t *conf, uint8_t cmd)
{
	static uint8_t Byte = 0;
	Byte = cmd;
	gpio_set_level(conf->pins.dc, SPI_Command_Mode);
	return spi_master_write_byte(conf->_SPIHandle, &Byte, 1);
}

esp_err_t spi_master_write_data_byte(st7789_conf_t *conf, uint8_t data)
{
	static uint8_t Byte = 0;
	Byte = data;
	gpio_set_level(conf->pins.dc, SPI_Data_Mode);
	return spi_master_write_byte(conf->_SPIHandle, &Byte, 1);
}

esp_err_t spi_master_write_addr(st7789_conf_t *conf, uint16_t addr1, uint16_t addr2)
{
	static uint8_t Byte[4];
	Byte[0] = (addr1 >> 8) & 0xFF;
	Byte[1] = addr1 & 0xFF;
	Byte[2] = (addr2 >> 8) & 0xFF;
	Byte[3] = addr2 & 0xFF;
	gpio_set_level(conf->pins.dc, SPI_Data_Mode);
	return spi_master_write_byte(conf->_SPIHandle, Byte, 4);
}
