#pragma once

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#define ST7789_NOP 0x00		// Нет операции (No Operation)
#define ST7789_SWRESET 0x01 // Программный сброс дисплея (Software Reset)
#define ST7789_RDDID 0x04	// Чтение идентификатора дисплея (Read Display ID)
#define ST7789_RDDST 0x09	// Чтение статуса дисплея (Read Display Status)
#define ST7789_SLPIN 0x10	// Вход в спящий режим (Sleep In)
#define ST7789_SLPOUT 0x11	// Выход из спящего режима (Sleep Out)
#define ST7789_PTLON 0x12	// Частичный режим включен (Partial Mode On)
#define ST7789_NORON 0x13	// Нормальный режим работы дисплея (Normal Display Mode On)
#define ST7789_INVOFF 0x20	// Отключение инверсии цветов (Display Inversion Off)
#define ST7789_INVON 0x21	// Включение инверсии цветов (Display Inversion On)
#define ST7789_DISPOFF 0x28 // Выключение дисплея (Display Off)
#define ST7789_DISPON 0x29	// Включение дисплея (Display On)
#define ST7789_CASET 0x2A	// Установка области столбцов (Column Address Set)
#define ST7789_RASET 0x2B	// Установка области строк (Row Address Set)
#define ST7789_RAMWR 0x2C	// Запись данных в GRAM (Memory Write)
#define ST7789_RAMRD 0x2E	// Чтение данных из GRAM (Memory Read)
#define ST7789_PTLAR 0x30	// Установка области частичного режима (Partial Area)
#define ST7789_COLMOD 0x3A	// Выбор формата пикселей (Pixel Format Set)
#define ST7789_MADCTL 0x36	// Настройка ориентации дисплея (Memory Access Control)

#define ST7789_COLOR_MODE_16bit 0x55 //  RGB565 (16bit)
#define ST7789_COLOR_MODE_18bit 0x66 //  RGB666 (18bit)

typedef enum
{
	ST7789_ROTATION_0 = 0x00,	// 0° (обычная ориентация)
	ST7789_ROTATION_90 = 0x60,	// 90° (по часовой)
	ST7789_ROTATION_180 = 0xC0, // 180° (перевернуто)
	ST7789_ROTATION_270 = 0xA0	// 270° (против часовой)
} Rotation;

typedef struct
{
	struct
	{
		int16_t mosi;  // GPIO номер для сигнала MOSI (Master Out Slave In)
		int16_t sclk;  // GPIO номер для сигнала SCLK (Serial Clock)
		int16_t reset; // GPIO номер для сброса дисплея
		int16_t cs;	   // GPIO номер для сигнала Chip Select (CS)
		int16_t dc;	   // GPIO номер для сигнала Data/Command (DC)
		int16_t bl;	   // GPIO номер для управления подсветкой дисплея (Backlight)
	} pins;
	struct
	{
		int16_t width;	   // Ширина дисплея в пикселях
		int16_t height;	   // Высота дисплея в пикселях
		int16_t offset_x;  // Смещение по оси X относительно начала координат
		int16_t offset_y;  // Смещение по оси Y относительно начала координат
		Rotation rotation; // Ориентация дисплея (см. enum Rotation)
	} display;
	spi_host_device_t spi_host;		// SPI-хост, к которому подключён дисплей
	spi_device_handle_t _SPIHandle; // Дескриптор SPI-устройства, связанный с дисплеем
	int clock_speed_hz;				// Частота тактирования SPI в герцах
} st7789_conf_t;

void st7789_write_data(st7789_conf_t *conf, uint16_t *data, uint32_t size);
void st7789_set_window(st7789_conf_t *conf, int32_t x1, int32_t x2, int32_t y1, int32_t y2);
void st7789_init(st7789_conf_t *conf);
void st7789_off(st7789_conf_t *conf);
void st7789_on(st7789_conf_t *conf);