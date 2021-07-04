  
#ifndef _LCD_LIB_H_
#define _LCD_LIB_H_

#include "esp_err.h"
#include "driver/gpio.h"

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1

#define ILI9341_BLACK   0x0000
#define ILI9341_BLUE    0x001F
#define ILI9341_RED     0xF800
#define ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW  0xFFE0  
#define ILI9341_WHITE   0xFFFF

#define TFT_DC_DATA     GPIO_OUTPUT_SET(21, 1)
#define TFT_DC_COMMAND  GPIO_OUTPUT_SET(21, 0)
#define TFT_DC_INIT     PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO21_U, FUNC_GPIO21_GPIO21); TFT_DC_DATA


#define TFT_BKL_INIT    gpio_set_direction(GPIO_NUM_5,GPIO_MODE_OUTPUT);TFT_BKL_ON 
#define TFT_BKL_ON      gpio_set_level(GPIO_NUM_5,0)     // Important - here was 0,1 swap so your screen is now light 
#define TFT_BKL_OFF     gpio_set_level(GPIO_NUM_5,1)    // Important - here was 0,1 swap so your screen is now light


#define TFT_RST_ACTIVE    GPIO_OUTPUT_SET(18, 0)
#define TFT_RST_DEACTIVE  GPIO_OUTPUT_SET(18, 1)
#define TFT_RST_INIT      PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO18_U, FUNC_GPIO18_GPIO18); TFT_RST_DEACTIVE

#define MAKEWORD(b1, b2, b3, b4) (uint32_t(b1) | ((b2) << 8) | ((b3) << 16) | ((b4) << 24))

#ifdef CONFIG_IDF_TARGET_ESP32
#define LCD_HOST    HSPI_HOST
#define DMA_CHAN    2

#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22

#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define LCD_HOST    SPI2_HOST
#define DMA_CHAN    LCD_HOST

#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK  36
#define PIN_NUM_CS   34

#define PIN_NUM_DC   4
#define PIN_NUM_RST  5
#define PIN_NUM_BCKL 6
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define LCD_HOST    SPI2_HOST
#define DMA_CHAN    LCD_HOST

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   10

#define PIN_NUM_DC   9
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 19
#endif


void lcd_spi_pre_transfer_callback(spi_transaction_t *t);
uint32_t lcd_get_id(spi_device_handle_t spi);
void lcd_init(spi_device_handle_t spi);

void fillScreen(spi_device_handle_t spi, uint16_t color);
void drawPixel(spi_device_handle_t spi, int16_t x, int16_t y, uint16_t color);
void drawFastVLine(spi_device_handle_t spi, int16_t x, int16_t y, int16_t h, uint16_t color);
void drawFastHLine(spi_device_handle_t spi, int16_t x, int16_t y, int16_t w, uint16_t color);
void fillRect(spi_device_handle_t spi, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void setRotation(spi_device_handle_t spi, uint8_t r);
void invertDisplay(bool i);
void setAddrWindow(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void setTextColor(uint16_t c, uint16_t b);
void drawStringX(spi_device_handle_t spi, char * string, uint16_t x, uint16_t y, uint8_t scale);
void drawString8_16(spi_device_handle_t spi, char * string, uint16_t x, uint16_t y);


#endif // end _LCD_LIB_H_

