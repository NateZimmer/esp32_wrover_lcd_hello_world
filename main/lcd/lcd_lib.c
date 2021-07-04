#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lcd_lib.h"
#include "lcd_fonts.h"

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum {
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;

static uint8_t lineBuf[ILI9341_TFTHEIGHT * 8];

int16_t _width = ILI9341_TFTWIDTH, _height = ILI9341_TFTHEIGHT; // Display w/h as modified by current rotation

static uint8_t rotation;
static uint16_t	textcolor;
static uint16_t	textbgcolor;

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[]={
    /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
    {0x36, {(1<<5)|(1<<6)}, 1},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Porch Setting */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    {0xB7, {0x45}, 1},
    /* VCOM Setting, VCOM=1.175V */
    {0xBB, {0x2B}, 1},
    /* LCM Control, XOR: BGR, MX, MH */
    {0xC0, {0x2C}, 1},
    /* VDV and VRH Command Enable, enable=1 */
    {0xC2, {0x01, 0xff}, 2},
    /* VRH Set, Vap=4.4+... */
    {0xC3, {0x11}, 1},
    /* VDV Set, VDV=0 */
    {0xC4, {0x20}, 1},
    /* Frame Rate Control, 60Hz, inversion=0 */
    {0xC6, {0x0f}, 1},
    /* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
    {0xD0, {0xA4, 0xA1}, 1},
    /* Positive Voltage Gamma Control */
    {0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14},
    /* Negative Voltage Gamma Control */
    {0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14},
    /* Sleep Out */
    {0x11, {0}, 0x80},
    /* Display On */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff}
};

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, {0x00, 0x83, 0X30}, 3},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    {0xE8, {0x85, 0x01, 0x79}, 3},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, {0x20}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    {0xC0, {0x26}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, {0x11}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, {0x35, 0x3E}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, {0xBE}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0x28}, 1},
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, {0x00, 0x1B}, 2},
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    /* Negative gamma correction */
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

uint32_t lcd_get_id(spi_device_handle_t spi)
{
    //get_id cmd
    lcd_cmd(spi, 0x04);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    return *(uint32_t*)t.rx_data;
}

//Initialize the display
void lcd_init(spi_device_handle_t spi)
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //detect LCD type
    uint32_t lcd_id = lcd_get_id(spi);
    int lcd_detected_type = 0;
    int lcd_type;

    printf("LCD ID: %08X\n", lcd_id);
    if ( lcd_id == 0 ) {
        //zero, ili
        lcd_detected_type = LCD_TYPE_ILI;
        printf("ILI9341 detected.\n");
    } else {
        // none-zero, ST
        lcd_detected_type = LCD_TYPE_ST;
        printf("ST7789V detected.\n");
    }

#ifdef CONFIG_LCD_TYPE_AUTO
    lcd_type = lcd_detected_type;
#elif defined( CONFIG_LCD_TYPE_ST7789V )
    printf("kconfig: force CONFIG_LCD_TYPE_ST7789V.\n");
    lcd_type = LCD_TYPE_ST;
#elif defined( CONFIG_LCD_TYPE_ILI9341 )
    printf("kconfig: force CONFIG_LCD_TYPE_ILI9341.\n");
    lcd_type = LCD_TYPE_ILI;
#endif
    if ( lcd_type == LCD_TYPE_ST ) {
        printf("LCD ST7789V initialization.\n");
        lcd_init_cmds = st_init_cmds;
    } else {
        printf("LCD ILI9341 initialization.\n");
        lcd_init_cmds = ili_init_cmds;
    }

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 0);

    setRotation(spi, 1);
    fillScreen(spi, ILI9341_BLACK);
    setTextColor(ILI9341_RED, ILI9341_BLACK);
}

void setAddrWindow(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t buf[4];
    lcd_cmd(spi,ILI9341_CASET);
    buf[0] = x0 >> 8;
    buf[1] = x0 & 0xFF;
    buf[2] = x1 >> 8;
    buf[3] = x1 & 0xFF;
    lcd_data(spi, buf, sizeof(buf));
    
    lcd_cmd(spi,ILI9341_PASET);
    buf[0] = y0 >> 8;
    buf[1] = y0 & 0xFF;
    buf[2] = y1 >> 8;
    buf[3] = y1 & 0xFF;
    lcd_data(spi, buf, sizeof(buf));

    lcd_cmd(spi,ILI9341_RAMWR);
}

void setTextColor(uint16_t c, uint16_t b) {
	textcolor   = c;
	textbgcolor = b;
}

void fillRect(spi_device_handle_t spi, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    uint8_t colorBuf[2];
    if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;
	setAddrWindow(spi, x, y, x+w-1, y+h-1);
    for(int i = 0; i < h*w; i++)
    {
        colorBuf[0] = color >> 8;
        colorBuf[1] = color & 0xFF;
        lcd_data(spi, colorBuf, sizeof(colorBuf));
    }
}

void drawPixel(spi_device_handle_t spi, int16_t x, int16_t y, uint16_t color)
{
    uint8_t colorBuf[2];
	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
	setAddrWindow(spi, x, y, x+1, y+1);
    colorBuf[0] = color >> 8;
    colorBuf[1] = color & 0xFF;
    lcd_data(spi, colorBuf, sizeof(colorBuf));
}

void fillScreen(spi_device_handle_t spi, uint16_t color)
{
    color = (color >> 8) + ((color & 0xFF) << 8);
	setAddrWindow(spi, 0, 0, _width -1, _height -1);
    uint8_t * bufPtr = lineBuf; 
    // Fill line buf
    for(int i = 0; i < _width; i++)
    {
        memcpy(bufPtr, (uint8_t *)(&color), sizeof(color));
        bufPtr +=2;
    }
    for(int i = 0; i < _height; i++)
    {
        lcd_data(spi, lineBuf, _width * 2);
    }

}

void setRotation(spi_device_handle_t spi, uint8_t m)
{
	uint8_t data;
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		data = MADCTL_MX | MADCTL_BGR;
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 1:
		data = MADCTL_MV | MADCTL_BGR;
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	case 2:
		data = MADCTL_MY | MADCTL_BGR;
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 3:
		data = MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR;
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	}
    lcd_cmd(spi,ILI9341_MADCTL);
    lcd_data(spi, &data, sizeof(data));
}

void drawFastHLine(spi_device_handle_t spi, int16_t x, int16_t y, int16_t w, uint16_t color) 
{
    color = (color >> 8) + ((color & 0xFF) << 8);
    if((x >= _width) || (y >= _height)) return;
	if((x+w-1) >= _width)  w = _width-x;
	setAddrWindow(spi, x, y, x+w-1, y);
    uint8_t * bufPtr = lineBuf; 
    // Fill line buf
    for(int i = 0; i < w; i++)
    {
        memcpy(bufPtr, (uint8_t *)(&color), sizeof(color));
        bufPtr +=2;
    }
    for(int i = 0; i < w; i++)
    {
        lcd_data(spi, lineBuf, sizeof(lineBuf));
    }
}

static void drawCharX(spi_device_handle_t spi, uint8_t c , uint16_t x, uint16_t y, uint8_t scale)
{
    c -= 32;
    int z = 0;
    for(int yy = 0; yy < 8; yy++)
    {
        for(int v = 0; v < scale; v++)
        {
            for(int xx = 0; xx < 6; xx++)
            {
                for(int r = 0; r < scale; r++)
                {
                    if(font6x8[c*6 + xx] & (1 << yy ))
                    {
                        lineBuf[z + 0] = textcolor >> 8;
                        lineBuf[z + 1] = textcolor & 0xFF;
                    }
                    else
                    {

                        lineBuf[z + 0] = textbgcolor >> 8;
                        lineBuf[z + 1] = textbgcolor & 0xFF;
                    }
                    z +=2;
                }
            }
        }
    }
    setAddrWindow(spi, x, y, x + 6*scale - 1, y + 8*scale - 1);
    lcd_data(spi, lineBuf, (6*scale)*(8*scale)*2);
}

/**
 * @brief Draws string on display 6x8 * scale font
 * @param spi - handle to SPI driver 
 * @param c - ascii character
 * @param x - initial pixel x position 
 * @param y - initial y position 
 * @param scale - scale parameter, 0 smallest, each number past doubles the size 
 */ 
void drawStringX(spi_device_handle_t spi, char * string, uint16_t x, uint16_t y, uint8_t scale)
{
    uint16_t xPos = x;
    int fontX = 6;
    scale++;
    while(*string)
    {
        drawCharX(spi, *string, xPos, y, scale);
        xPos += fontX * scale + 1;
        string++;
    }
}

static void drawChar8_16(spi_device_handle_t spi, uint8_t c , uint16_t x, uint16_t y)
{
    c -= 32;
    int z = 0;
    int fontX = 8;
    int fontY = 16;
    for(int yy = 0; yy < fontY; yy++)
    {
        for(int xx = 0; xx < fontX; xx++)
        {
            if(font8x16[c*fontY + xx + 8 * (yy / 8)] & (1 << (yy % 8) ))
            {
                lineBuf[z + 0] = textcolor >> 8;
                lineBuf[z + 1] = textcolor & 0xFF;
            }
            else
            {

                lineBuf[z + 0] = textbgcolor >> 8;
                lineBuf[z + 1] = textbgcolor & 0xFF;
            }
            z +=2;
        }
    }
    setAddrWindow(spi, x, y, x + fontX - 1, y + fontY - 1);
    lcd_data(spi, lineBuf, fontX * fontY * 2);
}

/**
 * @brief Draws string on display with 8x16 font 
 * @param spi - handle to SPI driver 
 * @param c - ascii character
 * @param x - initial pixel x position 
 * @param y - initial y position 
 */ 
void drawString8_16(spi_device_handle_t spi, char * string, uint16_t x, uint16_t y)
{
    uint16_t xPos = x;
    int fontX = 8;
    while(*string)
    {
        drawChar8_16(spi, *string, xPos,  y);
        xPos += fontX + 1;
        string++;
    }
}
