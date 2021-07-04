
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lcd/lcd_lib.h"
#include "esp_log.h"

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=320*2+8
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    //Initialize the LCD
    lcd_init(spi);

    // Draw text on screen 
    drawStringX(spi,"Hello World!",50,20,0);

    // Set foreground to Cyan & background to black 
    setTextColor(ILI9341_CYAN, ILI9341_BLACK); 

    // Test alternate font 
    drawString8_16(spi,"Hello world!",50,50);
    setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    
    // Test larger scales 
    drawStringX(spi,"Hello World!",50,90,1);
    setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    drawStringX(spi,"Hello World!",50,130,2);

}
