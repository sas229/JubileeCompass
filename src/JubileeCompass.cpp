#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "TinyGPSPlus.h"
#include <math.h>
#include "sd_card.h"
#include "ff.h"
#include "hw_config.h"
#include "diskio.h"
#include "f_util.h"
#include "WS2812.hpp"
#include "wmm.h"

// UART defines.
#define UART0_TX_PIN    0
#define UART0_RX_PIN    1
#define UART0_BAUD_RATE 9600
#define UART0_DATA_BITS 8
#define UART0_STOP_BITS 1
#define UART0_PARITY    UART_PARITY_NONE

// LED setup.
#define WS2812_PIN 28

// Namespaces.
using namespace std;

TinyGPSPlus gps;

#define d2r (M_PI / 180.0)

void add_spi(spi_t *const spi);
void add_sd_card(sd_card_t *const sd_card);

static spi_t *p_spi;
void spi1_dma_isr() { spi_irq_handler(p_spi); }

void test(sd_card_t *pSD) {
    DIR dir;
    FIL fil;
    FILINFO fno;
    FRESULT fr;
    // const char *const filename = "log.txt";
    // fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    // if (FR_OK != fr && FR_EXIST != fr)
    //     panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    // if (f_printf(&fil, "Hello, world!\n") < 0) {
    //     printf("f_printf failed\n");
    // }
    // fr = f_close(&fil);
    // if (FR_OK != fr) {
    //     printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    // }

    // Print every line in file over serial.
    char filename[] = "log.txt";
    printf("Reading from file '%s':\r\n", filename);
    fr = f_open(&fil, filename, FA_READ);
    if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);    
    char buf[100];
    printf("---\r\n");
    while (f_gets(buf, sizeof(buf), &fil)) {
        printf("%s", buf);
    }
    printf("\r\n---\r\n");
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }

    f_opendir(&dir, "/");
    do {
        fr = f_readdir(&dir, &fno);
        if(fno.fname[0] != 0)
            printf("File found: %s\n", fno.fname); // Print File Name
            // Delete file.
            char folder[] = "/";
            char filepath[50];
            strcpy(filepath, folder);
            strcat(filepath, fno.fname);
            FRESULT f = f_unlink(filepath);
        if (FR_OK != fr) {
            printf("f_readdir error: %s (%d)\n", FRESULT_str(fr), fr);
        }
    } while(fno.fname[0] != 0);

    printf("Now delete all the files.\n");
    f_opendir(&dir, "/");
    do {
        fr = f_readdir(&dir, &fno);
        if(fno.fname[0] != 0)
            printf("File found: %s\n", fno.fname); // Print File Name
        if (FR_OK != fr) {
            printf("f_readdir error: %s (%d)\n", FRESULT_str(fr), fr);
        }
    } while(fno.fname[0] != 0);

    f_unmount(pSD->pcName);
}

// Calculate haversine distance for linear distance.
double haversine_km(double lat1, double long1, double lat2, double long2)
{
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;

    return d;
}

double haversine_miles(double lat1, double long1, double lat2, double long2)
{
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 3956 * c; 

    return d;
}

void initialise_spi_sd_card_interface(){
    // Hardware Configuration of SPI "object".
    cout << "Setting up SPI hardware." << endl;
    p_spi = new spi_t;
    memset(p_spi, 0, sizeof(spi_t));
    if (!p_spi) panic("Out of memory");
    p_spi->hw_inst = spi1;
    p_spi->miso_gpio = 12;
    p_spi->mosi_gpio = 11;
    p_spi->sck_gpio = 10;
    p_spi->baud_rate = 12500 * 1000;
    p_spi->dma_isr = spi1_dma_isr;
    p_spi->initialized = false;
    add_spi(p_spi);
    cout << "SPI interface for SD card inisialised." << endl;
}

void initialise_sd_card() {
    // Hardware Configuration of the SD Card "object".
    cout << "Setting up SD card." << endl;
    sd_card_t *p_sd_card = new sd_card_t;
    if (!p_sd_card) panic("Out of memory");
    memset(p_sd_card, 0, sizeof(sd_card_t));
    p_sd_card->pcName = "0:";  // Name used to mount device.
    p_sd_card->spi = p_spi;    // Pointer to the SPI driving this card.
    p_sd_card->ss_gpio = 15;   // The SPI slave select GPIO for this SD card.
    p_sd_card->card_detect_gpio = 14;  // Card detect.
    p_sd_card->card_detected_true = -1; // Use -1 if there is no card detect.
    // State attributes:
    p_sd_card->m_Status = STA_NOINIT;
    p_sd_card->sectors = 0;
    p_sd_card->card_type = 0;
    add_sd_card(p_sd_card);
    cout << "SD card inisialised." << endl;
}

void mount_sd_card(sd_card_t *pSD) {
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (FR_OK != fr) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    fr = f_chdrive(pSD->pcName);
    if (FR_OK != fr) panic("f_chdrive error: %s (%d)\n", FRESULT_str(fr), fr);
    cout << "SD card mounted." << endl;
}

void unmount_sd_card(sd_card_t *pSD) {
    FRESULT fr = f_unmount(pSD->pcName);
    if (FR_OK != fr) panic("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
    cout << "SD card unmounted." << endl;
}

void write_to_log_file(char* filename, std::string line) {
    FIL fil;
    FRESULT fr;
    // const char *const filename = "log.txt";
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&fil, line.c_str()) < 0) {
        printf("f_printf failed\n");
    }
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}

void read_from_log_file() {
    FIL fil;
    FRESULT fr;
    // Print every line in file over serial.
    char filename[] = "log.txt";
    printf("Reading from file '%s':\r\n", filename);
    fr = f_open(&fil, filename, FA_READ);
    if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);    
    char buf[100];
    printf("---\r\n");
    while (f_gets(buf, sizeof(buf), &fil)) {
        printf("%s", buf);
    }
    printf("\r\n---\r\n");
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}

int main() {    
    stdio_init_all();
    sleep_ms(2000);

    // WiFi setup.
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
        printf("Failed to initialise...\n");
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    // Create RGB LED instance.
    WS2812 led(WS2812_PIN, 1, pio1, 3, WS2812::FORMAT_GRB);
    led.fill(WS2812::RGB(255, 0, 0));
    led.show();
    
    // Initialise UART0.
    uart_init(uart0, UART0_BAUD_RATE);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);

    // Set the data format.
    uart_set_format(uart0, UART0_DATA_BITS, UART0_STOP_BITS, UART0_PARITY);

    // Hardware Configuration of SPI "object"
    initialise_spi_sd_card_interface();

    // Hardware Configuration of the SD Card "object"
    initialise_sd_card();

    // Mount SD card.
    mount_sd_card(sd_get_by_num(0));

    // read_from_log_file();

    char filename[] = "log.txt";
    char ch;
    string sentence;
    double log_lat = 0.0;
    double log_lng = 0.0;
    bool first_fix = true;
    while (true) {
        while (uart_is_readable(uart0)) {
            ch = uart_getc(uart0);
            gps.encode(ch);
            if (ch == 36) {
                // Print captured sentence and then clear.
                char* sentence_char_array = &sentence[0];
                // cout << sentence;
                sentence.clear();
                sentence += ch;
                
            }
            else {
                sentence += ch;
            }
        } 
        // Print parsed GPS location.
        if (gps.location.isUpdated() && gps.time.isUpdated()) {
            if (first_fix) {
                log_lat = gps.location.lat();
                log_lng = gps.location.lng();
                first_fix = false;
            }
            led.fill(WS2812::RGB(0, 255, 0));
            led.show();
            double distance = haversine_km(log_lat, log_lng, gps.location.lat(), gps.location.lng())*1000;
            if (distance >= 1000.0) {
                log_lat = gps.location.lat();
                log_lng = gps.location.lng();
                std::stringstream line;
                line << setfill('0') << setw(2) << +gps.time.hour();
                line << ":" << setfill('0') << setw(2) << +gps.time.minute();
                line << ":" << setfill('0') << setw(2) << +gps.time.second();
                line << ", " << gps.location.lat();
                line << ", " << gps.location.lng() << endl;
                cout << line.str() << endl;
                write_to_log_file(filename, line.str());
                line.clear();
            }
        }
        // } else {
        //     // Lost fix.
        //     led.fill(WS2812::RGB(255, 0, 0));
        //     led.show();
        // }
    }
    // Unmount drive.
    unmount_sd_card(sd_get_by_num(0));

    return 0;
}