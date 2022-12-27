#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <cstring>
#include "pico/stdlib.h"`
#include "TinyGPSPlus.h"

// UART defines.
#define UART0_TX_PIN    0
#define UART0_RX_PIN    1
#define UART0_BAUD_RATE 9600
#define UART0_DATA_BITS 8
#define UART0_STOP_BITS 1
#define UART0_PARITY    UART_PARITY_NONE

// Namespaces.
using namespace std;

TinyGPSPlus gps;

int main()
{
    stdio_init_all();
    
    // Initialise UART0.
    uart_init(uart0, UART0_BAUD_RATE);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);

    // Set the data format.
    uart_set_format(uart0, UART0_DATA_BITS, UART0_STOP_BITS, UART0_PARITY);

    char ch;
    string sentence;
    while (true) {
        while (uart_is_readable(uart0)) {
            ch = uart_getc(uart0);
            gps.encode(ch);
            if (ch == 36) {
                // Print captured sentence and then clear.
                char* sentence_char_array = &sentence[0];
                cout << sentence;
                sentence.clear();
                sentence += ch;
            }
            else {
                sentence += ch;
            }
        } 
        // Print parsed GPS location.
        if (gps.location.isUpdated() && gps.time.isUpdated()) {
            cout << "\nTime: " << setfill('0') << setw(2) << +gps.time.hour();
            cout << ":" << setfill('0') << setw(2) << +gps.time.minute();
            cout << ":" << setfill('0') << setw(2) << +gps.time.second();
            cout << "; Latitude: " << gps.location.lat();
            cout << "; Longitude: " << gps.location.lng() << "\n";
        }
    }

    return 0;
}