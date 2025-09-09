/*
Example for reading commands from serial on pico.

THIS FILE IS NOT COMPILED IN THIS PROJECT AND IS PURELY FOR REFERENCE.
Can be used in place of ai4r-sensors-rp2.cpp if you want to test it.

Ensure your tty flags are set appropriately on the sender or this will not work!

Helpful Reference:
https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

*/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

const uint LED_PIN = 14;

int main() {
    stdio_init_all();
    //wait for usb to be initalised.
    sleep_ms(3000);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    bool onoff = 0;
    char read_buf [256];
    memset(&read_buf, '\0', sizeof(read_buf));

    while (true) {
        // Note that fgets keeps the newline character!
        while(fgets(read_buf, sizeof(read_buf),stdin)!= NULL) {
            if(read_buf[0] != '\n' && read_buf[1] != '\0'){
                if (!strcmp(read_buf,"on\n")) {
                    // toggle led to visually see it working
                    onoff = !onoff;
                    gpio_put(LED_PIN, onoff);
                }
            }
        }
    }
    return 0;
}
