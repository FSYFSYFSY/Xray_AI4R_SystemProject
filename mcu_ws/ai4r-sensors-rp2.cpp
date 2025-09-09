#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "PwmIn.pio.h"
#include <cstring>
#include <math.h>

const uint LED_PIN = 10;
const uint SPEED_SENSOR_PIN = 26;

extern "C" {
    #include "pico_servo/pico_servo.h"
}

/*
###############################################################################
#                                                                             #
#                            Class modified from:                             #
#   https://github.com/GitJer/Some_RPI-Pico_stuff/blob/main/PwmIn/PwmIn.cpp   #
#      Can read a maximum of 8 PWM signal, if nothing else is using pio.      #
#          If no PWM signal present, duty cycle will read 1.0                 #
#                   period and pulsewidth will read inf (2^32 - 1)            #
#                                                                             #
###############################################################################
*/

class PwmIn
{
public:
    // constructor
    // input = pin that receives the PWM pulses.
    PwmIn(uint input) {
        // can claim a maximum of 4 state machines on pio0 
        // could be extended to use pio1 for 8 state machines
        pio = pio0;
        sm = pio_claim_unused_sm(pio, false);
        if (sm == -1) {
            // no free state machine on pio0, try pio1
            pio = pio1;
            // if no free state machine on pio1, will now panic (hence the true)
            sm = pio_claim_unused_sm(pio, true);
        }
        // configure the used pins
        pio_gpio_init(pio, input);
        // load the pio program into the pio memory
        uint offset = pio_add_program(pio, &PwmIn_program);
        // make a sm config
        pio_sm_config c = PwmIn_program_get_default_config(offset);
        // set the 'jmp' pin
        sm_config_set_jmp_pin(&c, input);
        // set the 'wait' pin (uses 'in' pins)
        sm_config_set_in_pins(&c, input);
        // set shift direction
        sm_config_set_in_shift(&c, false, false, 0);
        // init the pio sm with the config
        pio_sm_init(pio, sm, offset, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
    }

    // read_period (in seconds)
    float read_period(void) {
        read();
        // one clock cycle is 1/125000000 seconds
        return (period * 0.000000008);
    }

    // read_pulsewidth (in seconds)
    float read_pulsewidth(void) {
        read();
        // one clock cycle is 1/125000000 seconds
        return (pulsewidth * 0.000000008);
    }

    // read_dutycycle (between 0 and 1)
    float read_dutycycle(void) {
        read();
        return ((float)pulsewidth / (float)period);
    }

    // read pulsewidth and period for one pulse
    void read_PWM(float *readings) {
        read();
        *(readings + 0) = (float)pulsewidth * 0.000000008;
        *(readings + 1) = (float)period * 0.000000008;
        *(readings + 2) = ((float)pulsewidth / (float)period);
    }

private:
    // read the period and pulsewidth
    void read(void) {
        if (pio_sm_get_rx_fifo_level(pio, sm) >= 2) {
            // read pulse width and period from the FIFO
            pulsewidth = pio_sm_get(pio, sm);
            period = pio_sm_get(pio, sm) + pulsewidth;
            pio_sm_clear_fifos(pio, sm);
            // the measurements are taken with 2 clock cycles per timer tick
            pulsewidth = 2 * pulsewidth;
            // calculate the period in clock cycles:
            period = 2 * period;
        } else if (count > cycles_before_period_is_inf){
            // no pulses measured since the last time we checked period 
            // hence set period and pulse width to infinity
            period = -1;
            pulsewidth = -1;
            count = 0;
        } else {
            count ++;
        }
        
    }
    // the tuning of the cycles_before_period_is_inf variable is dependant on the 
    // frequency the main loop is called, ie the frequency you call the read() method for a given
    // state machine if you count how many u_sec the main loop takes you could automatically calculate 
    // the ideal cycles number for some fixed constant you choose by passing in a 
    // u_sec value into the read function and doing some math
    int cycles_before_period_is_inf = 100;
    PIO pio;
    uint sm;
    int count;
    uint32_t pulsewidth, period;
};


void core1_entry() {
    // Handles the command stream because fgets() blocks until it recieves data.
    char read_buf [256];
    bool onoff = 1;
    memset(&read_buf, '\0', sizeof(read_buf));
    
    // note: fgets keeps the newline character!
    while(fgets(read_buf, sizeof(read_buf),stdin)!= NULL) {
        if(read_buf[0] != '\n' && read_buf[1] != '\0'){
            if (!strcmp(read_buf,"on\n")) {
                onoff = !onoff;
                gpio_put(LED_PIN, onoff);
            }
        }
    }

    // alternate non blocking way (not properly tested)

    // char c;
    // int i = 0;
    // memset(&read_buf, '\0', sizeof(read_buf));
    // while ((c = getchar_timeout_us(0)) != -1) {
    //     if (c == '\n' || i == 255) {
    //         read_buf[i] = '\0';
    //         if (!strcmp(read_buf,"on")) {
    //             onoff = !onoff;
    //             gpio_put(LED_PIN, onoff);
    //         }
    //         break;
    //     }
    //     read_buf[i] = c;
    //     i++;
    // }
}


int main() {
    stdio_init_all();
    sleep_ms(3000);
    multicore_launch_core1(core1_entry);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // RC REMOTE CHANNELS
    PwmIn my_PwmInA(6);
    PwmIn my_PwmInB(7);
    PwmIn my_PwmInC(8);
    PwmIn my_PwmInD(SPEED_SENSOR_PIN);


    int SERVO_PINA = 15;
    int SERVO_PINB = 14;

    servo_enable(SERVO_PINA);
    servo_enable(SERVO_PINB);

    while(1) {    
        // Read command from serial input

        // Read Duty Cycles
        float dcA = my_PwmInA.read_dutycycle();
        float dcB = my_PwmInB.read_dutycycle();
        float dcC = my_PwmInC.read_dutycycle();
        float dcD = my_PwmInD.read_dutycycle();

        
        // check that the angle generated is reasonable and make the servo move.
        if (dcA *1000 < 180){
            servo_set_position(SERVO_PINA, dcA*1000);
        }
        if (dcB *1000 < 180){
            servo_set_position(SERVO_PINB, dcB*1000);
        }
        
        printf("dcA = %.5f\tdcB = %.5f\tdcC = %.5f\tdcD = %.5f\n",dcA,dcB,dcC,dcD);
        sleep_ms(10);
    }
    
}

