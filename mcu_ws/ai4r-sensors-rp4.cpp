#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "PwmIn.pio.h"
//#include "encodertime.pio.h"
#include "ws2812.pio.h"
#include <cstring>
#include <math.h>

// Specify the pins
// > For the wheel speed sensor
const uint SPEED_SENSOR_INPUT_PIN = 26;
// > For the PWM outputs for commanding drive and steer
const uint DRIVE_OUTPUT_PIN = 14;
const uint STEER_OUTPUT_PIN = 15;
// > For the inputs from the RC switch
const uint RC_SWITCH_GOOD_INPUT_PIN = 0;
const uint RC_SWITCH_MODE_INPUT_PIN = 1;

// > For the GPIO input (0/1) for mode indicator
//const uint MODE_PIN = 12;

// > For the on-board RGB LED (WS2812) on RP2040-Zero
const uint WS2812_PIN = 16;


// Choose WS2812 mode: true = smooth rainbow wheel, false = discrete ROYGBIV
static constexpr bool WS2812_RAINBOW_WHEEL = true;
static constexpr uint8_t WS2812_HUE_STEP = 10;   // degrees per step if rainbow
static constexpr uint8_t WS2812_BRIGHTNESS = 8; // 0..255 linear scaling
// LED test mode: when true, the main loop advances the LED color continuously
static constexpr bool WS2812_TEST_MODE = false;
// Temporal dimming (frame-based). Show color for ON frames out of PERIOD frames.
static constexpr uint8_t LED_PERIOD_FRAMES = 12;   // total frames in cycle
static constexpr uint8_t LED_ON_FRAMES = 1;       // frames lit per cycle
// Alive timeout (ms): if no command for this long, stop color cycling
static constexpr uint32_t CMD_ALIVE_TIMEOUT_MS = 1000;
// Color cycle periods (ms) when alive
static constexpr uint32_t WHEEL_STEP_MS = 500;     // rainbow wheel advance period
static constexpr uint32_t LIST_STEP_MS  = 1000;    // ROYGBIV step period
// Telemetry control: set to 1 to enable "RP2 SPD ..." prints, 0 to silence
#define RP2_TELEMETRY 1

// Shared command/state between cores
volatile bool g_test_mode = false;
volatile uint16_t g_cmd_drive_us = 1500;
volatile uint16_t g_cmd_steer_us = 1500;
volatile uint8_t g_color_index = 0;
volatile uint16_t g_hue_deg = 0;
volatile uint64_t g_last_cmd_ms = 0; // updated on each command
volatile uint32_t g_frame_count = 0; // increments in main loop
static uint32_t g_led_current_grb = 0; // current unscaled GRB color
static uint32_t g_led_elapsed_ms = 0;  // elapsed ms since last color step


// ==========================
// WS2812 CODE
// ==========================
// WS2812 helpers (single LED)
static PIO ws_pio = pio0;
static int ws_sm = -1;
static uint ws_offset = 0;

// Some boards wire WS2812 variants with different channel order.
// Try RGB on Waveshare RP2040-Zero (reports suggest GRB on many boards).
enum { WS2812_ORDER_GRB = 0, WS2812_ORDER_RGB = 1, WS2812_ORDER_BRG = 2 };
static constexpr int WS2812_COLOR_ORDER = WS2812_ORDER_RGB;

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    switch (WS2812_COLOR_ORDER) {
    case WS2812_ORDER_RGB:
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
    case WS2812_ORDER_BRG:
        return ((uint32_t)b << 16) | ((uint32_t)r << 8) | (uint32_t)g;
    case WS2812_ORDER_GRB:
    default:
        return ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    }
}

static uint32_t hsv_to_grb(float h_deg, float s, float v) {
    float c = v * s;
    float h = fmodf(h_deg, 360.0f) / 60.0f;
    float x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
    float r1=0, g1=0, b1=0;
    if (0.0f <= h && h < 1.0f) { r1=c; g1=x; b1=0; }
    else if (1.0f <= h && h < 2.0f) { r1=x; g1=c; b1=0; }
    else if (2.0f <= h && h < 3.0f) { r1=0; g1=c; b1=x; }
    else if (3.0f <= h && h < 4.0f) { r1=0; g1=x; b1=c; }
    else if (4.0f <= h && h < 5.0f) { r1=x; g1=0; b1=c; }
    else { r1=c; g1=0; b1=x; }
    float m = v - c;
    uint8_t r = (uint8_t)fminf(255.0f, fmaxf(0.0f, (r1 + m) * 255.0f));
    uint8_t g = (uint8_t)fminf(255.0f, fmaxf(0.0f, (g1 + m) * 255.0f));
    uint8_t b = (uint8_t)fminf(255.0f, fmaxf(0.0f, (b1 + m) * 255.0f));
    return urgb_u32(r, g, b);
}

static void ws2812_init_single(uint pin, bool is_rgbw) {
    (void)is_rgbw;
    ws_offset = pio_add_program(ws_pio, &ws2812_program);
    ws_sm = pio_claim_unused_sm(ws_pio, true);
    pio_sm_config c = ws2812_program_get_default_config(ws_offset);
    // Map the state machine's SET and side-set pins to the given pin
    sm_config_set_sideset_pins(&c, pin);
    pio_gpio_init(ws_pio, pin);
    pio_sm_set_consecutive_pindirs(ws_pio, ws_sm, pin, 1, true);
    // Configure clock divider for ~800kHz with cycles per bit = T1+T2+T3
    const int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
    float div = (float)clock_get_hz(clk_sys) / (800000.0f * (float)cycles_per_bit);
    sm_config_set_clkdiv(&c, div);
    // Match Waveshare example: left shift, autopull 32 bits (RGBW)
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(ws_pio, ws_sm, ws_offset, &c);
    pio_sm_set_enabled(ws_pio, ws_sm, true);
}

static inline void ws2812_put_pixel(uint32_t grb) {
    // Extract GRB components and apply simple linear brightness scaling
    uint8_t g = (uint8_t)((grb >> 16) & 0xFF);
    uint8_t r = (uint8_t)((grb >> 8) & 0xFF);
    uint8_t b = (uint8_t)(grb & 0xFF);
    if (WS2812_BRIGHTNESS < 255) {
        r = (uint8_t)((uint16_t)r * WS2812_BRIGHTNESS / 255u);
        g = (uint8_t)((uint16_t)g * WS2812_BRIGHTNESS / 255u);
        b = (uint8_t)((uint16_t)b * WS2812_BRIGHTNESS / 255u);
    }
    // Temporal dimming gate
    bool show = true;
    if (LED_PERIOD_FRAMES > 0) {
        uint32_t fc = g_frame_count; // snapshot
        show = ((fc % LED_PERIOD_FRAMES) < LED_ON_FRAMES);
    }
    uint32_t scaled = show ? (((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b) : 0u;
    // Left align to 32 bits as per standard examples (send 24 MSBs)
    pio_sm_put_blocking(ws_pio, ws_sm, scaled << 8u);
}

static void ws2812_set_color_index(uint8_t idx) {
    // ROYGBIV sequence
    static const uint32_t colors[] = {
        urgb_u32(255,   0,   0), // Red (sum 255)
        urgb_u32(170,  85,   0), // Orange scaled to sum 255 (ratio ~2:1)
        urgb_u32(128, 127,   0), // Yellow approx equal, sum 255
        urgb_u32(  0, 255,   0), // Green (sum 255)
        urgb_u32(  0,   0, 255), // Blue (sum 255)
        urgb_u32( 93,   0, 162), // Indigo scaled from (75,0,130) to sum 255
        urgb_u32(105,   0, 150)  // Violet scaled from (148,0,211) to sum 255
    };
    // Set current color only; rendering handled in tick
    g_led_current_grb = colors[idx % (sizeof(colors)/sizeof(colors[0]))];
}

static void ws2812_next_color() {
    if (WS2812_RAINBOW_WHEEL) {
        g_hue_deg = (uint16_t)((g_hue_deg + WS2812_HUE_STEP) % 360);
        g_led_current_grb = hsv_to_grb((float)g_hue_deg, 1.0f, 1.0f);
    } else {
        g_color_index = (uint8_t)((g_color_index + 1) % 7);
        ws2812_set_color_index(g_color_index);
    }
}

static inline void ws2812_on_command() {
    g_last_cmd_ms = to_ms_since_boot(get_absolute_time());
}

static void ws2812_tick_1ms() {
    // Temporal dimming frame advance
    g_frame_count++;

    // In test mode, spoof alive; otherwise use last_cmd timestamp
    if (WS2812_TEST_MODE) {
        g_last_cmd_ms = to_ms_since_boot(get_absolute_time());
    }
    uint64_t now_ms = to_ms_since_boot(get_absolute_time());
    bool alive = (now_ms - g_last_cmd_ms) <= CMD_ALIVE_TIMEOUT_MS;
    if (alive) {
        g_led_elapsed_ms++;
        uint32_t step_ms = WS2812_RAINBOW_WHEEL ? WHEEL_STEP_MS : LIST_STEP_MS;
        if (g_led_elapsed_ms >= step_ms) {
            ws2812_next_color();
            g_led_elapsed_ms = 0;
        }
    } else {
        // Safety: neutralize drive and steer when command stream times out
        g_cmd_drive_us = 1500;
        g_cmd_steer_us = 1500;
    }
    // Render current color
    ws2812_put_pixel(g_led_current_grb);
}
// ==========================
// END OF: WS2812 CODE
// ==========================



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
    // Handles the command stream because fgets() blocks until it receives data.
    char read_buf [256];
    // use shared globals for commands
    //memset(&read_buf, '\0', sizeof(read_buf));

    while(fgets(read_buf, sizeof(read_buf), stdin) != NULL) {
        if (read_buf[0] == '\n' || read_buf[0] == '\0') {
            continue;
        }
        // Tokenize
        char *tok = strtok(read_buf, " \r\n");
        if (!tok) continue;
        if (!strcmp(tok, "RP2")) {
            char *sub = strtok(NULL, " \r\n");
            if (!sub) continue;
            if (!strcmp(sub, "CMD")) {
                char *drv = strtok(NULL, " \r\n");
                char *str = strtok(NULL, " \r\n");
                if (drv && str) {
                    uint16_t d = (uint16_t)atoi(drv);
                    uint16_t s = (uint16_t)atoi(str);
                    // Basic sanity clamp 1000..2000us
                    if (d < 1000) d = 1000; if (d > 2000) d = 2000;
                    if (s < 1000) s = 1000; if (s > 2000) s = 2000;
                    g_cmd_drive_us = d;
                    g_cmd_steer_us = s;
                    // Call the function that keeps alive
                    ws2812_on_command();
                }
            } else if (!strcmp(sub, "TEST")) {
                char *onoff = strtok(NULL, " \r\n");
                if (onoff) {
                    g_test_mode = (!strcmp(onoff, "ON") || !strcmp(onoff, "on") || !strcmp(onoff, "1"));
                }
            }
        }
        //memset(&read_buf, '\0', sizeof(read_buf));
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
    // Init WS2812 before launching core1 so both cores can update it
    ws2812_init_single(WS2812_PIN, false);
    // Initial color
    g_hue_deg = 0;
    g_color_index = 0;
    if (WS2812_RAINBOW_WHEEL) {
        g_led_current_grb = hsv_to_grb(0.0f, 1.0f, 1.0f);
    } else {
        ws2812_set_color_index(0); // Use the same list path for consistency
    }
    ws2812_put_pixel(g_led_current_grb);
    multicore_launch_core1(core1_entry);

    // Initialise the pins
    // > For speed sensor channel only (PWM on SPEED_SENSOR_INPUT_PIN)
    PwmIn my_PwmInD(SPEED_SENSOR_INPUT_PIN);

    // > For drive and steer
    servo_enable(DRIVE_OUTPUT_PIN);
    servo_enable(STEER_OUTPUT_PIN);

    // > For the RC switch "good"
    gpio_init(RC_SWITCH_GOOD_INPUT_PIN);
    gpio_set_dir(RC_SWITCH_GOOD_INPUT_PIN, GPIO_IN);
    //gpio_pull_down(RC_SWITCH_GOOD_INPUT_PIN);

    // > For the RC switch "mode"
    gpio_init(RC_SWITCH_MODE_INPUT_PIN);
    gpio_set_dir(RC_SWITCH_MODE_INPUT_PIN, GPIO_IN);
    //gpio_pull_down(RC_SWITCH_MODE_INPUT_PIN);

    //gpio_init(MODE_PIN);
    //gpio_set_dir(MODE_PIN, GPIO_IN);
    //gpio_pull_down(MODE_PIN);

    uint64_t last_pub_ms = 0;
    // use shared globals for commands
    uint32_t fake_period_us = 50000; // 20 Hz initial
    // Counter for polling the RC switch
    uint32_t counter_for_rc_switch = 0;

    while(1) {
        // LED engine 1ms tick
        ws2812_tick_1ms();
        // Direct microsecond control for finer resolution
        // auto us_to_deg = [](uint16_t us)->int { if (us < 1000) us = 1000; if (us > 2000) us = 2000; return (int)((us - 1000) * 180 / 1000); };
        // servo_set_position(DRIVE_OUTPUT_PIN, us_to_deg(g_cmd_drive_us));
        // servo_set_position(STEER_OUTPUT_PIN, us_to_deg(g_cmd_steer_us));
        servo_set_pulse_us(DRIVE_OUTPUT_PIN, g_cmd_drive_us);
        servo_set_pulse_us(STEER_OUTPUT_PIN, g_cmd_steer_us);

        float dcD = my_PwmInD.read_dutycycle();
        float period_s = my_PwmInD.read_period();

        // uint32_t mode_val = gpio_get(MODE_PIN) ? 1u : 0u;
        // // Test mode: synthesize values
        // if (g_test_mode) {
        //     mode_val = (to_ms_since_boot(get_absolute_time()) / 1000) % 2;
        //     fake_period_us = 40000 + ((to_ms_since_boot(get_absolute_time())/100) % 20000); // 40..60 ms
        // }

        // uint32_t period_us = g_test_mode ? fake_period_us : (period_s <= 0 ? 0u : (uint32_t)(period_s * 1e6f));
        // float duty = g_test_mode ? 0.5f : dcD;

        #if RP2_TELEMETRY
        uint64_t now_ms = to_ms_since_boot(get_absolute_time());
        // if (now_ms - last_pub_ms >= 50) { // ~20 Hz telemetry
        //     printf("RP2 SPD %u %.4f %u\n", period_us, duty, mode_val);
        //     last_pub_ms = now_ms;
        // }
        #endif

        // Poll and print the RC Switch Pins at regular intervals
        counter_for_rc_switch++;
        if (counter_for_rc_switch >= 200) {
            // Get the current value of each pin
            bool good_value = gpio_get(RC_SWITCH_GOOD_INPUT_PIN);
            bool mode_value = gpio_get(RC_SWITCH_MODE_INPUT_PIN);
            // Send out the data
            printf("RCS %d %d\n", good_value, mode_value);
            // Reset the counter
            counter_for_rc_switch = 0;
        }

        // 1ms loop for LED timing
        sleep_ms(1);
    }
    
}
