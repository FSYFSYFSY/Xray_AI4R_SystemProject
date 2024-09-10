#include <gpiod.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    const char * gpio_chip_name = "/dev/gpiochip0";
    struct gpiod_chip *chip;
    // struct gpiod_line *line;
    int val;

    // GPIO line number to monitor
    int line_number = 22;

    // Open GPIO chip
    chip = gpiod_chip_open_by_name(gpio_chip_name);

    // // Open GPIO lines
    // line = gpiod_chip_get_line(chip, line_number);

    // // Open switch line for input
    // gpiod_line_request_input(line, "example1");

    // // Read input gpio status
    // val = gpiod_line_get_value(line);

    val = gpiod_ctxless_get_value(gpio_chip_name, line_number, false, "foobar");
    printf("GPIO line value: %d\n", val);

    // Release lines and chip
    // gpiod_line_release(line);
    // gpiod_chip_close(chip);
    return 0;
}