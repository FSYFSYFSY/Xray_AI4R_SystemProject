add_library(pico_servo INTERFACE)

target_sources(pico_servo INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/pico_servo.c
)

target_link_libraries(pico_servo
        INTERFACE
        pico_stdlib 
        hardware_irq
        hardware_pwm
        hardware_pll
        hardware_clocks
)

