cmake_minimum_required(VERSION 3.28)
# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")
set(PICO_SDK_FETCH_FROM_GIT 0)
# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(ai4r-sensors C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

#######################################################################################
####################### RP1 CONFIG ####################################################
#######################################################################################

# Add executable. Default name is the project name, version 0.1
add_executable(ai4r-sensors-rp1 ai4r-sensors-rp1.cpp )

pico_set_program_name(ai4r-sensors-rp1 "ai4r-sensors")
pico_set_program_version(ai4r-sensors-rp1 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ai4r-sensors-rp1 0)
pico_enable_stdio_usb(ai4r-sensors-rp1 1)

# Add the standard library to the build
target_link_libraries(ai4r-sensors-rp1 pico_stdlib)


# Add the standard include files to the build
target_include_directories(ai4r-sensors-rp1 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

add_subdirectory(BNO08x_Pico_Library)
#add_subdirectory(vl53l5cx_library)

# Add any user requested libraries
target_link_libraries(ai4r-sensors-rp1
        #vl53l5cx
        BNO08x_Pico_Library
        #pico_multicore
        hardware_i2c
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        )

pico_add_extra_outputs(ai4r-sensors-rp1)



#######################################################################################
####################### RP2 CONFIG ####################################################
#######################################################################################

# Add executable. Default name is the project name, version 0.1
add_executable(ai4r-sensors-rp2 ai4r-sensors-rp2.cpp )
add_subdirectory(pico_servo)

#pico_generate_pio_header(ai4r-sensors-rp2 ${CMAKE_CURRENT_LIST_DIR}/PwmIn.pio)
pico_generate_pio_header(ai4r-sensors-rp2 ${CMAKE_CURRENT_LIST_DIR}/HighLowDurationReader.pio)
pico_generate_pio_header(ai4r-sensors-rp2 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

pico_set_program_name(ai4r-sensors-rp2 "ai4r-sensors-rp2")
pico_set_program_version(ai4r-sensors-rp2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ai4r-sensors-rp2 0)
pico_enable_stdio_usb(ai4r-sensors-rp2 1)

# Add the standard library to the build
target_link_libraries(ai4r-sensors-rp2 pico_stdlib)


# Add the standard include files to the build
target_include_directories(ai4r-sensors-rp2 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)


# Add any user requested libraries
target_link_libraries(ai4r-sensors-rp2
        pico_servo
        pico_multicore
        hardware_pio
        hardware_pwm
        hardware_i2c
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_irq
        )

pico_add_extra_outputs(ai4r-sensors-rp2)
