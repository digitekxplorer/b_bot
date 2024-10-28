# Oct 26, 2024
# src/rpp_bot source code subdirectory

add_library(rpp_bot 
            ssd1306_display.c
            ssd1306_display.h
            ssd1306_font.h
            motor.c
            motor.h
            print_num.c 
            print_num.h
            )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(rpp_bot 
	                  pico_stdlib
	                  hardware_pwm
	                  hardware_i2c
	                  FreeRTOS-Kernel-Heap4
	                  )

target_include_directories(rpp_bot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
