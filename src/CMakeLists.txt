add_executable(${NAME}
               main.c
               pico_init.c
               cmd_monitor.c
               veh_movmnt_fsm.c
               )
       
pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/hcsr04.pio)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	                  pico_stdlib
	                  hardware_pwm
	                  hardware_i2c
	                  hardware_pio
	                  hardware_adc
	                  FreeRTOS-Kernel-Heap4
	                  pico_cyw43_arch_none
	                  pico_btstack_ble
	                  pico_btstack_cyw43
	                  rpp_bot                 # src/rpp_bot subdirectory src code
	                  )
    
target_include_directories(${NAME} PRIVATE
                           ${CMAKE_CURRENT_LIST_DIR} # For btstack config
                           )
     
# GATT compiler
pico_btstack_make_gatt_header(${NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/GATT_GAP/server_gattfile.gatt")
     
target_compile_definitions(${NAME} PRIVATE
                           configNUM_CORES=1
#                           FREE_RTOS_KERNEL_SMP # symmetric multiprocessing SMP
                           ENABLE_BLE           # enable BLE
                           )

#Add source subdirectory
add_subdirectory(rpp_bot)


# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
