target_link_libraries(${CMAKE_PROJECT_NAME} 
#    hardware_uart
    hardware_dma
#    ws2812
#    audio
#    pico_stdlib
#    pico_sync
#    hardware_flash
#    pico_multicore
#    pico_rand
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/pico-dmx-src)

#pico_generate_pio_header(${CMAKE_PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/hw/uart_tx.pio)
#target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/hw)
#target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

