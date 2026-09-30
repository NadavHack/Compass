# SX127x LoRa driver - builds under ESP-IDF and the Pico SDK from one tree.
#
# ESP-IDF discovers this through EXTRA_COMPONENT_DIRS and sets ESP_PLATFORM, so
# the two build systems are told apart by that variable alone. Only the port
# file differs between them; src/ is shared.

set(LORA_CORE_SRCS src/lora.c)

if(ESP_PLATFORM)

    idf_component_register(
        SRCS ${LORA_CORE_SRCS} port/lora_port_esp32.c
        INCLUDE_DIRS include
        PRIV_INCLUDE_DIRS port
        PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_timer)

    target_compile_definitions(${COMPONENT_LIB} PUBLIC LORA_TARGET_ESP32=1)

else()

    # Pico SDK. INTERFACE matches how the SDK's own libraries are packaged: the
    # sources compile into the consuming executable and inherit its board flags.
    add_library(lora INTERFACE)

    set(LORA_SRCS "")
    foreach(src ${LORA_CORE_SRCS})
        list(APPEND LORA_SRCS ${CMAKE_CURRENT_LIST_DIR}/${src})
    endforeach()

    target_sources(lora INTERFACE
        ${LORA_SRCS}
        ${CMAKE_CURRENT_LIST_DIR}/port/lora_port_pico.c)

    target_include_directories(lora INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/port)

    target_link_libraries(lora INTERFACE
        pico_stdlib
        hardware_spi
        hardware_gpio)

    target_compile_definitions(lora INTERFACE LORA_TARGET_PICO=1)

endif()
