# NEO-M8N GNSS driver - builds under ESP-IDF and the Pico SDK from one tree.
#
# ESP-IDF discovers this through EXTRA_COMPONENT_DIRS and sets ESP_PLATFORM, so
# the two build systems are told apart by that variable alone. Only the port
# file differs between them; src/ is shared.

set(NEO_M8N_CORE_SRCS
    src/nmea.c
    src/neo_m8n.c)

if(ESP_PLATFORM)

    idf_component_register(
        SRCS ${NEO_M8N_CORE_SRCS} port/neo_m8n_port_esp32.c
        INCLUDE_DIRS include
        PRIV_INCLUDE_DIRS port
        # driver/uart.h moved into esp_driver_uart in IDF v5.3. PRIV_ because
        # neo_m8n.h exposes no IDF types, so dependents need none of it.
        PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_timer)

    target_compile_definitions(${COMPONENT_LIB} PUBLIC NEO_M8N_TARGET_ESP32=1)

else()

    # Pico SDK. INTERFACE matches how the SDK's own libraries are packaged: the
    # sources compile into the consuming executable and inherit its board flags.
    add_library(neo_m8n INTERFACE)

    set(NEO_M8N_SRCS "")
    foreach(src ${NEO_M8N_CORE_SRCS})
        list(APPEND NEO_M8N_SRCS ${CMAKE_CURRENT_LIST_DIR}/${src})
    endforeach()

    target_sources(neo_m8n INTERFACE
        ${NEO_M8N_SRCS}
        ${CMAKE_CURRENT_LIST_DIR}/port/neo_m8n_port_pico.c)

    target_include_directories(neo_m8n INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/port)

    target_link_libraries(neo_m8n INTERFACE
        pico_stdlib
        hardware_uart
        hardware_gpio
        hardware_irq)

    target_compile_definitions(neo_m8n INTERFACE NEO_M8N_TARGET_PICO=1)

endif()
