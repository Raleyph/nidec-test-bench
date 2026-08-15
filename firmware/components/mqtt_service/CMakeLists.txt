idf_component_register(
    SRCS
        "src/service.cpp"

    INCLUDE_DIRS
        "include"

    REQUIRES
        esp_event
        mqtt
        motor
)