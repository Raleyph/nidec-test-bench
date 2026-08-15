idf_component_register(
    SRCS "main.cpp"

    REQUIRES
        wifi
        mqtt_service
        motor
)