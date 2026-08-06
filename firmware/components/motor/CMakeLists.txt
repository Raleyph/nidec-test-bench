idf_component_register(
    SRCS
        "src/hardware_driver.cpp"
        "src/mock_driver.cpp"
        "src/motor_service.cpp"

    INCLUDE_DIRS
        "include"
        
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
)