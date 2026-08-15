// © 2026 Raleyph

#pragma once

#include <cstdint>
#include <string_view>

#include "mqtt_client.h"

#include "motor/service.hpp"

namespace mqtt {
    
class MqttService {
public:
    explicit MqttService(motor::MotorService& motor_service) : motor_service_(motor_service) {}

    esp_err_t initialize();
    esp_err_t start();

private:
    static void event_handler(
        void* arg,
        esp_event_base_t base,
        std::int32_t event_id,
        void* event_data
    );

    void handle_event(std::int32_t event_id, esp_mqtt_event_handle_t event);

    void handle_message(std::string_view topic, std::string_view payload);

    void publish(
        const char* topic,
        const char* payload,
        std::uint8_t qos = 1,
        bool retain = false
    );

private:
    esp_mqtt_client_handle_t client_{nullptr};
    bool connected_{false};
    bool started_{false};

    motor::MotorService& motor_service_;
};

} // namespace mqtt
