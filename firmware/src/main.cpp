// © 2026 Raleyph

#include "motor/service.hpp"
#include "network/wifi.hpp"
#include "mqtt/service.hpp"

namespace {

void on_wifi_connected(void* arg) {
    auto* mqtt_service = static_cast<mqtt::MqttService*>(arg);
    mqtt_service->start();
}

}

extern "C" void app_main() {
    static motor::HardwareDriver motor{};
    static motor::MotorService motor_service{motor};
    static mqtt::MqttService mqtt_service{motor_service};

    ESP_ERROR_CHECK(motor_service.initialize());
    ESP_ERROR_CHECK(motor_service.start_task());
    ESP_ERROR_CHECK(mqtt_service.initialize());
    
    ESP_ERROR_CHECK(
        wifi::initialize(
            on_wifi_connected,
            &mqtt_service
        )
    );
}
