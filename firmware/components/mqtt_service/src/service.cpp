// © 2026 Raleyph

#include <charconv>
#include <cstdio>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"

#include "mqtt/service.hpp"
#include "mqtt/config.hpp"

namespace {

constexpr char TAG[] = "mqtt";

}

namespace mqtt {

using namespace config;

//////////////////////////////////////////////////////////////////////////
// Public API

esp_err_t MqttService::initialize() {
    esp_mqtt_client_config_t config = {};

    config.broker.address.uri = BROKER_URI;
    config.credentials.username = USERNAME;
    config.credentials.authentication.password = PASSWORD;

    client_ = esp_mqtt_client_init(&config);

    const esp_err_t err = esp_mqtt_client_register_event(
        client_,
        MQTT_EVENT_ANY,
        MqttService::event_handler,
        this
    );

    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t MqttService::start() {
    if (client_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (started_) {
        return ESP_OK;
    }

    const esp_err_t err = esp_mqtt_client_start(client_);

    if (err != ESP_OK) {
        return err;
    }

    started_ = true;

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////////
// Event Handling

void MqttService::event_handler(
    void *arg,
    esp_event_base_t base,
    std::int32_t event_id,
    void *event_data
) {
    auto* self = static_cast<MqttService*>(arg);
    auto* event = static_cast<esp_mqtt_event_handle_t>(event_data);
    
    self->handle_event(event_id, event);
}

void MqttService::handle_event(std::int32_t event_id, esp_mqtt_event_handle_t event) {
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected");

            esp_mqtt_client_subscribe(
                event->client,
                "ventilation/command/#",
                1
            );

            connected_ = true;

            break;
        
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected");

            connected_ = false;

            break;
        
        case MQTT_EVENT_DATA: {
            ESP_LOGI(
                TAG,
                "topic=%.*s data=%.*s",
                event->topic_len,
                event->topic,
                event->data_len,
                event->data
            );

            const std::string_view topic{
                event->topic,
                static_cast<std::size_t>(event->topic_len)
            };

            const std::string_view payload{
                event->data,
                static_cast<std::size_t>(event->data_len)
            };

            handle_message(topic, payload);

            break;
        }
        
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;
        
        default:
            break;
    }
}

void MqttService::handle_message(std::string_view topic, std::string_view payload) {
    if (topic == "ventilation/command/enable") {
        if (payload == "START") {
            motor_service_.start();
        } else if (payload == "STOP") {
            motor_service_.stop();
        }
    }

    if (topic == "ventilation/command/speed") {
        std::uint16_t rpm{};

        const auto [ptr, ec] = std::from_chars(
            payload.data(),
            payload.data() + payload.size(),
            rpm
        );

        if (ec != std::errc() || ptr != payload.data() + payload.size()) {
            ESP_LOGW(TAG, "Invalid speed payload");
            return;
        }

        motor_service_.set_speed(rpm);
    }
}

//////////////////////////////////////////////////////////////////////////
// Message Publishing

void MqttService::publish(
    const char* topic,
    const char* payload,
    std::uint8_t qos,
    bool retain
) {
    if (!connected_) {
        return;
    }

    esp_mqtt_client_publish(
        client_,
        topic,
        payload,
        0,
        qos,
        retain
    );
}

} // namespace mqtt
