// © 2026 Raleyph

#pragma once

#include "esp_err.h"

namespace wifi {

using ConnectedCallback = void (*)(void*);

esp_err_t initialize(ConnectedCallback on_connected, void* arg);

}
