#include "rs485_light_output.h"
#include "esphome/core/log.h"



namespace esphome {
namespace rs485 {
  
  static const char *TAG = "rs485.light";

  void RS485LightOutput::dump_config() {
    ESP_LOGCONFIG(TAG, "RS485 LightOutput(Binary) '%s':", device_name_->c_str());
    dump_rs485_device_config(TAG);
  }

  void RS485LightOutput::publish(const uint8_t *data, const num_t len) {
    ESP_LOGW(TAG, "'%s' State not found: %s", device_name_->c_str(), hexencode(&data[0], len).c_str());
  }

  void RS485LightOutput::publish_state(bool state) {
    if(state == state_) return;
    else state_ = state;
    ESP_LOGD(TAG, "'%s' RS485LightOutput::publish_state(%s)", device_name_->c_str(), state ? "True" : "False");

    if(light_ != nullptr) light_->toggle().perform();
  }



}  // namespace rs485
}  // namespace esphome