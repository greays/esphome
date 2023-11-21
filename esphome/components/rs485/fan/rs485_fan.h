#pragma once

#include "esphome/components/rs485/rs485.h"
#include "esphome/components/fan/fan.h"

namespace esphome {
namespace rs485 {

class RS485Fan : public RS485Device, public fan::Fan {
  public:
    RS485Fan() { device_name_ = new std::string(this->name_.c_str()); }
    void setup() override;
    void dump_config() override;
    void set_speed_low(hex_t state, cmd_hex_t command) {
      this->state_speed_low_ = state;
      this->command_speed_low_ = command;
    }
    void set_speed_medium(hex_t state, cmd_hex_t command) {
      this->state_speed_medium_ = state;
      this->command_speed_medium_ = command;
    }
    void set_speed_high(hex_t state, cmd_hex_t command) {
      this->state_speed_high_ = state;
      this->command_speed_high_ = command;
    }
    void publish(const uint8_t *data, const num_t len) override;
    bool publish(bool state) override {
      if(this->state != state) {
        this->state = state;
        this->publish_state();
      }
      return !state;
    }
    fan::FanTraits get_traits() override;

  protected:
    void control(const fan::FanCall &call) override;

    hex_t state_speed_low_{};
    hex_t state_speed_medium_{};
    hex_t state_speed_high_{};
    cmd_hex_t command_speed_low_{};
    cmd_hex_t command_speed_medium_{};
    cmd_hex_t command_speed_high_{};
    int speed_count_{0};

};

}  // namespace rs485
}  // namespace esphome
