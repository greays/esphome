#pragma once

#include "esphome/components/rs485/rs485.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace rs485 {

class RS485Switch : public switch_::Switch, public RS485Device {
  public:
        RS485Switch() { device_name_.emplace(this->name_); }
        // ...
  protected:
        esphome::optional<std::string> device_name_;
};

}  // namespace rs485
}  // namespace esphome
