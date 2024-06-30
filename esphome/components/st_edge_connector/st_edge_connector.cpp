#include "st_edge_connector.h"

#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"
#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_ARDUINO
#include <StreamString.h>
#endif

namespace esphome {
namespace st_edge_connector {

static const char *const TAG = "st_edge_connector";

StEdgeConnector::StEdgeConnector() {
#ifdef USE_ESP32
  to_schedule_lock_ = xSemaphoreCreateMutex();
#endif
}

void StEdgeConnector::setup() {
  ESP_LOGCONFIG(TAG, "Setting up web server...");
  this->setup_controller(this->include_internal_);

#ifdef USE_BINARY_SENSOR
  for (auto *obj : App.get_binary_sensors()) {
    if (this->include_internal_ || !obj->is_internal())
      this->st_address_list_.push_back({obj->get_object_id(), obj->get_object_id_hash(), {0x00}});
  }
#endif

#ifdef USE_SENSOR
  for (auto *obj : App.get_sensors()) {
    if (this->include_internal_ || !obj->is_internal())
      this->st_address_list_.push_back({obj->get_object_id(), obj->get_object_id_hash(), {0x00}});
  }
#endif

#ifdef USE_SWITCH
  for (auto *obj : App.get_switches()) {
    if (this->include_internal_ || !obj->is_internal())
      this->st_address_list_.push_back({obj->get_object_id(), obj->get_object_id_hash(), {0x00}});
  }
#endif

#ifdef USE_LIGHT
  for (auto *obj : App.get_lights()) {
    if (this->include_internal_ || !obj->is_internal())
      this->st_address_list_.push_back({obj->get_object_id(), obj->get_object_id_hash(), {0x00}});
  }
#endif

  this->server_ = std::make_shared<AsyncWebServer>(this->port_);
  this->web_handler();
  this->server_->begin();
}

void StEdgeConnector::loop() {
  // Join Multicast
  if(this->pairing_mode_ && !this->udp_init_ && !this->pairing_all_device_ && network::is_connected()) {
    this->join_multicast_group();
  }
  // Stop Multicast
  else if(this->pairing_mode_ && this->udp_init_ && this->pairing_all_device_) {
    ESP_LOGI(TAG, "Stop Multicast...");
    this->udp_init_ = false;
    this->udp_client_.stop();
  }

  // Listen Packet
  if(this->udp_init_) {
    int packet_size = this->udp_client_.parsePacket();
    if(packet_size) {
      if(packet_size >= sizeof(this->buffer_))
        packet_size = sizeof(this->buffer_) - 1;
      this->buffer_[packet_size] = 0x00;
      this->udp_client_.read(this->buffer_, packet_size);
      ESP_LOGD(TAG, "Recv Packet: %s", this->buffer_);
      json::parse_json(this->buffer_, [this] (JsonObject root) {
        const char* cmd = root["cmd"];
        const char* data = root["data"];
        ESP_LOGD(TAG, "  cmd: %s", cmd);
        ESP_LOGD(TAG, "  data: %s", data);
        if(strcmp(cmd,"search") == 0)
          this->process_pairing(data);
      });
    }
  }

#ifdef USE_ESP32
  if (xSemaphoreTake(this->to_schedule_lock_, 0L)) {
    std::function<void()> fn;
    if (!to_schedule_.empty()) {
      // scheduler execute things out of order which may lead to incorrect state
      // this->defer(std::move(to_schedule_.front()));
      // let's execute it directly from the loop
      fn = std::move(to_schedule_.front());
      to_schedule_.pop_front();
    }
    xSemaphoreGive(this->to_schedule_lock_);
    if (fn) {
      fn();
    }
  }
#endif
}

void StEdgeConnector::dump_config() {
  ESP_LOGCONFIG(TAG, "StEdgeConnector for esphome:");
  ESP_LOGCONFIG(TAG, "  Pairing Mode: %s", YESNO(this->pairing_mode_));
  if(this->pairing_mode_) {
    ESP_LOGCONFIG(TAG, "    UDP Multicast Group: %s:%u", this->udp_mcast_group_.toString().c_str(), this->udp_mcast_port_);
    ESP_LOGCONFIG(TAG, "    UDP Multicast Join: %s", YESNO(this->udp_init_));
    ESP_LOGCONFIG(TAG, "    Include Internal Device: %s", YESNO(this->include_internal_));
    ESP_LOGCONFIG(TAG, "    Pairing All Device: %s (%d/%d)", YESNO(this->pairing_all_device_), this->pairing_device_cnt_, this->st_address_list_.size());
  }
  ESP_LOGCONFIG(TAG, "  SmartThings Hub: %s", this->hub_ip_.toString().c_str());
  ESP_LOGCONFIG(TAG, "  Control Server: %s:%u", network::get_use_address().c_str(), this->port_);
  for(auto obj : st_address_list_) {
    if(obj.callback_url[0] != 0x00)
      ESP_LOGCONFIG(TAG, "    - callback url: %s (id: %s)", obj.callback_url, obj.id.c_str());
  }
}

float StEdgeConnector::get_setup_priority() const {
  // After setup all devices
  return setup_priority::LATE - 100.0f;
}

// State push(esp to st) & refresh(st to esp)
#define set_json_state(root, entity_id, value, type)\
  auto state = (type) == PUSH ? (root) : (root).createNestedObject("state");\
  if((type) == PUSH) {\
    state["uuid"] = ("http://" + network::get_use_address() + ":" + std::to_string(this->port_) + "/" + entity_id);\
    state["data"] = (value);\
  }\
  else if(type == REFRESH) {\
    state["entity_id"] = (entity_id);\
    state["state"] = (value);\
  }\
  auto attr = state.createNestedObject("attributes");\
  attr["friendly_name"] = obj->get_name().str();

#ifdef USE_BINARY_SENSOR
void StEdgeConnector::on_binary_sensor_update(binary_sensor::BinarySensor *obj, bool state) {
  auto addr = get_st_address_(obj->get_object_id().c_str());
  if(addr == nullptr)
    return;
  this->process_callback(addr, this->binary_sensor_json(obj, state, PUSH).c_str());
}
void StEdgeConnector::handle_binary_sensor_request(AsyncWebServerRequest *request, const cmd_url_t &url) {
  auto addr = get_st_address_(url.id);
  auto obj = App.get_binary_sensor_by_key(addr->key);
  if(addr == nullptr)
    request->send(404);
  else if(strcmp(url.cmd, "refresh") == 0 && request->method() == HTTP_GET) {
    auto json = this->binary_sensor_json(obj, obj->state, REFRESH);
    ESP_LOGD(TAG, "  response: %s", json.c_str());
    request->send(200, "application/json; charset=utf-8", json.c_str());
  }
  else
    request->send(404);
}
std::string StEdgeConnector::binary_sensor_json(binary_sensor::BinarySensor *obj, bool value, JsonType type) {
  return json::build_json([this, obj, value, type](JsonObject root) {
    set_json_state(root, "binary_sensor." + obj->get_object_id(), value ? "on" : "off", type);
    attr["device_class"] = obj->get_device_class();
  });
}
#endif

#ifdef USE_SENSOR
void StEdgeConnector::on_sensor_update(sensor::Sensor *obj, float state) {
  auto addr = get_st_address_(obj->get_object_id().c_str());
  if(addr == nullptr)
    return;
  this->process_callback(addr, this->sensor_json(obj, state, PUSH).c_str());
}
void StEdgeConnector::handle_sensor_request(AsyncWebServerRequest *request, const cmd_url_t &url) {
  auto addr = get_st_address_(url.id);
  auto obj = App.get_sensor_by_key(addr->key);
  if(addr == nullptr)
    request->send(404);
  else if(strcmp(url.cmd, "refresh") == 0 && request->method() == HTTP_GET) {
    auto json = this->sensor_json(obj, obj->get_state(), REFRESH);
    ESP_LOGD(TAG, "  response: %s", json.c_str());
    request->send(200, "application/json; charset=utf-8", json.c_str());
  }
  else
    request->send(404);
}
std::string StEdgeConnector::sensor_json(sensor::Sensor *obj, float value, JsonType type) {
  return json::build_json([this, obj, value, type](JsonObject root) {
    set_json_state(root, "sensor." + obj->get_object_id(), value, type);
    attr["device_class"] = obj->get_device_class();
  });
}
#endif

#ifdef USE_SWITCH
void StEdgeConnector::on_switch_update(switch_::Switch *obj, bool state) {
  st_address_t *addr = get_st_address_(obj->get_object_id().c_str());
  if(addr == nullptr)
    return;
  this->process_callback(addr, this->switch_json(obj, state, PUSH).c_str());
}
void StEdgeConnector::handle_switch_request(AsyncWebServerRequest *request, const cmd_url_t &url) {
  auto addr = get_st_address_(url.id);
  auto obj = App.get_switch_by_key(addr->key);
  if(addr == nullptr)
    request->send(404);
  else if(strcmp(url.cmd, "refresh") == 0 && request->method() == HTTP_GET) {
    auto json = this->switch_json(obj, obj->state, REFRESH);
    ESP_LOGD(TAG, "  response: %s", json.c_str());
    request->send(200, "application/json; charset=utf-8", json.c_str());
  }
  else
    request->send(404);
}
std::string StEdgeConnector::switch_json(switch_::Switch *obj, bool value, JsonType type) {
  return json::build_json([this, obj, value, type](JsonObject root) {
    set_json_state(root, "switch." + obj->get_object_id(), value ? "on" : "off", type);
    attr["device_class"] = obj->get_device_class();
  });
}
#endif

#ifdef USE_LIGHT
void StEdgeConnector::on_light_update(light::LightState *obj) {
  st_address_t *addr = get_st_address_(obj->get_object_id().c_str());
  if(addr == nullptr)
    return;
  auto data = this->light_json(obj, PUSH);
  this->schedule_([this, addr, data]() { this->process_callback(addr, data.c_str()); });
}
void StEdgeConnector::handle_light_request(AsyncWebServerRequest *request, const cmd_url_t &url) {
  auto addr = get_st_address_(url.id);
  auto obj = App.get_light_by_key(addr->key);
  if(addr == nullptr)
    request->send(404);
  else if(strcmp(url.cmd, "refresh") == 0 && request->method() == HTTP_GET) {
    auto json = this->light_json(obj, REFRESH);
    ESP_LOGD(TAG, "  response: %s", json.c_str());
    request->send(200, "application/json; charset=utf-8", json.c_str());
  }
  else if(strcmp(url.cmd, "control") == 0 && request->method() == HTTP_POST) {
    if(request->getParam("command")->value() == "toggle") {
      this->schedule_([obj]() { obj->toggle().perform(); });
    }
    else if(request->getParam("command")->value() == "turn_on") {
      auto call = obj->turn_on();
      if (request->hasParam("s_command")) {
        auto s_command = request->getParam("s_command")->value();
        auto s_value = request->getParam("s_value")->value();

        if (s_command == "brightness_pct") {
          auto brightness = parse_number<float>(s_value.c_str());
          if (brightness.has_value()) {
            call.set_brightness(*brightness / 100.0f);
          }
        }
        else if (s_command == "brightness") {
          auto brightness = parse_number<float>(s_value.c_str());
          if (brightness.has_value()) {
            call.set_brightness(*brightness / 255.0f);
          }
        }
        else if (s_command == "r") {
          auto r = parse_number<float>(s_value.c_str());
          if (r.has_value()) {
            call.set_red(*r / 255.0f);
          }
        }
        else if (s_command == "g") {
          auto g = parse_number<float>(s_value.c_str());
          if (g.has_value()) {
            call.set_green(*g / 255.0f);
          }
        }
        else if (s_command == "b") {
          auto b = parse_number<float>(s_value.c_str());
          if (b.has_value()) {
            call.set_blue(*b / 255.0f);
          }
        }
        else if (s_command == "white_value") {
          auto white_value = parse_number<float>(s_value.c_str());
          if (white_value.has_value()) {
            call.set_white(*white_value / 255.0f);
          }
        }
        else if (s_command == "color_temp") {
          auto color_temp = parse_number<float>(s_value.c_str());
          if (color_temp.has_value()) {
            call.set_color_temperature(*color_temp);
          }
        }
        else if (s_command == "flash") {
          auto flash = parse_number<uint32_t>(s_value.c_str());
          if (flash.has_value()) {
            call.set_flash_length(*flash * 1000);
          }
        }
        else if (s_command == "transition") {
          auto transition = parse_number<uint32_t>(s_value.c_str());
          if (transition.has_value()) {
            call.set_transition_length(*transition * 1000);
          }
        }
        else if (s_command == "effect") {
          const char *effect = s_value.c_str();
          call.set_effect(effect);
        }
      }
      this->schedule_([call]() mutable { call.perform(); });
    }
    else if(request->getParam("command")->value() == "turn_off") {
      this->schedule_([obj]() { obj->turn_off().perform(); });
    }
    request->send(200, "application/json; charset=utf-8", "{\"result\": true}");
  }
  else
    request->send(404);
}
std::string StEdgeConnector::light_json(light::LightState *obj, JsonType type) {
  return json::build_json([this, obj, type](JsonObject root) {
    set_json_state(root, "light." + obj->get_object_id(), obj->remote_values.get_state() ? "on" : "off", type);
  });
}
#endif

void StEdgeConnector::join_multicast_group() {
  delay(100);
  ESP_LOGI(TAG, "Join Multicast Group...");
  bool begin_status = false;

#ifdef USE_ESP8266
  begin_status = this->udp_client_.beginMulticast(IPAddress((ip_addr_t) network::get_ip_address()), this->udp_mcast_group_, this->udp_mcast_port_);
#endif
#ifdef USE_ESP32
  begin_status = this->udp_client_.beginMulticast(this->udp_mcast_group_, this->udp_mcast_port_);
#endif

  if (begin_status) {
    this->udp_init_ = true;
  }
  else {
    ESP_LOGE(TAG, "Join Multicast Group Failed!");
  }
}

/**
 * Process Pairing (UDP)
 * [Receive Packet]
 *   - {"cmd":"search", "data":"light"}
 *   - {"cmd":"search", "data":"switch"}
 *   - {"cmd":"search", "data":"device_tracker"}
 *   - {"cmd":"search", "data":"binary_sensor"}
 *   - {"cmd":"search", "data":"cover"}
 *
 * [Reply Pcket]
 *   - {"port": 20000, "data": [{"id": "light.livingroom1", "attributes": {"supported_color_modes": ["onoff"], "friendly_name": "안방1", "supported_features": 8}}, {"id": "light.food_table", "attributes": {"min_color_temp_kelvin": 3003, "max_color_temp_kelvin": 5714, "min_mireds": 175, "max_mireds": 333, "supported_color_modes": ["color_temp"], "entity_id": ["light.food_table1", "light.food_table2"], "icon": "mdi:lightbulb-group", "friendly_name": "식탁", "supported_features": 0}}]}
 *   - {"port": 20000, "data": [{"id": "switch.bathroom2_fan", "attributes": {"friendly_name": "안방욕실 환풍기"}}]}
 */
void StEdgeConnector::process_pairing(const char *data) {
  if(this->pairing_all_device_) {
    ESP_LOGI(TAG, "Pairing Skip!!! All devices are registered...");
    ESP_LOGI(TAG, "  Type: %s", data);
    ESP_LOGI(TAG, "  Remote: %s:%u", this->udp_client_.remoteIP().toString().c_str(), this->udp_client_.remotePort());
    return;
  }
  ESP_LOGI(TAG, "Pairing!!!");
  ESP_LOGI(TAG, "  Type: %s", data);
  ESP_LOGI(TAG, "  Remote: %s:%u", this->udp_client_.remoteIP().toString().c_str(), this->udp_client_.remotePort());
  bool begin_status = false;
  bool end_status = false;
#ifdef USE_ESP8266
  begin_status = this->udp_client_.beginPacket(this->udp_client_.remoteIP(), this->udp_client_.remotePort());
#endif
#ifdef USE_ESP32
  begin_status = this->udp_client_.beginPacket(this->udp_client_.remoteIP(), this->udp_client_.remotePort());
#endif
  if(!begin_status)
    ESP_LOGE(TAG, "Begin Packet Failed!");

  // Build Reply Packet
  std::string replyPacket = json::build_json([this, data] (JsonObject root) {
    root["port"] = this->port_;
    auto list = root.createNestedArray("data");

#ifdef USE_BINARY_SENSOR
  if(strcmp(data, "binary_sensor") == 0)
    for (auto *obj : App.get_binary_sensors()) {
      if ((this->include_internal_ || !obj->is_internal()) && get_st_address_(obj->get_object_id().c_str())->callback_url[0] == 0x00) {
        auto device = list.createNestedObject();
        std::string id = "binary_sensor.";
        device["id"] = id + obj->get_object_id();
        auto attr = device.createNestedObject("attributes");
        attr["friendly_name"] = obj->get_name().str();
        attr["device_class"] = obj->get_device_class();
      }
    }
#endif
#ifdef USE_SENSOR
  if(strcmp(data, "sensor") == 0)
    for (auto *obj : App.get_sensors()) {
      if ((this->include_internal_ || !obj->is_internal()) && get_st_address_(obj->get_object_id().c_str())->callback_url[0] == 0x00) {
        auto device = list.createNestedObject();
        std::string id = "sensor.";
        device["id"] = id + obj->get_object_id();
        auto attr = device.createNestedObject("attributes");
        attr["friendly_name"] = obj->get_name().str();
        attr["device_class"] = obj->get_device_class();
        
        attr["unit_of_measurement"] = obj->get_unit_of_measurement();
        attr["accuracy_decimals"] = obj->get_accuracy_decimals();
        attr["state_class"] = sensor::state_class_to_string(obj->get_state_class());
      }
    }
#endif
#ifdef USE_SWITCH
  if(strcmp(data, "switch") == 0)
    for (auto *obj : App.get_switches()) {
      if ((this->include_internal_ || !obj->is_internal()) && get_st_address_(obj->get_object_id().c_str())->callback_url[0] == 0x00) {
        auto device = list.createNestedObject();
        std::string id = "switch.";
        device["id"] = id + obj->get_object_id();
        auto attr = device.createNestedObject("attributes");
        attr["friendly_name"] = obj->get_name().str();
        attr["device_class"] = obj->get_device_class();
      }
    }
#endif
#ifdef USE_LIGHT
  if(strcmp(data, "light") == 0)
    for (auto *obj : App.get_lights()) {
      if ((this->include_internal_ || !obj->is_internal()) && get_st_address_(obj->get_object_id().c_str())->callback_url[0] == 0x00) {
        auto device = list.createNestedObject();
        device["id"] = "light." + obj->get_object_id();
        auto attr = device.createNestedObject("attributes");
        attr["friendly_name"] = obj->get_name().str();

        auto traits = obj->get_traits();
        auto color_modes = attr.createNestedArray("supported_color_modes");
        if (traits.supports_color_mode(light::ColorMode::ON_OFF))
          color_modes.add("onoff");
        if (traits.supports_color_mode(light::ColorMode::BRIGHTNESS))
          color_modes.add("brightness");
        if (traits.supports_color_mode(light::ColorMode::WHITE))
          color_modes.add("white");
        if (traits.supports_color_mode(light::ColorMode::COLOR_TEMPERATURE) ||
            traits.supports_color_mode(light::ColorMode::COLD_WARM_WHITE))
          color_modes.add("color_temp");
        if (traits.supports_color_mode(light::ColorMode::RGB))
          color_modes.add("rgb");
        if (traits.supports_color_mode(light::ColorMode::RGB_WHITE) ||
            // HA doesn't support RGBCT, and there's no CWWW->CT emulation in ESPHome yet, so ignore CT control for now
            traits.supports_color_mode(light::ColorMode::RGB_COLOR_TEMPERATURE))
          color_modes.add("rgbw");
        if (traits.supports_color_mode(light::ColorMode::RGB_COLD_WARM_WHITE))
          color_modes.add("rgbww");
        
        if (traits.supports_color_capability(light::ColorCapability::BRIGHTNESS))
          attr["brightness"] = true;

        if (obj->supports_effects()) {
          attr["effect"] = true;
          JsonArray effect_list = attr.createNestedArray("effect_list");
          for (auto *effect : obj->get_effects())
            effect_list.add(effect->get_name());
          effect_list.add("None");
        }
      }
    }
#endif

  });
  ESP_LOGI(TAG, "Reply Packet:\n%s", replyPacket.c_str());
  this->udp_client_.write((const uint8_t *) replyPacket.c_str(), replyPacket.size());

  end_status = this->udp_client_.endPacket();
  if (!end_status) {
    ESP_LOGE(TAG, "Sending Packet Failed!");
  }
}

/**
 * Handle Ping Request (HTTP Response)
 * - Reqeust: /light.kitchen2/ping?address=192.168.7.7&port=46621
 *   (IP address and port are used for status updates)
 * - Response: {"result": true}
 */
void StEdgeConnector::handle_ping_request(AsyncWebServerRequest *request, const cmd_url_t &url) {
  IPAddress hub_ip{};
  if(hub_ip.fromString(request->arg("address"))) {
    auto obj = get_st_address_(url.id);
    if(obj == nullptr) {
      ESP_LOGW(TAG, "[ping] Not Found Device: %s.%s", url.type, url.id);
      request->send(200, "application/json; charset=utf-8", "{\"result\": false}");
      return;
    }
    else if(obj->callback_url[0] == 0x00) {
      this->pairing_device_cnt_++;
      sprintf(obj->callback_url, "http://%s:%s/push-state", request->arg("address").c_str(), request->arg("port").c_str());
      ESP_LOGD(TAG, "[ping] Add: [id=%s][callback_url=%s] (%d/%d)", obj->id.c_str(), obj->callback_url, this->pairing_device_cnt_, this->st_address_list_.size());
      
      if(!this->pairing_all_device_ && this->pairing_device_cnt_ >= this->st_address_list_.size()) {
        this->pairing_all_device_ = true;
        ESP_LOGI(TAG, "All devices are registered. (%d/%d)", this->pairing_device_cnt_, this->st_address_list_.size());
      }
    }
    else {
      sprintf(obj->callback_url, "http://%s:%s/push-state", request->arg("address").c_str(), request->arg("port").c_str());
      ESP_LOGD(TAG, "[ping] Update: [id=%s][callback_url=%s]", obj->id.c_str(), obj->callback_url);
    }
    request->send(200, "application/json; charset=utf-8", "{\"result\": true}");
  } else {
    ESP_LOGW(TAG, "[ping] Invalid IP address: %s", request->arg("address").c_str());
    request->send(200, "application/json; charset=utf-8", "{\"result\": false}");
  }
}

void StEdgeConnector::web_handler() {
  this->server_->onNotFound([this] (AsyncWebServerRequest *request) {
    ESP_LOGD(TAG, "ST Edge Server Handler (%s) %s", (request->method() == HTTP_GET)?"GET":"POST", request->url().c_str());
    ESP_LOGD(TAG, "  Remote IP: %s", request->client()->remoteIP().toString().c_str());
    ESP_LOGV(TAG, "  Header: %d", request->headers());
    for (uint8_t i=0; i<request->headers(); i++){
      ESP_LOGV(TAG, "    - %s: %s", request->headerName(i).c_str(), request->header(i).c_str());
    }
    ESP_LOGD(TAG, "  Arguments: %d", request->args());
    for (uint8_t i=0; i<request->args(); i++){
      ESP_LOGD(TAG, "    - %s: %s", request->argName(i).c_str(), request->arg(i).c_str());
    }

    // Auth Remote IP
    if(this->hub_ip_.isSet() && this->hub_ip_ != request->client()->remoteIP()) {
      ESP_LOGW(TAG, "Does not match configured SmartThings Hub IP(%s)! (Remote IP: %s)", this->hub_ip_.toString().c_str(), request->client()->remoteIP().toString().c_str());
      request->send(404);
      return;
    }

    // Parse URL
    int idxOf1 = request->url().indexOf('.');
    int idxOf2 = request->url().indexOf('/', idxOf1 + 1);
    if(idxOf1 == -1 || idxOf2 == -1) {
      request->send(404);
      return;
    }
    
    char url[request->url().length()+1];
    memset(url, 0x00, sizeof(url));
    strcpy(url, request->url().c_str());
    url[idxOf1] = 0x00;
    url[idxOf2] = 0x00;

    char *type = &url[1];
    char *entity_id = &url[idxOf1+1];
    char *cmd = &url[idxOf2+1];
    ESP_LOGD(TAG, "  type: %s, entity_id: %s, cmd: %s", type, entity_id, cmd);
    
    // Process command 
    if(strcmp(cmd, "ping") == 0 && request->method() == HTTP_POST)
      this->handle_ping_request(request, {type, entity_id, cmd});
#ifdef USE_BINARY_SENSOR
    else if(strcmp(type, "binary_sensor") == 0)
      this->handle_binary_sensor_request(request, {type, entity_id, cmd});
#endif
#ifdef USE_SENSOR
    else if(strcmp(type, "sensor") == 0)
      this->handle_sensor_request(request, {type, entity_id, cmd});
#endif
#ifdef USE_SWITCH
    else if(strcmp(type, "switch") == 0)
      this->handle_switch_request(request, {type, entity_id, cmd});
#endif
#ifdef USE_LIGHT
    else if(strcmp(type, "light") == 0)
      this->handle_light_request(request, {type, entity_id, cmd});
#endif
    else
      request->send(404);
  });
}

void StEdgeConnector::process_callback(st_address_t *addr, const char *json) {
  char *url = addr->callback_url;
  ESP_LOGD(TAG, "Callback URL: %s", url);
  ESP_LOGD(TAG, "        Data: %s", json);

  if (!network::is_connected()) {
    this->http_client_.end();
    ESP_LOGW(TAG, "HTTP Request failed; Not connected to network");
    return;
  }
  bool begin_status = false;
#if defined(USE_ESP32)
  begin_status = this->http_client_.begin(url);
#elif defined(USE_ESP8266)
  if (this->wifi_client_ == nullptr) {
    this->wifi_client_ = std::make_shared<WiFiClient>();
  }
  begin_status = this->http_client_.begin(*this->wifi_client_, url);
#endif

  if (!begin_status) {
    this->http_client_.end();
    ESP_LOGW(TAG, "HTTP Request failed at the begin phase.");
    return;
  }
  this->http_client_.setTimeout(this->timeout_);
  this->http_client_.addHeader("Content-Type", "application/json");
  uint32_t start_time = millis();
  int http_code = this->http_client_.sendRequest("POST", json);
  uint32_t duration = millis() - start_time;

  if (http_code < 0) {
    ESP_LOGW(TAG, "HTTP Request failed; URL: %s; Error: %s; Duration: %u ms", url,
             HTTPClient::errorToString(http_code).c_str(), duration);
    this->http_client_.end();
    return;
  }

  if (http_code < 200 || http_code >= 300) {
    ESP_LOGW(TAG, "HTTP Request failed; URL: %s; Code: %d; Duration: %u ms", url, http_code, duration);
    this->http_client_.end();
    return;
  }

  ESP_LOGD(TAG, "HTTP Request completed; URL: %s; Code: %d; Duration: %u ms", url, http_code, duration);
  this->http_client_.end();
}

void StEdgeConnector::schedule_(std::function<void()> &&f) {
#ifdef USE_ESP32
  xSemaphoreTake(this->to_schedule_lock_, portMAX_DELAY);
  to_schedule_.push_back(std::move(f));
  xSemaphoreGive(this->to_schedule_lock_);
#else
  this->defer(std::move(f));
#endif
}

}  // namespace st_edge_connector
}  // namespace esphome
