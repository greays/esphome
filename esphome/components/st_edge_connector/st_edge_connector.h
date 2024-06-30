#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"
#include "esphome/core/component.h"
#include "esphome/core/controller.h"
#include "esphome/core/defines.h"
#include "WiFiUdp.h"

#ifdef USE_ARDUINO
#include <ESPAsyncWebServer.h>
#elif USE_ESP_IDF
#include "esphome/core/hal.h"
#include "esphome/components/web_server_idf/web_server_idf.h"
#endif

#ifdef USE_ESP32
#include <HTTPClient.h>
#include <deque>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif
#ifdef USE_ESP8266
#include <ESP8266HTTPClient.h>
#endif

namespace esphome {
namespace st_edge_connector {

/** SmartThings Address by Entity */
struct st_address_t
{
  const std::string id;
  const uint32_t key;
  char callback_url[50];
};

/** SmartThings to ESPHome Request URL Struct (/switch.room/control) */
struct cmd_url_t
{
  const char *type;
  const char *id;
  const char *cmd;
};

enum JsonType { PUSH, REFRESH };

class StEdgeConnector : public Component, public Controller {
 public:
  StEdgeConnector();

  void set_pairing_mode_(bool pairing_mode) { pairing_mode_ = pairing_mode; }
  void set_port(uint16_t port) { port_ = port; }
  void set_hub_ip(IPAddress hub_ip) { hub_ip_ = hub_ip; }
  void set_include_internal(bool include_internal) { include_internal_ = include_internal; }
 
  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  /// Setup the internal web server and register handlers.
  void setup() override;
  void loop() override;

  void dump_config() override;
  float get_setup_priority() const override;

#ifdef USE_BINARY_SENSOR
  void on_binary_sensor_update(binary_sensor::BinarySensor *obj, bool state) override;
  /// Handle a binary sensor request under '/binary_sensor.<id>/<cmd>'.
  void handle_binary_sensor_request(AsyncWebServerRequest *request, const cmd_url_t &url);
  /// Dump the binary sensor state with its value as a JSON string.
  std::string binary_sensor_json(binary_sensor::BinarySensor *obj, bool value, JsonType type);
#endif
#ifdef USE_SENSOR
  void on_sensor_update(sensor::Sensor *obj, float state) override;
  /// Handle a sensor request under '/sensor.<id>/<cmd>'.
  void handle_sensor_request(AsyncWebServerRequest *request, const cmd_url_t &url);
  /// Dump the sensor state with its value as a JSON string.
  std::string sensor_json(sensor::Sensor *obj, float value, JsonType type);
#endif
#ifdef USE_SWITCH
  void on_switch_update(switch_::Switch *obj, bool state) override;
  /// Handle a switch request under '/switch.<id>/<cmd>'.
  void handle_switch_request(AsyncWebServerRequest *request, const cmd_url_t &url);
  /// Dump the switch state with its value as a JSON string.
  std::string switch_json(switch_::Switch *obj, bool value, JsonType type);
#endif
#ifdef USE_LIGHT
  void on_light_update(light::LightState *obj) override;
  /// Handle a light request under '/light.<id>/<cmd>'.
  void handle_light_request(AsyncWebServerRequest *request, const cmd_url_t &url);
  /// Dump the light state with its value as a JSON string.
  std::string light_json(light::LightState *obj, JsonType type);
#endif

 protected:
  void schedule_(std::function<void()> &&f);
  WiFiUDP udp_client_{};
  const IPAddress udp_mcast_group_{239, 255, 255, 250};
  const uint16_t  udp_mcast_port_{33333};
  bool udp_init_{false};
  char buffer_[1024]{};
  bool pairing_mode_{true};
  bool pairing_all_device_{false};
  size_t pairing_device_cnt_{0};

  /** Join Multicast Group for Pairing */
  void join_multicast_group();

  /** Process Pairing (UDP) */
  void process_pairing(const char *data);

  /** Control Server Daemon */
  std::shared_ptr<AsyncWebServer> server_{nullptr};
  /** Control Server Port (default: 8123) */
  uint16_t port_{8123};

  /** Web Handler */
  void web_handler();
  /** Handle Ping Request (HTTP Response) */
  void handle_ping_request(AsyncWebServerRequest *request, const cmd_url_t &url);

  /** SmartThings Hub IP (Allow control only from set IP) */ 
  IPAddress hub_ip_{};
  /** SmartThings IP & Port List (Information received from ST Hub) */
  std::vector<st_address_t> st_address_list_{};
  st_address_t* get_st_address_(const std::string id) {
    for(int i=0; i<this->st_address_list_.size(); i++) {
      if(this->st_address_list_[i].id == id)
        return &this->st_address_list_[i];
    }
    return nullptr;
  }

  /** Includes internal devices */
  bool include_internal_{false};

  /** Process Callback (HTTP Request)*/
  void process_callback(st_address_t *addr, const char *json);
  HTTPClient http_client_{};
  uint16_t timeout_{2000};

#ifdef USE_ESP8266
  std::shared_ptr<WiFiClient> wifi_client_;
#endif

#ifdef USE_ESP32
  std::deque<std::function<void()>> to_schedule_;
  SemaphoreHandle_t to_schedule_lock_;
#endif

};


}  // namespace st_edge_connector
}  // namespace esphome
