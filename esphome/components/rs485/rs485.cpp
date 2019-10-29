#include "rs485.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"

namespace esphome {
namespace rs485 {

static const char *TAG = "rs485";

void RS485Component::dump_config() {
    ESP_LOGCONFIG(TAG, "TTL to RS485 module");
    ESP_LOGCONFIG(TAG, "  Baud Rate: %d"          , conf_baud_   );
    ESP_LOGCONFIG(TAG, "  Data bits: %d"          , conf_data_   );
    ESP_LOGCONFIG(TAG, "  Parity: %d"             , conf_parity_ );
    ESP_LOGCONFIG(TAG, "  Stop bits: %d"          , conf_stop_   );
    ESP_LOGCONFIG(TAG, "  RX Receive Timeout: %d" , conf_rx_wait_ );
    ESP_LOGCONFIG(TAG, "  TX Transmission Timeout: %d", conf_tx_wait_ );
    ESP_LOGCONFIG(TAG, "  TX Retry Count: %d"     , conf_tx_retry_cnt_);
    if(prefix_.has_value())
        ESP_LOGCONFIG(TAG, "  Data prefix: %s"    , hexencode(&prefix_.value()[0], prefix_len_).c_str() );
    if(suffix_.has_value())
        ESP_LOGCONFIG(TAG, "  Data suffix: %s"    , hexencode(&suffix_.value()[0], suffix_len_).c_str() );
    ESP_LOGCONFIG(TAG, "  Data checksum: %s"      , YESNO(checksum_));
    if(state_response_.has_value())
        ESP_LOGCONFIG(TAG, "  Data response: %s, offset: %d", hexencode(&state_response_.value().data[0], state_response_.value().data.size()).c_str(), state_response_.value().offset);

    ESP_LOGCONFIG(TAG, "  Listener count: %d"     , listeners_.size() );
}

void RS485Component::setup() {
    #ifdef ARDUINO_ARCH_ESP8266
        byte serialconfig = 0x10;
    #endif
    #ifdef ARDUINO_ARCH_ESP32
        uint32_t serialconfig = 0x8000010;
    #endif

    serialconfig += conf_parity_;
    serialconfig += (conf_data_ - 5) << 2;
    if (conf_stop_ == 2)
        serialconfig += 0x20;
    #ifdef ARDUINO_ARCH_ESP8266
        this->hw_serial_ = &Serial;
        this->hw_serial_->begin(conf_baud_, (SerialConfig)serialconfig);
    #endif
    #ifdef ARDUINO_ARCH_ESP32
        this->hw_serial_ = &Serial2;
        this->hw_serial_->begin(conf_baud_, serialconfig);
    #endif
    ESP_LOGI(TAG, "HW Serial Initaialize.");
    rx_lastTime_ = millis();
}

void RS485Component::loop() {
    if(!init_ && millis() - rx_lastTime_ < 20000)  return;
    else if(!init_) init_ = true;

    // Ack Timeout
    if(tx_ack_wait_ && millis() - tx_start_time_ > conf_tx_wait_) tx_ack_wait_ = false;

    // Receive Process
    rx_proc();
    
    // Publish Receive Packet
    if(rx_bytesRead_ > 0) {
        rx_buffer_[rx_bytesRead_] = 0; // before logging as a char array, zero terminate the last position to be safe.

        if(!validate(&rx_buffer_[0], rx_bytesRead_))
            return;

        // Patket type
        if(state_response_.has_value()) {
            if(compare(&rx_buffer_[prefix_len_], rx_bytesRead_-prefix_len_, &state_response_.value().data[0], state_response_.value().data.size(), state_response_.value().offset))
                response_wait_ = false;
            else
                response_wait_ = true;
        }

        // for Ack
        if(tx_ack_wait_ && tx_current_cmd_) {
            if(compare(&rx_buffer_[prefix_len_], rx_bytesRead_-prefix_len_, &tx_current_cmd_->ack[0], tx_current_cmd_->ack.size(), 0)) {
                tx_current_cmd_ = nullptr;
                tx_ack_wait_ = false;
                tx_retry_cnt_ = 0;

                if(tx_current_device_) {
                    tx_current_device_->callback();
                    tx_current_device_ = nullptr;
                }
                ESP_LOGD(TAG, "Ack: %s, Gap Time: %dms", hexencode(rx_buffer_, rx_bytesRead_).c_str(), millis() - tx_start_time_);
                rx_lastTime_ = millis();
                return;
            }
        }
        
        // Publish State
        bool found = false;
        for (auto *listener : this->listeners_)
            if (listener->parse_data(&rx_buffer_[prefix_len_], rx_bytesRead_-prefix_len_-suffix_len_ )) {
                found = true;
                //if(!listener->is_monitor()) break;
            }

        #ifdef ESPHOME_LOG_HAS_VERY_VERBOSE
            ESP_LOGVV(TAG, "Receive data-> %s, Gap Time: %dms", hexencode(&rx_buffer_[0], rx_bytesRead_).c_str(), millis() - rx_lastTime_);
        #else
            #ifdef ESPHOME_LOG_HAS_VERBOSE
            if (!found) {
                ESP_LOGV(TAG, "Notfound data-> %s", hexencode(&rx_buffer_[0], rx_bytesRead_).c_str());
            }
            #endif
        #endif
        rx_lastTime_ = millis();
    }

    // queue Process
    if(!response_wait_ && (!tx_queue_.empty() || !tx_queue_late_.empty() || (tx_current_cmd_ && !tx_ack_wait_)) && rx_bytesRead_ == 0) {
        tx_proc();
        rx_lastTime_ = millis();
    }
}

void RS485Component::rx_proc() {
    memset(&rx_buffer_, 0, BUFFER_SIZE) ;
    rx_timeOut_ = conf_rx_wait_;
    rx_bytesRead_ = 0;
    while (rx_timeOut_ > 0)
    {
        while (this->hw_serial_->available()) {
            if (rx_bytesRead_ < BUFFER_SIZE) {
                rx_buffer_[rx_bytesRead_] = this->hw_serial_->read();
                rx_bytesRead_++;
                
                if(suffix_.has_value() && rx_bytesRead_ > prefix_len_+suffix_len_ && compare(&rx_buffer_[0], rx_bytesRead_, &suffix_.value()[0], suffix_len_, rx_bytesRead_-suffix_len_)) return;
            }
            else
                this->hw_serial_->read();  // when the buffer is full, just read remaining input, but do not store...
            rx_timeOut_ = conf_rx_wait_; // if serial received, reset timeout counter
        }
        delay(1);
        rx_timeOut_--;
    }
}

void RS485Component::tx_proc() {
    //if(millis()-rx_lastTime_ < conf_rx_wait_*2) return;        

    // Command retry
    if(!tx_ack_wait_ && tx_current_cmd_) {
        if(conf_tx_retry_cnt_ > tx_retry_cnt_) {
            ESP_LOGD(TAG, "Retry count: %d", tx_retry_cnt_);
            write_with_header(tx_current_cmd_->data);
            tx_ack_wait_ = true;
            tx_retry_cnt_++;
            return;
        }
        else {
            tx_current_cmd_ = nullptr;
            tx_ack_wait_ = false;
            tx_retry_cnt_ = 0;

            if(tx_current_device_) {
                tx_current_device_->callback();
                tx_current_device_ = nullptr;
            }
            ESP_LOGD(TAG, "Retry fail.");
        }
    }

    // for State request
    if(tx_queue_.empty() && !tx_queue_late_.empty()) {
        write_with_header(tx_queue_late_.front()->data);

        if(tx_queue_late_.front()->ack.size() > 0) {
            tx_current_cmd_ = tx_queue_late_.front();
            tx_current_device_ = nullptr;
            tx_ack_wait_ = true;
            tx_retry_cnt_ = 1;
        }
        tx_queue_late_.pop();
    }

    // for Command
    else if(!tx_queue_.empty()) {
        write_with_header(tx_queue_.front().cmd->data);
        
        // Pending Ack
        if(tx_queue_.front().cmd->ack.size() > 0) {
            tx_current_cmd_ = tx_queue_.front().cmd;
            tx_current_device_ = tx_queue_.front().device;
            tx_ack_wait_ = true;
            tx_retry_cnt_ = 1;
        }
        else if(tx_queue_.front().device) {
            (*tx_queue_.front().device).callback();
        }
        tx_queue_.pop();
    }
}

void RS485Component::write_with_header(const std::vector<uint8_t> &data) {
    tx_start_time_ = millis();

    // Header
    if(prefix_.has_value()) write_array(prefix_.value());
    
    // Data part
    write_array(data);

    // XOR Checksum
    if(checksum_)
        write_byte(make_checksum(&(data[0]), data.size()));

    // Footer
    if(suffix_.has_value()) write_array(suffix_.value());

    // wait for send
    flush();

    // for Ack wait
    tx_start_time_ = millis();
}

void RS485Component::write_byte(uint8_t data) {
    this->hw_serial_->write(data);
    ESP_LOGD(TAG, "Write byte-> 0x%02X", data);
}

void RS485Component::write_array(const uint8_t *data, const num_t len) {
    this->hw_serial_->write(data, len);
    ESP_LOGD(TAG, "Write array-> %s", hexencode(&data[0], len).c_str());
}

void RS485Component::write_next(const send_hex_t send) {
    if(!init_) {
        if(send.device) (*send.device).callback();
        return;
    }
    tx_queue_.push(send);
}

void RS485Component::write_next_late(const cmd_hex_t *cmd) {
    if(!init_) return;
    tx_queue_late_.push(cmd);
}


void RS485Component::flush() {
    this->hw_serial_->flush();
    ESP_LOGD(TAG, "Flushing... (%dms)", millis() - tx_start_time_);
}

bool RS485Component::validate(const uint8_t *data, const num_t len) {
    if(prefix_.has_value() && !compare(&data[0], len, &prefix_.value()[0], prefix_len_, 0)) {
        ESP_LOGW(TAG, "[Read] Prefix not match: %s", hexencode(&data[0], len).c_str());
        return false;
    }
    if(suffix_.has_value() && !compare(&data[0], len, &suffix_.value()[0], suffix_len_, len-suffix_len_)) {
        ESP_LOGW(TAG, "[Read] Suffix not match: %s", hexencode(&data[0], len).c_str());
        return false;
    }
    if(checksum_ && make_checksum(&data[prefix_len_], len-prefix_len_-suffix_len_-1) != data[len-suffix_len_-1]) {
        ESP_LOGW(TAG, "[Read] Checksum error: %s", hexencode(&data[0], len).c_str());
        return false;
    }
    return true;
}

uint8_t RS485Component::make_checksum(const uint8_t *data, const num_t len) const {
    if (this->checksum_f_.has_value()) {
        return (*checksum_f_)(data, len);
    }
    else {
        // CheckSum8 Xor (Default)
        uint8_t crc = 0;
        if(prefix_.has_value())
            for(num_t i=0; i<prefix_len_; i++)
                crc ^= prefix_.value()[i];
        for(num_t i=0; i<len; i++)
            crc ^= data[i];
        return crc;
    }
}



void RS485Device::update() {
    if(!command_state_.has_value()) return;

    ESP_LOGD(TAG, "'%s' update(): Request current state...", device_name_->c_str());
    parent_->write_next_late(&command_state_.value());
}

void RS485Device::dump_rs485_device_config(const char *TAG) {
    ESP_LOGCONFIG(TAG, "  Device: %s", hexencode(&device_.data[0], device_.data.size()).c_str(), device_.offset);
    if(sub_device_.has_value())
        ESP_LOGCONFIG(TAG, "  Sub device: %s, offset: %d", hexencode(&sub_device_.value().data[0], sub_device_.value().data.size()).c_str(), sub_device_.value().offset);

    if(state_on_.has_value())
        ESP_LOGCONFIG(TAG, "  State ON: %s, offset: %d", hexencode(&state_on_.value().data[0], state_on_.value().data.size()).c_str(), state_on_.value().offset);
    if(state_off_.has_value())
        ESP_LOGCONFIG(TAG, "  State OFF: %s, offset: %d", hexencode(&state_off_.value().data[0], state_off_.value().data.size()).c_str(), state_off_.value().offset);
    
    if(command_on_.has_value())
        ESP_LOGCONFIG(TAG, "  Command ON: %s", hexencode(&command_on_.value().data[0], command_on_.value().data.size()).c_str());
    if(command_on_.has_value())
        ESP_LOGCONFIG(TAG, "  Command ON Ack: %s", hexencode(&command_on_.value().ack[0], command_on_.value().ack.size()).c_str());

    if(command_off_.has_value())
        ESP_LOGCONFIG(TAG, "  Command OFF: %s", hexencode(&command_off_.value().data[0], command_off_.value().data.size()).c_str());
    if(command_off_.has_value())
        ESP_LOGCONFIG(TAG, "  Command OFF Ack: %s", hexencode(&command_off_.value().ack[0], command_off_.value().ack.size()).c_str());

    if(command_state_.has_value())
        ESP_LOGCONFIG(TAG, "  Command State: %s", hexencode(&command_state_.value().data[0], command_state_.value().data.size()).c_str());
    if(command_state_.has_value()) {
        ESP_LOGCONFIG(TAG, "  Command State Ack: %s", hexencode(&command_state_.value().ack[0], command_state_.value().ack.size()).c_str());
        ESP_LOGCONFIG(TAG, "  Status request interval: %u", update_interval_ );
    }
    LOG_UPDATE_INTERVAL(this);
}

bool RS485Device::parse_data(const uint8_t *data, const num_t len) {
    if(tx_pending_) return false;

    if(!compare(&data[0], len, &device_.data[0], device_.data.size(), device_.offset))
        return false;
    else if(sub_device_.has_value() && !compare(&data[0], len, &sub_device_.value().data[0], sub_device_.value().data.size(), sub_device_.value().offset))
        return false;
    
    // Turn OFF Message
    if(state_off_.has_value() && compare(&data[0], len, &state_off_.value().data[0], state_off_.value().data.size(), state_off_.value().offset)) {
        if(!publish(false)) publish(data, len);
        return true;
    }
    // Turn ON Message
    else if(state_on_.has_value() && compare(&data[0], len, &state_on_.value().data[0], state_on_.value().data.size(), state_on_.value().offset)) {
        if(!publish(true)) publish(data, len);
        return true;
    }

    // Other Message
    publish(data, len);
    return true;
}

void RS485Device::write_with_header(const cmd_hex_t *cmd) {
    tx_pending_ = true;
    parent_->write_next({this, cmd});
}



bool SerialMonitor::parse_data(const uint8_t *data, const num_t len) {
    bool found = false;
    if(filters_.size() == 0) found = true;
    else {
        for(hex_t filter : filters_) {
            found = compare(&data[0], len, &(filter.data[0]), filter.data.size(), filter.offset);
            if(found) break;
        }
    }

    if(found)
        ESP_LOGI(TAG, "Serial Monitor: %s", hexencode(&data[0], len).c_str());
        
    return found;
}


std::string hexencode(const uint8_t *raw_data, num_t len) {
    char buf[20];
    std::string res;
    for(num_t i=0; i<len; i++) {
        sprintf(buf, "0x%02X ", raw_data[i]);
        res += buf;
    }
    sprintf(buf, "(%d byte)", len);
    res += buf;
    return res;
}

bool compare(const uint8_t *data1, const num_t len1, const uint8_t *data2, const num_t len2, const num_t offset) {
    if(len1-offset < len2)
        return false;
    //ESP_LOGD(TAG, "compare(0x%02X, 0x%02X, %d)=> %d", data1[offset], data2[0], len2, memcmp(&data1[offset], &data2[0], len2));
    return memcmp(&data1[offset], &data2[0], len2) == 0;
}

float hex_to_float(const uint8_t *data, const num_t len, const num_t precision) {
    unsigned int val = 0;
    for(num_t i=0; i<len; i++) {
        val = (val << 8) | data[i];
    }
    return val/powf(10, precision);
}


}  // namespace rs485
}  // namespace esphome