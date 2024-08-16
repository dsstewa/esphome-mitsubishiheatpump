/**
 * espmhp.cpp
 *
 * Implementation of esphome-mitsubishiheatpump
 *
 * Author: Geoff Davis <geoff@geoffdavis.com>
 * Author: Phil Genera @pgenera on Github.
 * Author: Barry Loong @loongyh on GitHub.
 * Author: @am-io on Github.
 * Author: @nao-pon on Github.
 * Author: Simon Knopp @sijk on Github
 * Author: Paul Murphy @donutsoft on GitHub
 * Last Updated: 2023-04-22
 * License: BSD
 *
 * Requirements:
 * - https://github.com/SwiCago/HeatPump
 * - ESPHome 1.18.0 or greater
 */

#include "espmhp.h"
using namespace esphome;

/**
 * Create a new MitsubishiHeatPump object
 *
 * Args:
 *   hw_serial: pointer to an Arduino HardwareSerial instance
 *   poll_interval: polling interval in milliseconds
 */
MitsubishiHeatPump::MitsubishiHeatPump(
        HardwareSerial* hw_serial,
        uint32_t poll_interval
) :
    PollingComponent{poll_interval}, // member initializers list
    hw_serial_{hw_serial}
{
    this->traits_.set_supports_action(true);
    this->traits_.set_supports_current_temperature(true);
    this->traits_.set_supports_two_point_target_temperature(false);
    this->traits_.set_visual_min_temperature(ESPMHP_MIN_TEMPERATURE);
    this->traits_.set_visual_max_temperature(ESPMHP_MAX_TEMPERATURE);
    this->traits_.set_visual_temperature_step(ESPMHP_TEMPERATURE_STEP);

    // Assume a succesful connection was made to the ESPHome controller on
    // launch.
    this->ping();
}

bool MitsubishiHeatPump::verify_serial() {
    if (!this->get_hw_serial_()) {
        ESP_LOGCONFIG(
                TAG,
                "No HardwareSerial was provided. "
                "Software serial ports are unsupported by this component."
        );
        return false;
    }

#ifdef USE_LOGGER
    if (this->get_hw_serial_() == logger::global_logger->get_hw_serial()) {
        ESP_LOGW(TAG, "  You're using the same serial port for logging"
                " and the MitsubishiHeatPump component. Please disable"
                " logging over the serial port by setting"
                " logger:baud_rate to 0.");
        return false;
    }
#endif
    // unless something went wrong, assume we have a valid serial configuration
    return true;
}

void MitsubishiHeatPump::banner() {
    ESP_LOGI(TAG, "ESPHome MitsubishiHeatPump version %s",
            ESPMHP_VERSION);
}

void MitsubishiHeatPump::update() {
    // This will be called every "update_interval" milliseconds.
    //this->dump_config();
    this->hp->sync();
#ifndef USE_CALLBACKS
    this->hpSettingsChanged();
    heatpumpStatus currentStatus = hp->getStatus();
    this->hpStatusChanged(currentStatus);
#endif
    this->enforce_remote_temperature_sensor_timeout();
}

void MitsubishiHeatPump::set_baud_rate(int baud) {
    this->baud_ = baud;
}

void MitsubishiHeatPump::set_rx_pin(int rx_pin) {
    this->rx_pin_ = rx_pin;
}

void MitsubishiHeatPump::set_tx_pin(int tx_pin) {
    this->tx_pin_ = tx_pin;
}

/**
 * Get our supported traits.
 *
 * Note:
 * Many of the following traits are only available in the 1.5.0 dev train of
 * ESPHome, particularly the Dry operation mode, and several of the fan modes.
 *
 * Returns:
 *   This class' supported climate::ClimateTraits.
 */
climate::ClimateTraits MitsubishiHeatPump::traits() {
    return traits_;
}

/**
 * Modify our supported traits.
 *
 * Returns:
 *   A reference to this class' supported climate::ClimateTraits.
 */
climate::ClimateTraits& MitsubishiHeatPump::config_traits() {
    return traits_;
}

void MitsubishiHeatPump::update_swing_horizontal(const std::string &swing) {
    this->horizontal_swing_state_ = swing;

    if (this->horizontal_vane_select_ != nullptr &&
        this->horizontal_vane_select_->state != this->horizontal_swing_state_) {
        this->horizontal_vane_select_->publish_state(
            this->horizontal_swing_state_);  // Set current horizontal swing
                                             // position
    }
}

void MitsubishiHeatPump::update_swing_vertical(const std::string &swing) {
    this->vertical_swing_state_ = swing;

    if (this->vertical_vane_select_ != nullptr &&
        this->vertical_vane_select_->state != this->vertical_swing_state_) {
        this->vertical_vane_select_->publish_state(
            this->vertical_swing_state_);  // Set current vertical swing position
    }
}

void MitsubishiHeatPump::set_vertical_vane_select(
    select::Select *vertical_vane_select) {
    this->vertical_vane_select_ = vertical_vane_select;
    this->vertical_vane_select_->add_on_state_callback(
        [this](const std::string &value, size_t index) {
            if (value == this->vertical_swing_state_) return;
            this->on_vertical_swing_change(value);
        });
}

void MitsubishiHeatPump::set_horizontal_vane_select(
    select::Select *horizontal_vane_select) {
      this->horizontal_vane_select_ = horizontal_vane_select;
      this->horizontal_vane_select_->add_on_state_callback(
          [this](const std::string &value, size_t index) {
              if (value == this->horizontal_swing_state_) return;
              this->on_horizontal_swing_change(value);
          });
}

void MitsubishiHeatPump::on_vertical_swing_change(const std::string &swing) {
    ESP_LOGD(TAG, "Setting vertical swing position");
    bool updated = false;

    if (swing == "swing") {
        hp->setVaneSetting("SWING");
        updated = true;
    } else if (swing == "auto") {
        hp->setVaneSetting("AUTO");
        updated = true;
    } else if (swing == "up") {
        hp->setVaneSetting("1");
        updated = true;
    } else if (swing == "up_center") {
        hp->setVaneSetting("2");
        updated = true;
    } else if (swing == "center") {
        hp->setVaneSetting("3");
        updated = true;
    } else if (swing == "down_center") {
        hp->setVaneSetting("4");
        updated = true;
    } else if (swing == "down") {
        hp->setVaneSetting("5");
        updated = true;
    } else {
        ESP_LOGW(TAG, "Invalid vertical vane position %s", swing);
    }

    ESP_LOGD(TAG, "Vertical vane - Was HeatPump updated? %s", YESNO(updated));

    // and the heat pump:
    hp->update();
}

void MitsubishiHeatPump::on_horizontal_swing_change(const std::string &swing) {
    ESP_LOGD(TAG, "Setting horizontal swing position");
    bool updated = false;

    if (swing == "swing") {
        hp->setWideVaneSetting("SWING");
        updated = true;
    } else if (swing == "auto") {
        hp->setWideVaneSetting("<>");
        updated = true;
    } else if (swing == "left") {
        hp->setWideVaneSetting("<<");
        updated = true;
    } else if (swing == "left_center") {
        hp->setWideVaneSetting("<");
        updated = true;
    } else if (swing == "center") {
        hp->setWideVaneSetting("|");
        updated = true;
    } else if (swing == "right_center") {
        hp->setWideVaneSetting(">");
        updated = true;
    } else if (swing == "right") {
        hp->setWideVaneSetting(">>");
        updated = true;
    } else {
        ESP_LOGW(TAG, "Invalid horizontal vane position %s", swing);
    }

    ESP_LOGD(TAG, "Horizontal vane - Was HeatPump updated? %s", YESNO(updated));

    // and the heat pump:
    hp->update();
 }

/**
 * Implement control of a MitsubishiHeatPump.
 *
 * Maps HomeAssistant/ESPHome modes to Mitsubishi modes.
 */
void MitsubishiHeatPump::control(const climate::ClimateCall &call) {
    ESP_LOGV(TAG, "Control called.");

    bool updated = false;
    bool has_mode = call.get_mode().has_value();
    bool has_temp = call.get_target_temperature().has_value();
    if (has_mode){
        this->mode = *call.get_mode();
    }

    if (last_remote_temperature_sensor_update_.has_value()) {
        // Some remote temperature sensors will only issue updates when a change
        // in temperature occurs. 

        // Assume a case where the idle sensor timeout is 12hrs and operating 
        // timeout is 1hr. If the user changes the HP setpoint after 1.5hrs, the
        // machine will switch to operating mode, the remote temperature 
        // reading will expire and the HP will revert to it's internal 
        // temperature sensor.

        // This change ensures that if the user changes the machine setpoint,
        // the remote sensor has an opportunity to issue an update to reflect
        // the new change in temperature.
        last_remote_temperature_sensor_update_ =
            std::chrono::steady_clock::now();
    }

    switch (this->mode) {
        case climate::CLIMATE_MODE_COOL:
            hp->setModeSetting("COOL");
            hp->setPowerSetting("ON");
            ESP_LOGI(TAG, "COOL SWITCH STATEMENT");
            ESP_LOGI(TAG, "Mode: COOL");
            ESP_LOGI(TAG, "Power: ON");
            ESP_LOGI(TAG, "Has Mode: %s", YESNO(has_mode));
            ESP_LOGI(TAG, "Has Temp: %s", YESNO(has_temp));
            ESP_LOGI(TAG, "Cool Setpoint: %.1f", cool_setpoint.value_or(-1));
            ESP_LOGI(TAG, "Target Temperature: %.1f", this->target_temperature);
            if (has_mode){
                if (cool_setpoint.has_value() && !has_temp) {
                    hp->setTemperature(cool_setpoint.value());
                    this->target_temperature = cool_setpoint.value();
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_HEAT:
            hp->setModeSetting("HEAT");
            hp->setPowerSetting("ON");
            if (has_mode){
                if (heat_setpoint.has_value() && !has_temp) {
                    hp->setTemperature(heat_setpoint.value());
                    this->target_temperature = heat_setpoint.value();
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_DRY:
            hp->setModeSetting("DRY");
            hp->setPowerSetting("ON");
            if (has_mode){
                this->action = climate::CLIMATE_ACTION_DRYING;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_HEAT_COOL:
            hp->setModeSetting("AUTO");
            hp->setPowerSetting("ON");
            if (has_mode){
                if (auto_setpoint.has_value() && !has_temp) {
                    hp->setTemperature(auto_setpoint.value());
                    this->target_temperature = auto_setpoint.value();
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
            }
            updated = true;
            break;
        case climate::CLIMATE_MODE_FAN_ONLY:
            hp->setModeSetting("FAN");
            hp->setPowerSetting("ON");
            if (has_mode){
                this->action = climate::CLIMATE_ACTION_FAN;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_OFF:
        default:
            if (has_mode){
                hp->setPowerSetting("OFF");
                this->action = climate::CLIMATE_ACTION_OFF;
                updated = true;
            }
            break;
    }

    if (has_temp){
        float rounded_temp = roundCelsiusValues(*call.get_target_temperature());
        ESP_LOGV(
            "control", "Sending target temp: %.1f",
            rounded_temp
        );
        hp->setTemperature(rounded_temp);
        this->target_temperature = rounded_temp;
        updated = true;
    }

    //const char* FAN_MAP[6]         = {"AUTO", "QUIET", "1", "2", "3", "4"};
    if (call.get_fan_mode().has_value()) {
        ESP_LOGV("control", "Requested fan mode is %s",
                 climate::climate_fan_mode_to_string(*call.get_fan_mode()));
        this->fan_mode = *call.get_fan_mode();
        switch(*call.get_fan_mode()) {
            case climate::CLIMATE_FAN_OFF:
                hp->setPowerSetting("OFF");
                updated = true;
                break;
            case climate::CLIMATE_FAN_DIFFUSE:
                hp->setFanSpeed("QUIET");
                updated = true;
                break;
            case climate::CLIMATE_FAN_LOW:
                hp->setFanSpeed("1");
                updated = true;
                break;
            case climate::CLIMATE_FAN_MEDIUM:
                hp->setFanSpeed("2");
                updated = true;
                break;
            case climate::CLIMATE_FAN_MIDDLE:
                hp->setFanSpeed("3");
                updated = true;
                break;
            case climate::CLIMATE_FAN_HIGH:
                hp->setFanSpeed("4");
                updated = true;
                break;
            case climate::CLIMATE_FAN_ON:
            case climate::CLIMATE_FAN_AUTO:
            default:
                hp->setFanSpeed("AUTO");
                updated = true;
                break;
        }
    }


    ESP_LOGV(TAG, "in the swing mode stage");
    //const char* VANE_MAP[7]        = {"AUTO", "1", "2", "3", "4", "5", "SWING"};
    if (call.get_swing_mode().has_value()) {
        ESP_LOGV(TAG, "control - requested swing mode is %s",
                climate::climate_swing_mode_to_string(*call.get_swing_mode()));

        this->swing_mode = *call.get_swing_mode();
        switch(*call.get_swing_mode()) {
            case climate::CLIMATE_SWING_OFF:
                hp->setVaneSetting("AUTO");
                hp->setWideVaneSetting("|");
                updated = true;
                break;
            case climate::CLIMATE_SWING_VERTICAL:
                hp->setVaneSetting("SWING");
                hp->setWideVaneSetting("|");
                updated = true;
                break;
            case climate::CLIMATE_SWING_HORIZONTAL:
                hp->setVaneSetting("3");
                hp->setWideVaneSetting("SWING");
                updated = true;
                break;
            case climate::CLIMATE_SWING_BOTH:
                hp->setVaneSetting("SWING");
                hp->setWideVaneSetting("SWING");
                updated = true;
                break;
            default:
                ESP_LOGW(TAG, "control - received unsupported swing mode request.");

        }
    }
    ESP_LOGD(TAG, "control - Was HeatPump updated? %s", YESNO(updated));

    // send the update back to esphome:
    this->publish_state();
    // and the heat pump:
   // hp->update();   uncomment 
}

void MitsubishiHeatPump::hpSettingsChanged() {
    heatpumpSettings currentSettings = hp->getSettings();

    if (currentSettings.power == NULL) {
        /*
         * We should always get a valid pointer here once the HeatPump
         * component fully initializes. If HeatPump hasn't read the settings
         * from the unit yet (hp->connect() doesn't do this, sadly), we'll need
         * to punt on the update. Likely not an issue when run in callback
         * mode, but that isn't working right yet.
         */
        ESP_LOGW(TAG, "Waiting for HeatPump to read the settings the first time.");
        esphome::delay(10);
        return;
    }

    /*
     * ************ HANDLE POWER AND MODE CHANGES ***********
     * https://github.com/geoffdavis/HeatPump/blob/stream/src/HeatPump.h#L125
     * const char* POWER_MAP[2]       = {"OFF", "ON"};
     * const char* MODE_MAP[5]        = {"HEAT", "DRY", "COOL", "FAN", "AUTO"};
     */
    if (strcmp(currentSettings.power, "ON") == 0) {
        if (strcmp(currentSettings.mode, "HEAT") == 0) {
            this->mode = climate::CLIMATE_MODE_HEAT;
            if (heat_setpoint != currentSettings.temperature) {
                heat_setpoint = currentSettings.temperature;
                save(currentSettings.temperature, heat_storage);
            }
            this->action = climate::CLIMATE_ACTION_IDLE;
        } else if (strcmp(currentSettings.mode, "DRY") == 0) {
            this->mode = climate::CLIMATE_MODE_DRY;
            this->action = climate::CLIMATE_ACTION_DRYING;
        } else if (strcmp(currentSettings.mode, "COOL") == 0) {
            this->mode = climate::CLIMATE_MODE_COOL;
            if (cool_setpoint != currentSettings.temperature) {
                cool_setpoint = currentSettings.temperature;
                save(currentSettings.temperature, cool_storage);
            }
            this->action = climate::CLIMATE_ACTION_IDLE;
        } else if (strcmp(currentSettings.mode, "FAN") == 0) {
            this->mode = climate::CLIMATE_MODE_FAN_ONLY;
            this->action = climate::CLIMATE_ACTION_FAN;
        } else if (strcmp(currentSettings.mode, "AUTO") == 0) {
            this->mode = climate::CLIMATE_MODE_HEAT_COOL;
            if (auto_setpoint != currentSettings.temperature) {
                auto_setpoint = currentSettings.temperature;
                save(currentSettings.temperature, auto_storage);
            }
            this->action = climate::CLIMATE_ACTION_IDLE;
        } else {
            ESP_LOGW(
                    TAG,
                    "Unknown climate mode value %s received from HeatPump",
                    currentSettings.mode
            );
        }
    } else {
        this->mode = climate::CLIMATE_MODE_OFF;
        this->action = climate::CLIMATE_ACTION_OFF;
    }

    ESP_LOGI(TAG, "Climate mode is: %i", this->mode);

    /*
     * ******* HANDLE FAN CHANGES ********
     *
     * const char* FAN_MAP[6]         = {"AUTO", "QUIET", "1", "2", "3", "4"};
     */
    if (strcmp(currentSettings.fan, "QUIET") == 0) {
        this->fan_mode = climate::CLIMATE_FAN_DIFFUSE;
    } else if (strcmp(currentSettings.fan, "1") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_LOW;
    } else if (strcmp(currentSettings.fan, "2") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
    } else if (strcmp(currentSettings.fan, "3") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
    } else if (strcmp(currentSettings.fan, "4") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_HIGH;
    } else { //case "AUTO" or default:
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
    }
    ESP_LOGI(TAG, "Fan mode is: %i", this->fan_mode.value_or(-1));

    /* ******** HANDLE MITSUBISHI VANE CHANGES ********
     * const char* VANE_MAP[7]        = {"AUTO", "1", "2", "3", "4", "5", "SWING"};
     */
    if (strcmp(currentSettings.vane, "SWING") == 0 &&
        strcmp(currentSettings.wideVane, "SWING") == 0) {
        this->swing_mode = climate::CLIMATE_SWING_BOTH;
    } else if (strcmp(currentSettings.vane, "SWING") == 0) {
        this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
    } else if (strcmp(currentSettings.wideVane, "SWING") == 0) {
        this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
    } else {
        this->swing_mode = climate::CLIMATE_SWING_OFF;
    }
    ESP_LOGI(TAG, "Swing mode is: %i", this->swing_mode);
    if (strcmp(currentSettings.vane, "SWING") == 0) {
        this->update_swing_vertical("swing");
    } else if (strcmp(currentSettings.vane, "AUTO") == 0) {
        this->update_swing_vertical("auto");
    } else if (strcmp(currentSettings.vane, "1") == 0) {
        this->update_swing_vertical("up");
    } else if (strcmp(currentSettings.vane, "2") == 0) {
        this->update_swing_vertical("up_center");
    } else if (strcmp(currentSettings.vane, "3") == 0) {
        this->update_swing_vertical("center");
    } else if (strcmp(currentSettings.vane, "4") == 0) {
        this->update_swing_vertical("down_center");
    } else if (strcmp(currentSettings.vane, "5") == 0) {
        this->update_swing_vertical("down");
    }

    ESP_LOGI(TAG, "Vertical vane mode is: %s", currentSettings.vane);

    if (strcmp(currentSettings.wideVane, "SWING") == 0) {
        this->update_swing_horizontal("swing");
    } else if (strcmp(currentSettings.wideVane, "<>") == 0) {
        this->update_swing_horizontal("auto");
    } else if (strcmp(currentSettings.wideVane, "<<") == 0) {
        this->update_swing_horizontal("left");
    } else if (strcmp(currentSettings.wideVane, "<") == 0) {
        this->update_swing_horizontal("left_center");
    } else if (strcmp(currentSettings.wideVane, "|") == 0) {
        this->update_swing_horizontal("center");
    } else if (strcmp(currentSettings.wideVane, ">") == 0) {
        this->update_swing_horizontal("right_center");
    } else if (strcmp(currentSettings.wideVane, ">>") == 0) {
        this->update_swing_horizontal("right");
    }

    ESP_LOGI(TAG, "Horizontal vane mode is: %s", currentSettings.wideVane);

    /*
     * ******** HANDLE TARGET TEMPERATURE CHANGES ********
     */
    this->target_temperature = currentSettings.temperature;
    ESP_LOGI(TAG, "Target temp is: %f", this->target_temperature);
    ESP_LOGI(TAG, "Rounded Target temp is: %.1f", this->target_temperature);
    /*
     * ******** Publish state back to ESPHome. ********
     */
    this->publish_state();
}

/**
 * Report changes in the current temperature sensed by the HeatPump.
 */
void MitsubishiHeatPump::hpStatusChanged(heatpumpStatus currentStatus) {
    this->current_temperature = currentStatus.roomTemperature;
    switch (this->mode) {
        case climate::CLIMATE_MODE_HEAT:
            if (currentStatus.operating) {
                this->action = climate::CLIMATE_ACTION_HEATING;
            }
            else {
                this->action = climate::CLIMATE_ACTION_IDLE;
            }
            break;
        case climate::CLIMATE_MODE_COOL:
            if (currentStatus.operating) {
                this->action = climate::CLIMATE_ACTION_COOLING;
            }
            else {
                this->action = climate::CLIMATE_ACTION_IDLE;
            }
            break;
        case climate::CLIMATE_MODE_HEAT_COOL:
            this->action = climate::CLIMATE_ACTION_IDLE;
            if (currentStatus.operating) {
              if (this->current_temperature > this->target_temperature) {
                  this->action = climate::CLIMATE_ACTION_COOLING;
              } else if (this->current_temperature < this->target_temperature) {
                  this->action = climate::CLIMATE_ACTION_HEATING;
              }
            }
            break;
        case climate::CLIMATE_MODE_DRY:
            if (currentStatus.operating) {
                this->action = climate::CLIMATE_ACTION_DRYING;
            }
            else {
                this->action = climate::CLIMATE_ACTION_IDLE;
            }
            break;
        case climate::CLIMATE_MODE_FAN_ONLY:
            this->action = climate::CLIMATE_ACTION_FAN;
            break;
        default:
            this->action = climate::CLIMATE_ACTION_OFF;
    }

    this->operating_ = currentStatus.operating;

    this->publish_state();
}

void MitsubishiHeatPump::set_remote_temperature(float temp) {
    ESP_LOGD(TAG, "Setting remote temp: %.1f", temp);
    if (temp > 0) {
        last_remote_temperature_sensor_update_ = 
            std::chrono::steady_clock::now();
    } else {
        last_remote_temperature_sensor_update_.reset();
    }

    this->hp->setRemoteTemperature(temp);
}

void MitsubishiHeatPump::ping() {
    ESP_LOGD(TAG, "Ping request received");
    last_ping_request_ = std::chrono::steady_clock::now();
}

void MitsubishiHeatPump::set_remote_operating_timeout_minutes(int minutes) {
    ESP_LOGD(TAG, "Setting remote operating timeout time: %d minutes", minutes);
    remote_operating_timeout_ = std::chrono::minutes(minutes);
}

void MitsubishiHeatPump::set_remote_idle_timeout_minutes(int minutes) {
    ESP_LOGD(TAG, "Setting remote idle timeout time: %d minutes", minutes);
    remote_idle_timeout_ = std::chrono::minutes(minutes);
}

void MitsubishiHeatPump::set_remote_ping_timeout_minutes(int minutes) {
    ESP_LOGD(TAG, "Setting remote ping timeout time: %d minutes", minutes);
    remote_ping_timeout_ = std::chrono::minutes(minutes);
}

void MitsubishiHeatPump::enforce_remote_temperature_sensor_timeout() {
    // Handle ping timeouts.
    if (remote_ping_timeout_.has_value() && last_ping_request_.has_value()) {
        auto time_since_last_ping =
            std::chrono::steady_clock::now() - last_ping_request_.value();
        if(time_since_last_ping > remote_ping_timeout_.value()) {
            ESP_LOGW(TAG, "Ping timeout.");
            this->set_remote_temperature(0);
            last_ping_request_.reset();
            return;
        }
    }

    // Handle set_remote_temperature timeouts.
    auto remote_set_temperature_timeout =
        this->operating_ ? remote_operating_timeout_ : remote_idle_timeout_;
    if (remote_set_temperature_timeout.has_value() &&
            last_remote_temperature_sensor_update_.has_value()) {
        auto time_since_last_temperature_update =
            std::chrono::steady_clock::now() - last_remote_temperature_sensor_update_.value();
        if (time_since_last_temperature_update > remote_set_temperature_timeout.value()) {
            ESP_LOGW(TAG, "Set remote temperature timeout, operating=%d", this->operating_);
            this->set_remote_temperature(0);
            return;
        }
    }
}

void MitsubishiHeatPump::setup() {
    // This will be called by App.setup()
    this->banner();
    ESP_LOGCONFIG(TAG, "Setting up UART...");

    if (!this->verify_serial()) {
        this->mark_failed();
        return;
    }

    ESP_LOGCONFIG(TAG, "Initializing new HeatPump object.");
    this->hp = new HeatPump();
    this->current_temperature = NAN;
    this->target_temperature = NAN;
    this->fan_mode = climate::CLIMATE_FAN_OFF;
    this->swing_mode = climate::CLIMATE_SWING_OFF;
    this->vertical_swing_state_ = "auto";
    this->horizontal_swing_state_ = "auto";

#ifdef USE_CALLBACKS
    hp->setSettingsChangedCallback(
            [this]() {
                this->hpSettingsChanged();
            }
    );

    hp->setStatusChangedCallback(
            [this](heatpumpStatus currentStatus) {
                this->hpStatusChanged(currentStatus);
            }
    );

    hp->setPacketCallback(this->log_packet);    
#endif

    ESP_LOGCONFIG(
            TAG,
            "hw_serial(%p) is &Serial(%p)? %s",
            this->get_hw_serial_(),
            &Serial,
            YESNO((void *)this->get_hw_serial_() == (void *)&Serial)
    );

    ESP_LOGCONFIG(TAG, "Calling hp->connect(%p)", this->get_hw_serial_());
    if (hp->connect(this->get_hw_serial_(), this->baud_, this->rx_pin_, this->tx_pin_)) {
        hp->sync();
    }
    else {
        ESP_LOGCONFIG(
                TAG,
                "Connection to HeatPump failed."
                " Marking MitsubishiHeatPump component as failed."
        );
        this->mark_failed();
    }

    // create various setpoint persistence:
    cool_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 1);
    heat_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 2);
    auto_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 3);

    // load values from storage:
    cool_setpoint = load(cool_storage);
    heat_setpoint = load(heat_storage);
    auto_setpoint = load(auto_storage);

    this->dump_config();
}

/**
 * The ESP only has a few bytes of rtc storage, so instead
 * of storing floats directly, we'll store the number of
 * TEMPERATURE_STEPs from MIN_TEMPERATURE.
 **/
void MitsubishiHeatPump::save(float value, ESPPreferenceObject& storage) {
    uint8_t steps = (value - ESPMHP_MIN_TEMPERATURE) / ESPMHP_TEMPERATURE_STEP;
    storage.save(&steps);
}

optional<float> MitsubishiHeatPump::load(ESPPreferenceObject& storage) {
    uint8_t steps = 0;
    if (!storage.load(&steps)) {
        return {};
    }
    return ESPMHP_MIN_TEMPERATURE + (steps * ESPMHP_TEMPERATURE_STEP);
}

void MitsubishiHeatPump::dump_config() {
    this->banner();
    ESP_LOGI(TAG, "  Supports HEAT: %s", YESNO(true));
    ESP_LOGI(TAG, "  Supports COOL: %s", YESNO(true));
    ESP_LOGI(TAG, "  Supports AWAY mode: %s", YESNO(false));
    ESP_LOGI(TAG, "  Saved heat: %.1f", heat_setpoint.value_or(-1));
    ESP_LOGI(TAG, "  Saved cool: %.1f", cool_setpoint.value_or(-1));
    ESP_LOGI(TAG, "  Saved auto: %.1f", auto_setpoint.value_or(-1));
}

void MitsubishiHeatPump::dump_state() {
    LOG_CLIMATE("", "MitsubishiHeatPump Climate", this);
    ESP_LOGI(TAG, "HELLO");
}

void MitsubishiHeatPump::log_packet(byte* packet, unsigned int length, char* packetDirection) {
    String packetHex;
    char textBuf[15];

    for (int i = 0; i < length; i++) {
        memset(textBuf, 0, 15);
        sprintf(textBuf, "%02X ", packet[i]);
        packetHex += textBuf;
    }
    
    ESP_LOGV(TAG, "PKT: [%s] %s", packetDirection, packetHex.c_str());
}

// This function takes in the exact Celsius value and returns the rounded Celsius value.
// These values correspond to Exact Fahrenheit values converted to Celsius.
float MitsubishiHeatPump::roundCelsiusValues(float exactCelsius) {
    ESP_LOGI(TAG, "Input exactCelsius value: %.2f", exactCelsius);
  
   
    // Mapping of exact Celsius values to their corresponding rounded values
    const std::vector<std::pair<float, float>> lookupTable = {
        {16.11, 16.0}, {16.67, 16.5}, {17.22, 17.0}, {17.78, 17.5}, {18.33, 18.0},
        {18.89, 18.5}, {19.44, 19.0}, {20.00, 20.0}, {20.56, 21.0}, {21.11, 21.5},
        {21.67, 22.0}, {22.22, 22.5}, {22.78, 23.0}, {23.33, 23.5}, {23.89, 24.0},
        {24.44, 24.5}, {25.00, 25.0}, {25.56, 25.5}, {26.11, 26.0}, {26.67, 26.5},
        {27.22, 27.0}, {27.78, 27.5}, {28.33, 28.0}, {28.89, 28.5}, {29.44, 29.0},
        {30.00, 29.5}, {30.56, 30.0}, {31.11, 30.5}
    };

    // Find the closest match in the lookup table
    for (const auto& entry : lookupTable) {
        if (fabs(exactCelsius - entry.first) < 0.5) {
            ESP_LOGI(TAG, "Output roundedCelsius value: %.2f", entry.second);
            return entry.second;
        }
    }

    // If no close match is found, round to the nearest 0.5
    return round(exactCelsius * 2.0) / 2.0;
}

float MitsubishiHeatPump::exactCelsiusValues(float roundedCelsius) {
    // Mapping of rounded Celsius values to their corresponding exact values
    
    const std::map<float, float> reverseLookupTable = {
        {16.0, 16.11}, {16.5, 16.67}, {17.0, 17.22}, {17.5, 17.78}, {18.0, 18.33},
        {18.5, 18.89}, {19.0, 19.44}, {20.0, 20.00}, {21.0, 20.56}, {21.5, 21.11},
        {22.0, 21.67}, {22.5, 22.22}, {23.0, 22.78}, {23.5, 23.33}, {24.0, 23.89},
        {24.5, 24.44}, {25.0, 25.00}, {25.5, 25.56}, {26.0, 26.11}, {26.5, 26.67},
        {27.0, 27.22}, {27.5, 27.78}, {28.0, 28.33}, {28.5, 28.89}, {29.0, 29.44},
        {29.5, 30.00}, {30.0, 30.56}, {30.5, 31.11}
    };

    // Look for the rounded Celsius value in the reverse lookup table
    auto it = reverseLookupTable.find(roundedCelsius);
    if (it != reverseLookupTable.end()) {
        return it->second;  // Return the exact Celsius value if found
    }

    // If no exact match is found, return the input value
    return roundedCelsius;
}


float MitsubishiHeatPump::toCelsius(float fromFahrenheit) {
    const std::map<int, float> lookupTable = {
        {61, 16.0}, {62, 16.5}, {63, 17.0}, {64, 17.5}, {65, 18.0},
        {66, 18.5}, {67, 19.0}, {68, 20.0}, {69, 21.0}, {70, 21.5},
        {71, 22.0}, {72, 22.5}, {73, 23.0}, {74, 23.5}, {75, 24.0},
        {76, 24.5}, {77, 25.0}, {78, 25.5}, {79, 26.0}, {80, 26.5},
        {81, 27.0}, {82, 27.5}, {83, 28.0}, {84, 28.5}, {85, 29.0},
        {86, 29.5}, {87, 30.0}, {88, 30.5}
    };

    auto it = lookupTable.find(static_cast<int>(fromFahrenheit));
    if (it != lookupTable.end()) {
        return it->second;
    }

    float result = (fromFahrenheit - 32.0) / 1.8;
    return round(result * 2) / 2.0;
}
