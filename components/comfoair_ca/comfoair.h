#pragma once

#include "esphome/core/component.h"
#include "esphome/core/optional.h"
#include "esphome/core/preferences.h"
#include "esphome/core/preference_backend.h"

#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/climate/climate_mode.h"
#include "esphome/components/climate/climate_traits.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "logger.h"
#include "registers.h"

namespace esphome
{
  namespace comfoair_ca
  {

    static const char *TAG = "comfoair_ca";
    
    const uint32_t preference_developer_mode = 0x12345678;

    class ComfoAirComponent : public climate::Climate, public PollingComponent, public uart::UARTDevice
    {
    public:
      // Poll every 600ms
      ComfoAirComponent() : Climate(),
                            PollingComponent(600),
                            UARTDevice() {}

      void setup() override
      {
        this->pref_ = global_preferences->make_preference<bool>(preference_developer_mode);
        bool value;
        if (this->pref_.load(&value))
        {
          ::esphome::comfoair_ca::logEnable = value;
        }

        // Defensive: if the unit was left in test mode by a previous crash/reset,
        // bring it back to normal self-regulation on boot.
        write_command_(CMD_EXIT_TEST_MODE, nullptr, 0);
        test_mode_active_ = false;
        test_mode_exit_at_ms_ = 0;
      }

      /// Return the traits of this controller.
      climate::ClimateTraits traits() override
      {
        auto traits = climate::ClimateTraits();

        // Supported modes
        traits.set_supported_modes({climate::CLIMATE_MODE_FAN_ONLY});

        // Supported presets
        traits.set_supported_presets({
            climate::CLIMATE_PRESET_HOME,
        });

        // Visual temperature limits
        traits.set_visual_min_temperature(12);
        traits.set_visual_max_temperature(29);
        traits.set_visual_temperature_step(1);

        // Supported fan modes
        traits.set_supported_fan_modes({
            climate::CLIMATE_FAN_AUTO,
            climate::CLIMATE_FAN_LOW,
            climate::CLIMATE_FAN_MEDIUM,
            climate::CLIMATE_FAN_HIGH,
            climate::CLIMATE_FAN_OFF,
        });

        return traits;
      }

      /// Override control to change settings of the climate device.
      void control(const climate::ClimateCall &call) override
      {
        if (call.get_fan_mode().has_value())
        {
          int level;

          fan_mode = *call.get_fan_mode();
          switch (fan_mode.value())
          {
          case climate::CLIMATE_FAN_HIGH:
            level = 0x04;
            break;
          case climate::CLIMATE_FAN_MEDIUM:
            level = 0x03;
            break;
          case climate::CLIMATE_FAN_LOW:
            level = 0x02;
            break;
          case climate::CLIMATE_FAN_OFF:
            level = 0x01;
            break;
          case climate::CLIMATE_FAN_AUTO:
            level = 0x00;
            break;
          case climate::CLIMATE_FAN_ON:
          case climate::CLIMATE_FAN_MIDDLE:
          case climate::CLIMATE_FAN_DIFFUSE:
          default:
            level = -1;
            break;
          }

          if (level >= 0)
          {
            set_level_(level);
          }
        }
        if (call.get_target_temperature().has_value())
        {
          target_temperature = *call.get_target_temperature();
          set_comfort_temperature_(target_temperature);
        }

        publish_state();
      }

      void dump_config() override
      {
        uint8_t *p;
        LOG_CONFIG(TAG, "ComfoAir:");
        // LOG_UPDATE_INTERVAL(this);
        p = bootloader_version_;
        LOG_CONFIG(TAG, "  Bootloader %.10s v%0d.%02d b%2d", p + 3, *p, *(p + 1), *(p + 2));
        p = firmware_version_;
        LOG_CONFIG(TAG, "  Firmware %.10s v%0d.%02d b%2d", p + 3, *p, *(p + 1), *(p + 2));
        p = connector_board_version_;
        LOG_CONFIG(TAG, "  Connector Board %.10s v%0d.%02d", p + 2, *p, *(p + 1));

        if (*(p + 12) != 0)
        {
          LOG_CONFIG(TAG, "  CC-Ease v%0d.%02d", *(p + 12) >> 4, *(p + 12) & 0x0f);
        }
        if (*(p + 13) != 0)
        {
          LOG_CONFIG(TAG, "  CC-Luxe v%0d.%02d", *(p + 13) >> 4, *(p + 13) & 0x0f);
        }
        check_uart_settings(9600);
      }

      void update() override
      {
        // While a test-mode override is running, the state machine owns the bus.
        // Suppress regular polling so poll frames don't collide with test frames.
        if (test_state_ != TestState::IDLE)
          return;

        switch (update_counter_)
        {
        case -4:
          write_command_(CMD_GET_BOOTLOADER_VERSION, nullptr, 0);
          break;
        case -3:
          write_command_(CMD_GET_FIRMWARE_VERSION, nullptr, 0);
          break;
        case -2:
          write_command_(CMD_GET_CONNECTOR_BOARD_VERSION, nullptr, 0);
          break;
        case -1:
          write_command_(CMD_GET_STATUS, nullptr, 0);
          break;
        case 0:
          get_fan_status_();
          break;
        case 1:
          get_valve_status_();
          break;
        case 2:
          get_sensor_data_();
          break;
        case 3:
          get_ventilation_level_();
          break;
        case 4:
          get_temperatures_();
          break;
        case 5:
          get_error_status_();
          break;
        case 6:
          get_bypass_control_status_();
          break;
        case 7:
          get_operation_hours_();
          break;
        case 8:
          get_preheating_status_();
          break;
        case 9:
          get_time_delay_();
          break;
        case 10:
          get_ewt_postheating_();
          break;
        case 11:
          get_analog_values_();
          break;
        }

        update_counter_++;
        if (update_counter_ > num_update_counter_elements_)
          update_counter_ = 0;
      }

      void loop() override
      {
        // Drive the test-mode state machine (enter -> drive -> hold -> exit),
        // sending at most one frame per tick with spacing so frames don't collide.
        test_mode_pump_();

        while (available() != 0)
        {
          read_byte(&data_[data_index_]);
          auto check = check_byte_();
          if (!check.has_value())
          {

            // finished
            if (data_[COMMAND_ID_ACK] != COMMAND_ACK)
            {
              parse_data_();
            }
            data_index_ = 0;
          }
          else if (!*check)
          {
            // wrong data
            LOG_VERBOSE(TAG, "Byte %i of received data frame is invalid.", data_index_);
            data_index_ = 0;
          }
          else
          {
            // next byte
            data_index_++;
          }
        }
      }

      float get_setup_priority() const override { return setup_priority::DATA; }

      void error_reset(void)
      {
        uint8_t reset_cmd[4] = {1, 0, 0, 0};
        write_command_(CMD_RESET_AND_SELF_TEST, reset_cmd, sizeof(reset_cmd));
      }

      void filter_reset(void)
      {
        uint8_t reset_cmd[4] = {0, 0, 0, 1};
        write_command_(CMD_RESET_AND_SELF_TEST, reset_cmd, sizeof(reset_cmd));
      }

      void set_name(const char *value) { name = value; }
      void set_name(const char *value, uint32_t name_hash)
      {
        (void)name_hash;
        name = value;
      }
      void set_uart_component(uart::UARTComponent *parent) { set_uart_parent(parent); }

      void set_developer_mode(bool enable)
      {
        ::esphome::comfoair_ca::logEnable = enable;
        this->pref_.save(&enable);
      }

      bool get_developer_mode() const
      {
        return ::esphome::comfoair_ca::logEnable;
      }

    protected:
      ESPPreferenceObject pref_;
      std::string filter_ascii_(const char *input)
      {
        std::string out;

        for (size_t i = 0; input[i] != '\0'; i++)
        {
          char c = input[i];

          if (c >= 32 && c <= 126) // printable ASCII
            out += c;
        }

        return out;
      }

      void set_level_(int level)
      {
        if (level < 0 || level > 5)
        {
          LOG_INFO(TAG, "Ignoring invalid level request: %i", level);
          return;
        }

        LOG_INFO(TAG, "Setting level to: %i", level);
        {
          uint8_t command[1] = {(uint8_t)level};
          write_command_(CMD_SET_LEVEL, command, sizeof(command));
        }
      }

      void set_comfort_temperature_(float temperature)
      {
        if (temperature < 12.0f || temperature > 29.0f)
        {
          LOG_INFO(TAG, "Ignoring invalid temperature request: %i", (int)temperature);
          return;
        }

        LOG_INFO(TAG, "Setting temperature to: %i", (int)temperature);
        {
          uint8_t command[1] = {(uint8_t)((temperature + 20.0f) * 2.0f)};
          write_command_(CMD_SET_COMFORT_TEMPERATURE, command, sizeof(command));
        }
      }

      void send_ventilation_levels_()
      {
        uint8_t command_data[9] = {
            exhaust_absent_,
            exhaust_low_,
            exhaust_medium_,
            supply_absent_,
            supply_low_,
            supply_medium_,
            exhaust_high_,
            supply_high_,
            unused_byte_};
        // Debug write to check if the values are correct
        LOG_DEBUG(TAG, "-----------------------------------");
        LOG_DEBUG(TAG, "SET VENTILATION LEVEL %%:");
        LOG_DEBUG(TAG, "- Supply Absent: %02x\t(%d%%)", supply_absent_, supply_absent_);
        LOG_DEBUG(TAG, "- Exhaust Absent: %02x\t(%d%%)", exhaust_absent_, exhaust_absent_);
        LOG_DEBUG(TAG, "- Supply Low: %02x\t(%d%%)", supply_low_, supply_low_);
        LOG_DEBUG(TAG, "- Exhaust Low: %02x\t(%d%%)", exhaust_low_, exhaust_low_);
        LOG_DEBUG(TAG, "- Supply Medium: %02x\t(%d%%)", supply_medium_, supply_medium_);
        LOG_DEBUG(TAG, "- Exhaust Medium: %02x\t(%d%%)", exhaust_medium_, exhaust_medium_);
        LOG_DEBUG(TAG, "- Supply High: %02x\t(%d%%)", supply_high_, supply_high_);
        LOG_DEBUG(TAG, "- Exhaust High: %02x\t(%d%%)", exhaust_high_, exhaust_high_);
        // LOG_DEBUG(TAG, "- Unused Byte: %02x", unused_byte_);
        LOG_DEBUG(TAG, "-----------------------------------");

        if (can_send_ventilation_levels_)
        {
          write_command_(CMD_SET_VENTILATION_LEVEL, command_data, sizeof(command_data));
        }
        else
        {
          LOG_WARN(TAG, "Can't send ventilation levels yet");
        }
      }

      void write_command_(const uint8_t command, const uint8_t *command_data, uint8_t command_data_length)
      {
        write_byte(COMMAND_PREFIX);
        write_byte(COMMAND_HEAD);
        write_byte(0x00);
        write_byte(command);
        write_byte(command_data_length);
        if (command_data_length > 0)
        {
          write_array(command_data, command_data_length);
          write_byte((command + command_data_length + comfoair_checksum_(command_data, command_data_length)) & 0xff);
        }
        else
        {
          write_byte(comfoair_checksum_(&command, 1));
        }
        write_byte(COMMAND_PREFIX);
        write_byte(COMMAND_TAIL);
        flush();
      }

      uint8_t comfoair_checksum_(const uint8_t *command_data, uint8_t length) const
      {
        uint8_t sum = 173;
        bool skip = false;
        for (uint8_t i = 0; i < length; i++)
        {
          if (command_data[i] == 0x07)
          {
            if (skip)
              continue;
            else
              skip = true;
          }
          sum += command_data[i];
        }
        return sum % 256;
      }

      optional<bool> check_byte_() const
      {
        uint8_t index = data_index_;
        uint8_t byte = data_[index];

        if (index == 0)
        {
          return byte == COMMAND_PREFIX;
        }

        if (index == 1)
        {
          if (byte == COMMAND_ACK)
          {
            return {};
          }
          else
          {
            return byte == COMMAND_HEAD;
          }
        }

        if (index == 2)
        {
          return byte == 0x00;
        }

        if (index < COMMAND_LEN_HEAD)
        {
          return true;
        }

        uint8_t data_length = data_[COMMAND_IDX_DATA];

        if ((COMMAND_LEN_HEAD + data_length + COMMAND_LEN_TAIL) > sizeof(data_))
        {
          LOG_WARN(TAG, "ComfoAir message too large");
          return false;
        }

        if (index < COMMAND_LEN_HEAD + data_length)
        {
          return true;
        }

        if (index == COMMAND_LEN_HEAD + data_length)
        {
          // checksum is without checksum bytes
          uint8_t checksum = comfoair_checksum_(data_ + 2, COMMAND_LEN_HEAD + data_length - 2);
          if (checksum != byte)
          {
            // LOG_WARN(TAG, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", data_[0], data_[1], data_[2], data_[3], data_[4], data_[5], data_[6], data_[7], data_[8], data_[9], data_[10]);
            LOG_WARN(TAG, "ComfoAir Checksum doesn't match: 0x%02X!=0x%02X", byte, checksum);
            return false;
          }
          return true;
        }

        if (index == COMMAND_LEN_HEAD + data_length + 1)
        {
          return byte == COMMAND_PREFIX;
        }

        if (index == COMMAND_LEN_HEAD + data_length + 2)
        {
          if (byte != COMMAND_TAIL)
          {
            return false;
          }
        }

        return {};
      }

      void parse_data_()
      {
        status_clear_warning();
        uint8_t *msg = &data_[COMMAND_LEN_HEAD];
        char buf[64];

        switch (data_[COMMAND_IDX_MSG_ID])
        {
        case RES_GET_BOOTLOADER_VERSION:
          memcpy(bootloader_version_, msg, data_[COMMAND_IDX_DATA]);
          snprintf(buf, sizeof(buf), "%.10s v%0d.%02d b%2d", bootloader_version_ + 3, *bootloader_version_, *(bootloader_version_ + 1), *(bootloader_version_ + 2));
          bootloader_info->publish_state(filter_ascii_(buf).c_str());
          break;

        case RES_GET_FIRMWARE_VERSION:
          memcpy(firmware_version_, msg, data_[COMMAND_IDX_DATA]);
          snprintf(buf, sizeof(buf), "%.10s v%0d.%02d b%2d", firmware_version_ + 3, *firmware_version_, *(firmware_version_ + 1), *(firmware_version_ + 2));
          firmware_info->publish_state(filter_ascii_(buf).c_str());
          break;

        case RES_GET_CONNECTOR_BOARD_VERSION:
          memcpy(connector_board_version_, msg, data_[COMMAND_IDX_DATA]);
          snprintf(buf, sizeof(buf), "%.10s v%0d.%02d", connector_board_version_ + 2, *connector_board_version_, *(connector_board_version_ + 1));
          connector_board_info->publish_state(filter_ascii_(buf).c_str());
          break;

        case RES_GET_FAN_STATUS:
        {
          if (intake_fan_speed != nullptr)
          {
            intake_fan_speed->publish_state(msg[0]);
          }
          if (exhaust_fan_speed != nullptr)
          {
            exhaust_fan_speed->publish_state(msg[1]);
          }
          if (intake_fan_speed_rpm != nullptr)
          {
            intake_fan_speed_rpm->publish_state(static_cast<int>(1875000.0f / get_uint16_(2)));
          }
          if (exhaust_fan_speed_rpm != nullptr)
          {
            exhaust_fan_speed_rpm->publish_state(static_cast<int>(1875000.0f / get_uint16_(4)));
          }
          break;
        }
        case RES_GET_VALVE_STATUS:
        {
          if (bypass_valve != nullptr)
          {
            bypass_valve->publish_state(msg[0]);
          }
          if (bypass_valve_open != nullptr)
          {
            bypass_valve_open->publish_state(msg[0] != 0);
          }
          if (preheating_state != nullptr)
          {
            preheating_state->publish_state(msg[1] != 0);
          }
          if (motor_current_bypass != nullptr)
          {
            motor_current_bypass->publish_state(msg[2]);
          }
          if (motor_current_preheating != nullptr)
          {
            motor_current_preheating->publish_state(msg[3]);
          }
          break;
        }
        case RES_GET_BYPASS_CONTROL_STATUS:
        {
          if (bypass_factor != nullptr)
          {
            bypass_factor->publish_state(msg[2]);
          }
          if (bypass_step != nullptr)
          {
            bypass_step->publish_state(msg[3]);
          }
          if (bypass_correction != nullptr)
          {
            bypass_correction->publish_state(msg[4]);
          }
          if (summer_mode != nullptr)
          {
            summer_mode->publish_state(msg[6] != 0);
          }
          break;
        }
        case RES_GET_TEMPERATURE_STATUS:
        {

          // T1 / outside air
          if (outside_air_temperature != nullptr)
          {
            outside_air_temperature->publish_state((float)msg[0] / 2.0f - 20.0f);
          }
          // T2 / supply air
          if (supply_air_temperature != nullptr)
          {
            supply_air_temperature->publish_state((float)msg[1] / 2.0f - 20.0f);
          }
          // T3 / return air
          if (return_air_temperature != nullptr)
          {
            return_air_temperature->publish_state((float)msg[2] / 2.0f - 20.0f);
          }
          // T4 / exhaust air
          if (exhaust_air_temperature != nullptr)
          {
            exhaust_air_temperature->publish_state((float)msg[3] / 2.0f - 20.0f);
          }
          break;
        }
        case RES_GET_SENSOR_DATA:
        {

          if (enthalpy_temperature != nullptr)
          {
            enthalpy_temperature->publish_state((float)msg[0] / 2.0f - 20.0f);
          }

          break;
        }
        case RES_GET_VENTILATION_LEVEL:
        {

          LOG_DEBUG(TAG, "Level %02x", msg[8]);

          exhaust_absent_ = msg[0];
          exhaust_low_ = msg[1];
          exhaust_medium_ = msg[2];
          supply_absent_ = msg[3];
          supply_low_ = msg[4];
          supply_medium_ = msg[5];
          exhaust_high_ = msg[10];
          supply_high_ = msg[11];

          if (exhaust_absent_level != nullptr)
            exhaust_absent_level->publish_state(exhaust_absent_);
          if (exhaust_low_level != nullptr)
            exhaust_low_level->publish_state(exhaust_low_);
          if (exhaust_medium_level != nullptr)
            exhaust_medium_level->publish_state(exhaust_medium_);
          if (exhaust_high_level != nullptr)
            exhaust_high_level->publish_state(exhaust_high_);
          if (supply_absent_level != nullptr)
            supply_absent_level->publish_state(supply_absent_);
          if (supply_low_level != nullptr)
            supply_low_level->publish_state(supply_low_);
          if (supply_medium_level != nullptr)
            supply_medium_level->publish_state(supply_medium_);
          if (supply_high_level != nullptr)
            supply_high_level->publish_state(supply_high_);

          can_send_ventilation_levels_ = true;

          LOG_DEBUG(TAG, "-----------------------------------");
          LOG_DEBUG(TAG, "READ VENTILATION LEVEL %%:");
          LOG_DEBUG(TAG, "- Supply Absent: %02x\t(%d%%)", supply_absent_, supply_absent_);
          LOG_DEBUG(TAG, "- Exhaust Absent: %02x\t(%d%%)", exhaust_absent_, exhaust_absent_);
          LOG_DEBUG(TAG, "- Supply Low: %02x\t(%d%%)", supply_low_, supply_low_);
          LOG_DEBUG(TAG, "- Exhaust Low: %02x\t(%d%%)", exhaust_low_, exhaust_low_);
          LOG_DEBUG(TAG, "- Supply Medium: %02x\t(%d%%)", supply_medium_, supply_medium_);
          LOG_DEBUG(TAG, "- Exhaust Medium: %02x\t(%d%%)", exhaust_medium_, exhaust_medium_);
          LOG_DEBUG(TAG, "- Supply High: %02x\t(%d%%)", supply_high_, supply_high_);
          LOG_DEBUG(TAG, "- Exhaust High: %02x\t(%d%%)", exhaust_high_, exhaust_high_);
          // LOG_DEBUG(TAG, "- Unused Byte: %02x", unused_byte_);
          LOG_DEBUG(TAG, "-----------------------------------");

          if (return_air_level != nullptr)
          {
            return_air_level->publish_state(msg[6]);
          }
          if (supply_air_level != nullptr)
          {
            supply_air_level->publish_state(msg[7]);
          }

          if (ventilation_level != nullptr)
          {
            ventilation_level->publish_state(msg[8] - 1);
          }

          // Fan Speed
          switch (msg[8])
          {
          case 0x00:
            fan_mode = climate::CLIMATE_FAN_AUTO;
            mode = climate::CLIMATE_MODE_FAN_ONLY;
            break;
          case 0x01:
            fan_mode = climate::CLIMATE_FAN_OFF;
            mode = climate::CLIMATE_MODE_FAN_ONLY;
            break;
          case 0x02:
            fan_mode = climate::CLIMATE_FAN_LOW;
            mode = climate::CLIMATE_MODE_FAN_ONLY;
            break;
          case 0x03:
            fan_mode = climate::CLIMATE_FAN_MEDIUM;
            mode = climate::CLIMATE_MODE_FAN_ONLY;
            break;
          case 0x04:
            fan_mode = climate::CLIMATE_FAN_HIGH;
            mode = climate::CLIMATE_MODE_FAN_ONLY;
            break;
          }

          publish_state();

          // Supply air fan active (1 = active / 0 = inactive)
          if (supply_fan_active != nullptr)
          {
            supply_fan_active->publish_state(msg[9] == 1);
          }
          break;
        }
        case RES_GET_FAULTS:
        {
          if (filter_status != nullptr)
          {
            uint8_t status = msg[8];
            filter_status->publish_state(status == 0 ? "Ok" : (status == 1 ? "Full" : "Unknown"));
          }
          break;
        }
        case RES_GET_TEMPERATURES:
        {

          // comfort temperature
          target_temperature = (float)msg[0] / 2.0f - 20.0f;
          publish_state();

          // T1 / outside air
          if (outside_air_temperature != nullptr && msg[5] & 0x01)
          {
            outside_air_temperature->publish_state((float)msg[1] / 2.0f - 20.0f);
          }
          // T2 / supply air
          if (supply_air_temperature != nullptr && msg[5] & 0x02)
          {
            supply_air_temperature->publish_state((float)msg[2] / 2.0f - 20.0f);
          }
          // T3 / exhaust air
          if (return_air_temperature != nullptr && msg[5] & 0x04)
          {
            return_air_temperature->publish_state((float)msg[3] / 2.0f - 20.0f);
            current_temperature = (float)msg[3] / 2.0f - 20.0f;
          }
          // T4 / continued air
          if (exhaust_air_temperature != nullptr && msg[5] & 0x08)
          {
            exhaust_air_temperature->publish_state((float)msg[4] / 2.0f - 20.0f);
          }
          // EWT
          if (ewt_temperature != nullptr && msg[5] & 0x10)
          {
            ewt_temperature->publish_state((float)msg[6] / 2.0f - 20.0f);
          }
          // reheating
          if (reheating_temperature != nullptr && msg[5] & 0x20)
          {
            reheating_temperature->publish_state((float)msg[7] / 2.0f - 20.0f);
          }
          // kitchen hood
          if (kitchen_hood_temperature != nullptr && msg[5] & 0x40)
          {
            kitchen_hood_temperature->publish_state((float)msg[8] / 2.0f - 20.0f);
          }

          break;
        }
        case RES_GET_STATUS:
        {
          // Cache the config bytes for read-modify-write via CMD_SET_STATUS (0xD7).
          // The 0xD6 reply layout: [0] preheater present, [1] bypass present,
          // [2] type, [3] size, [4] options bitmask, ... [9] enthalpy, [10] EWT.
          status_preheater_present_ = msg[0];
          status_bypass_present_ = msg[1];
          status_type_ = msg[2];
          status_size_ = msg[3];
          status_options_ = msg[4];
          status_enthalpy_present_ = msg[9];
          status_ewt_present_ = msg[10];
          can_send_status_ = true;

          if (preheating_present != nullptr)
          {
            preheating_present->publish_state(msg[0]);
          }

          if (bypass_present != nullptr)
          {
            bypass_present->publish_state(msg[1]);
          }

          if (type != nullptr)
          {
            type->publish_state(msg[2] == 1 ? "Left" : (msg[2] == 2 ? "Right" : "Unknown"));
          }

          if (size != nullptr)
          {
            size->publish_state(msg[3] == 1 ? "Large" : (msg[3] == 2 ? "Small" : "Unknown"));
          }

          if (options_present != nullptr)
          {
            options_present->publish_state(msg[4]);
          }

          if (fireplace_present != nullptr)
          {
            fireplace_present->publish_state(msg[4] & 0x01);
          }

          if (kitchen_hood_present != nullptr)
          {
            kitchen_hood_present->publish_state(msg[4] & 0x02);
          }

          if (postheating_present != nullptr)
          {
            postheating_present->publish_state(msg[4] & 0x04);
          }

          if (postheating_pwm_mode_present != nullptr)
          {
            postheating_pwm_mode_present->publish_state(msg[4] & 0x40);
          }

          if (p10_active != nullptr)
          {
            p10_active->publish_state(msg[6] & 0x01);
          }

          if (p11_active != nullptr)
          {
            p11_active->publish_state(msg[6] & 0x02);
          }

          if (p12_active != nullptr)
          {
            p12_active->publish_state(msg[6] & 0x04);
          }

          if (p13_active != nullptr)
          {
            p13_active->publish_state(msg[6] & 0x08);
          }

          if (p14_active != nullptr)
          {
            p14_active->publish_state(msg[6] & 0x10);
          }

          if (p15_active != nullptr)
          {
            p15_active->publish_state(msg[6] & 0x20);
          }

          if (p16_active != nullptr)
          {
            p16_active->publish_state(msg[6] & 0x40);
          }

          if (p17_active != nullptr)
          {
            p17_active->publish_state(msg[6] & 0x80);
          }

          if (p18_active != nullptr)
          {
            p18_active->publish_state(msg[7] & 0x01);
          }

          if (p19_active != nullptr)
          {
            p19_active->publish_state(msg[7] & 0x02);
          }

          if (p90_active != nullptr)
          {
            p90_active->publish_state(msg[8] & 0x01);
          }

          if (p91_active != nullptr)
          {
            p91_active->publish_state(msg[8] & 0x02);
          }

          if (p92_active != nullptr)
          {
            p92_active->publish_state(msg[8] & 0x04);
          }

          if (p93_active != nullptr)
          {
            p93_active->publish_state(msg[8] & 0x08);
          }

          if (p94_active != nullptr)
          {
            p94_active->publish_state(msg[8] & 0x10);
          }

          if (p95_active != nullptr)
          {
            p95_active->publish_state(msg[8] & 0x20);
          }

          if (p96_active != nullptr)
          {
            p96_active->publish_state(msg[8] & 0x40);
          }

          if (p97_active != nullptr)
          {
            p97_active->publish_state(msg[8] & 0x80);
          }

          if (enthalpy_present != nullptr)
          {
            enthalpy_present->publish_state(msg[9]);
          }

          if (ewt_present != nullptr)
          {
            ewt_present->publish_state(msg[10]);
          }
          break;
        }
        case RES_GET_OPERATION_HOURS:
        {
          if (level0_hours != nullptr)
          {
            level0_hours->publish_state((msg[0] << 16) | (msg[1] << 8) | msg[2]);
          }

          if (level1_hours != nullptr)
          {
            level1_hours->publish_state((msg[3] << 16) | (msg[4] << 8) | msg[5]);
          }

          if (level2_hours != nullptr)
          {
            level2_hours->publish_state((msg[6] << 16) | (msg[7] << 8) | msg[8]);
          }

          if (level3_hours != nullptr)
          {
            level3_hours->publish_state((msg[17] << 16) | (msg[18] << 8) | msg[19]);
          }

          if (frost_protection_hours != nullptr)
          {
            frost_protection_hours->publish_state((msg[9] << 8) | msg[10]);
          }

          if (bypass_open_hours != nullptr)
          {
            bypass_open_hours->publish_state((msg[13] << 8) | msg[14]);
          }

          if (preheating_hours != nullptr)
          {
            preheating_hours->publish_state((msg[11] << 8) | msg[12]);
          }

          if (filter_hours != nullptr)
          {
            filter_hours->publish_state((msg[15] << 8) | msg[16]);
          }
          break;
        }

        case RES_GET_PREHEATING_STATUS:
        {
          if (preheating_valve != nullptr)
          {
            std::string name_preheating_valve;
            switch (msg[0])
            {
            case 0:
              name_preheating_valve = "Closed";
              break;

            case 1:
              name_preheating_valve = "Open";
              break;

            default:
              name_preheating_valve = "Unknown";
              break;
            }
            preheating_valve->publish_state(name_preheating_valve);
          }

          if (frost_protection_active != nullptr)
          {
            frost_protection_active->publish_state(msg[1] != 0);
          }

          if (preheating_state != nullptr)
          {
            preheating_state->publish_state(msg[2] != 0);
          }

          if (frost_protection_minutes != nullptr)
          {
            frost_protection_minutes->publish_state((msg[3] << 8) | msg[4]);
          }

          if (frost_protection_level != nullptr)
          {
            std::string name_frost_protection_level;
            switch (msg[5])
            {
            case 0:
              name_frost_protection_level = "GuaranteedProtection";
              break;

            case 1:
              name_frost_protection_level = "HighProtection";
              break;

            case 2:
              name_frost_protection_level = "NominalProtection";
              break;

            case 3:
              name_frost_protection_level = "Economy";
              break;

            default:
              name_frost_protection_level = "Unknown";
              break;
            }
            frost_protection_level->publish_state(name_frost_protection_level);
          }
          break;
        }
        case RES_GET_TIME_DELAY:
        {
          // Cache the full 8-byte block so writes can read-modify-write.
          td_bathroom_on_ = msg[0];
          td_bathroom_off_ = msg[1];
          td_l1_off_ = msg[2];
          td_boost_ = msg[3];
          td_filter_weeks_ = msg[4];
          td_rf_short_ = msg[5];
          td_rf_long_ = msg[6];
          td_cookerhood_off_ = msg[7];
          can_send_time_delay_ = true;

          if (bathroom_switch_on_delay_minutes != nullptr)
          {
            bathroom_switch_on_delay_minutes->publish_state(msg[0]);
          }

          if (bathroom_switch_off_delay_minutes != nullptr)
          {
            bathroom_switch_off_delay_minutes->publish_state(msg[1]);
          }

          if (l1_switch_off_delay_minutes != nullptr)
          {
            l1_switch_off_delay_minutes->publish_state(msg[2]);
          }

          if (boost_ventilation_minutes != nullptr)
          {
            boost_ventilation_minutes->publish_state(msg[3]);
          }

          if (filter_warning_weeks != nullptr)
          {
            filter_warning_weeks->publish_state(msg[4]);
          }

          if (rf_high_time_short_minutes != nullptr)
          {
            rf_high_time_short_minutes->publish_state(msg[5]);
          }

          if (rf_high_time_long_minutes != nullptr)
          {
            rf_high_time_long_minutes->publish_state(msg[6]);
          }

          if (extractor_hood_switch_off_delay_minutes != nullptr)
          {
            extractor_hood_switch_off_delay_minutes->publish_state(msg[7]);
          }

          break;
        }
        case RES_EWT_POST_HEATING:
        {
          // Byte[0] EWT low (°C), Byte[1] EWT high (°C), Byte[2] EWT speed up (%),
          // Byte[3] Kitchen hood speed up (%), Byte[4] Post-heating performance,
          // Byte[5] Post-heating I parameter, Byte[6] Reheating T desired (°C).
          // Temperatures are encoded (°C + 20) * 2.
          ewt_low_ = msg[0];
          ewt_high_ = msg[1];
          ewt_speed_up_ = msg[2];
          cookerhood_speed_up_ = msg[3];
          postheating_performance_ = msg[4];
          postheating_i_parameter_ = msg[5];
          reheating_target_ = msg[6];
          can_send_ewt_postheating_ = true;

          if (ewt_low_temperature != nullptr)
            ewt_low_temperature->publish_state((float)msg[0] / 2.0f - 20.0f);
          if (ewt_high_temperature != nullptr)
            ewt_high_temperature->publish_state((float)msg[1] / 2.0f - 20.0f);
          if (ewt_speed_up_level != nullptr)
            ewt_speed_up_level->publish_state(msg[2]);
          if (cookerhood_speed_up_level != nullptr)
            cookerhood_speed_up_level->publish_state(msg[3]);
          if (reheating_target_temperature != nullptr)
            reheating_target_temperature->publish_state((float)msg[6] / 2.0f - 20.0f);
          break;
        }
        case RES_GET_ANALOG_VALUES:
        {
          // Full 19-byte block. Cache so writes can read-modify-write.
          for (uint8_t i = 0; i < sizeof(analog_values_); i++)
          {
            analog_values_[i] = msg[i];
          }
          can_send_analog_values_ = true;
          break;
        }
        }
      }

      void get_fan_status_()
      {
        if (intake_fan_speed != nullptr ||
            exhaust_fan_speed != nullptr ||
            intake_fan_speed_rpm != nullptr ||
            exhaust_fan_speed_rpm != nullptr)
        {
          LOG_DEBUG(TAG, "getting fan status");
          write_command_(CMD_GET_FAN_STATUS, nullptr, 0);
        }
      }

      void get_valve_status_()
      {
        if (bypass_valve != nullptr ||
            bypass_valve_open != nullptr ||
            preheating_state != nullptr)
        {
          LOG_DEBUG(TAG, "getting valve status");
          write_command_(CMD_GET_VALVE_STATUS, nullptr, 0);
        }
      }

      void get_error_status_()
      {
        if (filter_status != nullptr)
        {
          LOG_DEBUG(TAG, "getting error status");
          write_command_(CMD_GET_FAULTS, nullptr, 0);
        }
      }

      void get_bypass_control_status_()
      {
        if (bypass_factor != nullptr ||
            bypass_step != nullptr ||
            bypass_correction != nullptr ||
            summer_mode != nullptr)
        {
          LOG_DEBUG(TAG, "getting bypass control");
          write_command_(CMD_GET_BYPASS_CONTROL_STATUS, nullptr, 0);
        }
      }

      void get_temperature_()
      {
        if (outside_air_temperature != nullptr ||
            supply_air_temperature != nullptr ||
            return_air_temperature != nullptr ||
            outside_air_temperature != nullptr)
        {
          LOG_DEBUG(TAG, "getting temperature");
          write_command_(CMD_GET_TEMPERATURE_STATUS, nullptr, 0);
        }
      }

      void get_sensor_data_()
      {
        if (enthalpy_temperature != nullptr)
        {
          LOG_DEBUG(TAG, "getting sensor data");
          write_command_(CMD_GET_SENSOR_DATA, nullptr, 0);
        }
      }

      void get_ventilation_level_()
      {
        LOG_DEBUG(TAG, "getting ventilation level");
        write_command_(CMD_GET_VENTILATION_LEVEL, nullptr, 0);
      }

      void get_temperatures_()
      {
        LOG_DEBUG(TAG, "getting temperatures");
        write_command_(CMD_GET_TEMPERATURES, nullptr, 0);
      }

      void get_operation_hours_()
      {
        LOG_DEBUG(TAG, "getting operation hours");
        write_command_(CMD_GET_OPERATION_HOURS, nullptr, 0);
      }

      void get_preheating_status_()
      {
        LOG_DEBUG(TAG, "getting preheating status");
        write_command_(CMD_GET_PREHEATING_STATUS, nullptr, 0);
      }

      void get_time_delay_()
      {
        LOG_DEBUG(TAG, "getting time delay");
        write_command_(CMD_GET_TIME_DELAY, nullptr, 0);
      }

      void get_ewt_postheating_()
      {
        LOG_DEBUG(TAG, "getting ewt / postheating");
        write_command_(CMD_EWT_POST_HEATING, nullptr, 0);
      }

      void get_analog_values_()
      {
        LOG_DEBUG(TAG, "getting analog values");
        write_command_(CMD_GET_ANALOG_VALUES, nullptr, 0);
      }

      uint8_t get_uint8_t_(uint8_t start_index) const
      {
        return data_[COMMAND_LEN_HEAD + start_index];
      }

      uint16_t get_uint16_(uint8_t start_index) const
      {
        return (uint16_t(data_[COMMAND_LEN_HEAD + start_index + 1] | data_[COMMAND_LEN_HEAD + start_index] << 8));
      }

      uint8_t data_[30];
      uint8_t data_index_{0};
      int8_t update_counter_{-4};
      const int8_t num_update_counter_elements_{11};

      uint8_t bootloader_version_[13]{0};
      uint8_t firmware_version_[13]{0};
      uint8_t connector_board_version_[14]{0};
      const char *name{0};

      uint8_t supply_absent_{0};
      uint8_t exhaust_absent_{0};
      uint8_t supply_low_{0};
      uint8_t exhaust_low_{0};
      uint8_t supply_medium_{0};
      uint8_t exhaust_medium_{0};
      uint8_t supply_high_{0};
      uint8_t exhaust_high_{0};
      uint8_t unused_byte_{0};
      bool can_send_ventilation_levels_{false};

      // Cached time-delay block (CMD_SET_TIME_DELAY 0xCB, 8 bytes)
      uint8_t td_bathroom_on_{0};
      uint8_t td_bathroom_off_{0};
      uint8_t td_l1_off_{0};
      uint8_t td_boost_{0};
      uint8_t td_filter_weeks_{0};
      uint8_t td_rf_short_{0};
      uint8_t td_rf_long_{0};
      uint8_t td_cookerhood_off_{0};
      bool can_send_time_delay_{false};

      // Cached EWT / post-heating block (CMD_SET_EWT_POSTHEATING 0xED, 5 bytes written)
      uint8_t ewt_low_{0};
      uint8_t ewt_high_{0};
      uint8_t ewt_speed_up_{0};
      uint8_t cookerhood_speed_up_{0};
      uint8_t reheating_target_{0};
      // Read-only extras from the 0xEC reply (not part of the 5-byte write)
      uint8_t postheating_performance_{0};
      uint8_t postheating_i_parameter_{0};
      bool can_send_ewt_postheating_{false};

      // Cached analog-values block (CMD_SET_ANALOG_VALUES 0x9F, 19 bytes)
      uint8_t analog_values_[19]{0};
      bool can_send_analog_values_{false};

      // Cached unit-config block (CMD_SET_STATUS 0xD7, 8 bytes written; read from 0xD6)
      uint8_t status_preheater_present_{0};
      uint8_t status_bypass_present_{0};
      uint8_t status_type_{0};
      uint8_t status_size_{0};
      uint8_t status_options_{0};
      uint8_t status_enthalpy_present_{0};
      uint8_t status_ewt_present_{0};
      bool can_send_status_{false};

      // Test-mode state machine for the preheater override (Tier 3)
      enum class TestState : uint8_t
      {
        IDLE,
        ENTER,   // send CMD_SET_TEST_MODE, then wait
        DRIVE,   // send CMD_SET_OUTPUTS / CMD_SET_FLAPS, then hold
        HOLD,    // keep driving until the deadline
        EXIT     // send CMD_EXIT_TEST_MODE, then back to IDLE
      };
      TestState test_state_{TestState::IDLE};
      bool test_mode_active_{false};
      uint32_t test_mode_exit_at_ms_{0};      // deadline to leave test mode
      uint32_t test_next_action_at_ms_{0};    // earliest time for the next frame
      uint32_t test_duration_ms_{0};          // how long to hold the override
      bool test_use_flap_{false};             // false = SET_OUTPUTS, true = SET_FLAPS
      uint8_t test_drive_byte_a_{0};
      uint8_t test_drive_byte_b_{0};
      static const uint32_t test_frame_gap_ms_{750};   // spacing between enter/drive/exit
      static const uint32_t test_reassert_ms_{2000};   // re-send drive while holding

    public:
      text_sensor::TextSensor *bootloader_info{nullptr};
      text_sensor::TextSensor *firmware_info{nullptr};
      text_sensor::TextSensor *connector_board_info{nullptr};
      text_sensor::TextSensor *type{nullptr};
      text_sensor::TextSensor *size{nullptr};
      text_sensor::TextSensor *filter_status{nullptr};
      text_sensor::TextSensor *frost_protection_level{nullptr};
      text_sensor::TextSensor *preheating_valve{nullptr};
      sensor::Sensor *intake_fan_speed{nullptr};
      sensor::Sensor *exhaust_fan_speed{nullptr};
      sensor::Sensor *intake_fan_speed_rpm{nullptr};
      sensor::Sensor *exhaust_fan_speed_rpm{nullptr};
      sensor::Sensor *ventilation_level{nullptr};
      sensor::Sensor *outside_air_temperature{nullptr};
      sensor::Sensor *supply_air_temperature{nullptr};
      sensor::Sensor *return_air_temperature{nullptr};
      sensor::Sensor *exhaust_air_temperature{nullptr};
      sensor::Sensor *enthalpy_temperature{nullptr};
      sensor::Sensor *ewt_temperature{nullptr};
      sensor::Sensor *reheating_temperature{nullptr};
      sensor::Sensor *kitchen_hood_temperature{nullptr};
      sensor::Sensor *return_air_level{nullptr};
      sensor::Sensor *supply_air_level{nullptr};
      sensor::Sensor *bypass_factor{nullptr};
      sensor::Sensor *bypass_step{nullptr};
      sensor::Sensor *bypass_correction{nullptr};
      sensor::Sensor *bypass_open_hours{nullptr};
      sensor::Sensor *motor_current_bypass{nullptr};
      sensor::Sensor *motor_current_preheating{nullptr};
      sensor::Sensor *preheating_hours{nullptr};
      sensor::Sensor *level0_hours{nullptr};
      sensor::Sensor *level1_hours{nullptr};
      sensor::Sensor *level2_hours{nullptr};
      sensor::Sensor *level3_hours{nullptr};
      binary_sensor::BinarySensor *frost_protection_active{nullptr};
      sensor::Sensor *frost_protection_hours{nullptr};
      sensor::Sensor *frost_protection_minutes{nullptr};
      sensor::Sensor *filter_hours{nullptr};
      sensor::Sensor *bypass_valve{nullptr};
      binary_sensor::BinarySensor *bypass_present{nullptr};
      binary_sensor::BinarySensor *enthalpy_present{nullptr};
      binary_sensor::BinarySensor *ewt_present{nullptr};
      binary_sensor::BinarySensor *preheating_present{nullptr};
      binary_sensor::BinarySensor *options_present{nullptr};
      binary_sensor::BinarySensor *fireplace_present{nullptr};
      binary_sensor::BinarySensor *kitchen_hood_present{nullptr};
      binary_sensor::BinarySensor *postheating_present{nullptr};
      binary_sensor::BinarySensor *postheating_pwm_mode_present{nullptr};
      binary_sensor::BinarySensor *bypass_valve_open{nullptr};
      binary_sensor::BinarySensor *preheating_state{nullptr};
      binary_sensor::BinarySensor *summer_mode{nullptr};
      binary_sensor::BinarySensor *supply_fan_active{nullptr};
      binary_sensor::BinarySensor *p10_active{nullptr};
      binary_sensor::BinarySensor *p11_active{nullptr};
      binary_sensor::BinarySensor *p12_active{nullptr};
      binary_sensor::BinarySensor *p13_active{nullptr};
      binary_sensor::BinarySensor *p14_active{nullptr};
      binary_sensor::BinarySensor *p15_active{nullptr};
      binary_sensor::BinarySensor *p16_active{nullptr};
      binary_sensor::BinarySensor *p17_active{nullptr};
      binary_sensor::BinarySensor *p18_active{nullptr};
      binary_sensor::BinarySensor *p19_active{nullptr};
      binary_sensor::BinarySensor *p90_active{nullptr};
      binary_sensor::BinarySensor *p91_active{nullptr};
      binary_sensor::BinarySensor *p92_active{nullptr};
      binary_sensor::BinarySensor *p93_active{nullptr};
      binary_sensor::BinarySensor *p94_active{nullptr};
      binary_sensor::BinarySensor *p95_active{nullptr};
      binary_sensor::BinarySensor *p96_active{nullptr};
      binary_sensor::BinarySensor *p97_active{nullptr};
      sensor::Sensor *bathroom_switch_on_delay_minutes{nullptr};
      sensor::Sensor *bathroom_switch_off_delay_minutes{nullptr};
      sensor::Sensor *l1_switch_off_delay_minutes{nullptr};
      sensor::Sensor *boost_ventilation_minutes{nullptr};
      sensor::Sensor *filter_warning_weeks{nullptr};
      sensor::Sensor *rf_high_time_short_minutes{nullptr};
      sensor::Sensor *rf_high_time_long_minutes{nullptr};
      sensor::Sensor *extractor_hood_switch_off_delay_minutes{nullptr};
      sensor::Sensor *supply_absent_level{nullptr};
      sensor::Sensor *exhaust_absent_level{nullptr};
      sensor::Sensor *supply_low_level{nullptr};
      sensor::Sensor *exhaust_low_level{nullptr};
      sensor::Sensor *supply_medium_level{nullptr};
      sensor::Sensor *exhaust_medium_level{nullptr};
      sensor::Sensor *supply_high_level{nullptr};
      sensor::Sensor *exhaust_high_level{nullptr};
      // EWT / post-heating read-back sensors (0xEC)
      sensor::Sensor *ewt_low_temperature{nullptr};
      sensor::Sensor *ewt_high_temperature{nullptr};
      sensor::Sensor *ewt_speed_up_level{nullptr};
      sensor::Sensor *cookerhood_speed_up_level{nullptr};
      sensor::Sensor *reheating_target_temperature{nullptr};

      void set_bootloader_info(text_sensor::TextSensor *bootloader_info) { this->bootloader_info = bootloader_info; };
      void set_firmware_info(text_sensor::TextSensor *firmware_info) { this->firmware_info = firmware_info; };
      void set_connector_board_info(text_sensor::TextSensor *connector_board_info) { this->connector_board_info = connector_board_info; };
      void set_type(text_sensor::TextSensor *type) { this->type = type; };
      void set_size(text_sensor::TextSensor *size) { this->size = size; };
      void set_intake_fan_speed(sensor::Sensor *intake_fan_speed) { this->intake_fan_speed = intake_fan_speed; };
      void set_exhaust_fan_speed(sensor::Sensor *exhaust_fan_speed) { this->exhaust_fan_speed = exhaust_fan_speed; };
      void set_intake_fan_speed_rpm(sensor::Sensor *intake_fan_speed_rpm)
      {
        this->intake_fan_speed_rpm = intake_fan_speed_rpm;
        this->intake_fan_speed_rpm->set_accuracy_decimals(0);
      };
      void set_exhaust_fan_speed_rpm(sensor::Sensor *exhaust_fan_speed_rpm)
      {
        this->exhaust_fan_speed_rpm = exhaust_fan_speed_rpm;
        this->exhaust_fan_speed_rpm->set_accuracy_decimals(0);
      };
      void set_ventilation_level(sensor::Sensor *ventilation_level) { this->ventilation_level = ventilation_level; };
      void set_bypass_valve(sensor::Sensor *bypass_valve) { this->bypass_valve = bypass_valve; };
      void set_bypass_present(binary_sensor::BinarySensor *bypass_present) { this->bypass_present = bypass_present; };
      void set_enthalpy_present(binary_sensor::BinarySensor *enthalpy_present) { this->enthalpy_present = enthalpy_present; };
      void set_ewt_present(binary_sensor::BinarySensor *ewt_present) { this->ewt_present = ewt_present; };
      void set_preheating_present(binary_sensor::BinarySensor *preheating_present) { this->preheating_present = preheating_present; };
      void set_options_present(binary_sensor::BinarySensor *options_present) { this->options_present = options_present; };
      void set_fireplace_present(binary_sensor::BinarySensor *fireplace_present) { this->fireplace_present = fireplace_present; };
      void set_kitchen_hood_present(binary_sensor::BinarySensor *kitchen_hood_present) { this->kitchen_hood_present = kitchen_hood_present; };
      void set_postheating_present(binary_sensor::BinarySensor *postheating_present) { this->postheating_present = postheating_present; };
      void set_postheating_pwm_mode_present(binary_sensor::BinarySensor *postheating_pwm_mode_present) { this->postheating_pwm_mode_present = postheating_pwm_mode_present; };
      void set_bypass_valve_open(binary_sensor::BinarySensor *bypass_valve_open) { this->bypass_valve_open = bypass_valve_open; };
      void set_preheating_state(binary_sensor::BinarySensor *preheating_state) { this->preheating_state = preheating_state; };
      void set_outside_air_temperature(sensor::Sensor *outside_air_temperature) { this->outside_air_temperature = outside_air_temperature; };
      void set_supply_air_temperature(sensor::Sensor *supply_air_temperature) { this->supply_air_temperature = supply_air_temperature; };
      void set_return_air_temperature(sensor::Sensor *return_air_temperature) { this->return_air_temperature = return_air_temperature; };
      void set_exhaust_air_temperature(sensor::Sensor *exhaust_air_temperature) { this->exhaust_air_temperature = exhaust_air_temperature; };
      void set_enthalpy_temperature(sensor::Sensor *enthalpy_temperature) { this->enthalpy_temperature = enthalpy_temperature; };
      void set_ewt_temperature(sensor::Sensor *ewt_temperature) { this->ewt_temperature = ewt_temperature; };
      void set_reheating_temperature(sensor::Sensor *reheating_temperature) { this->reheating_temperature = reheating_temperature; };
      void set_kitchen_hood_temperature(sensor::Sensor *kitchen_hood_temperature) { this->kitchen_hood_temperature = kitchen_hood_temperature; };
      void set_return_air_level(sensor::Sensor *return_air_level) { this->return_air_level = return_air_level; };
      void set_supply_air_level(sensor::Sensor *supply_air_level) { this->supply_air_level = supply_air_level; };
      void set_supply_fan_active(binary_sensor::BinarySensor *supply_fan_active) { this->supply_fan_active = supply_fan_active; };
      void set_filter_status(text_sensor::TextSensor *filter_status) { this->filter_status = filter_status; };
      void set_bypass_factor(sensor::Sensor *bypass_factor) { this->bypass_factor = bypass_factor; };
      void set_bypass_step(sensor::Sensor *bypass_step) { this->bypass_step = bypass_step; };
      void set_bypass_correction(sensor::Sensor *bypass_correction) { this->bypass_correction = bypass_correction; };
      void set_bypass_open_hours(sensor::Sensor *bypass_open_hours) { this->bypass_open_hours = bypass_open_hours; };
      void set_motor_current_bypass(sensor::Sensor *motor_current_bypass) { this->motor_current_bypass = motor_current_bypass; };
      void set_motor_current_preheating(sensor::Sensor *motor_current_preheating) { this->motor_current_preheating = motor_current_preheating; };
      void set_preheating_hours(sensor::Sensor *preheating_hours) { this->preheating_hours = preheating_hours; };
      void set_preheating_valve(text_sensor::TextSensor *preheating_valve) { this->preheating_valve = preheating_valve; };
      void set_level0_hours(sensor::Sensor *level0_hours) { this->level0_hours = level0_hours; };
      void set_level1_hours(sensor::Sensor *level1_hours) { this->level1_hours = level1_hours; };
      void set_level2_hours(sensor::Sensor *level2_hours) { this->level2_hours = level2_hours; };
      void set_level3_hours(sensor::Sensor *level3_hours) { this->level3_hours = level3_hours; };
      void set_frost_protection_active(binary_sensor::BinarySensor *frost_protection_active) { this->frost_protection_active = frost_protection_active; };
      void set_frost_protection_hours(sensor::Sensor *frost_protection_hours) { this->frost_protection_hours = frost_protection_hours; };
      void set_frost_protection_minutes(sensor::Sensor *frost_protection_minutes) { this->frost_protection_minutes = frost_protection_minutes; };
      void set_frost_protection_level(text_sensor::TextSensor *frost_protection_level) { this->frost_protection_level = frost_protection_level; };
      void set_filter_hours(sensor::Sensor *filter_hours) { this->filter_hours = filter_hours; };
      void set_summer_mode(binary_sensor::BinarySensor *summer_mode) { this->summer_mode = summer_mode; };
      void set_p10_active(binary_sensor::BinarySensor *p10_active) { this->p10_active = p10_active; };
      void set_p11_active(binary_sensor::BinarySensor *p11_active) { this->p11_active = p11_active; };
      void set_p12_active(binary_sensor::BinarySensor *p12_active) { this->p12_active = p12_active; };
      void set_p13_active(binary_sensor::BinarySensor *p13_active) { this->p13_active = p13_active; };
      void set_p14_active(binary_sensor::BinarySensor *p14_active) { this->p14_active = p14_active; };
      void set_p15_active(binary_sensor::BinarySensor *p15_active) { this->p15_active = p15_active; };
      void set_p16_active(binary_sensor::BinarySensor *p16_active) { this->p16_active = p16_active; };
      void set_p17_active(binary_sensor::BinarySensor *p17_active) { this->p17_active = p17_active; };
      void set_p18_active(binary_sensor::BinarySensor *p18_active) { this->p18_active = p18_active; };
      void set_p19_active(binary_sensor::BinarySensor *p19_active) { this->p19_active = p19_active; };
      void set_p90_active(binary_sensor::BinarySensor *p90_active) { this->p90_active = p90_active; };
      void set_p91_active(binary_sensor::BinarySensor *p91_active) { this->p91_active = p91_active; };
      void set_p92_active(binary_sensor::BinarySensor *p92_active) { this->p92_active = p92_active; };
      void set_p93_active(binary_sensor::BinarySensor *p93_active) { this->p93_active = p93_active; };
      void set_p94_active(binary_sensor::BinarySensor *p94_active) { this->p94_active = p94_active; };
      void set_p95_active(binary_sensor::BinarySensor *p95_active) { this->p95_active = p95_active; };
      void set_p96_active(binary_sensor::BinarySensor *p96_active) { this->p96_active = p96_active; };
      void set_p97_active(binary_sensor::BinarySensor *p97_active) { this->p97_active = p97_active; };
      void set_bathroom_switch_on_delay_minutes(sensor::Sensor *bathroom_switch_on_delay_minutes) { this->bathroom_switch_on_delay_minutes = bathroom_switch_on_delay_minutes; };
      void set_bathroom_switch_off_delay_minutes(sensor::Sensor *bathroom_switch_off_delay_minutes) { this->bathroom_switch_off_delay_minutes = bathroom_switch_off_delay_minutes; };
      void set_l1_switch_off_delay_minutes(sensor::Sensor *l1_switch_off_delay_minutes) { this->l1_switch_off_delay_minutes = l1_switch_off_delay_minutes; };
      void set_boost_ventilation_minutes(sensor::Sensor *boost_ventilation_minutes) { this->boost_ventilation_minutes = boost_ventilation_minutes; };
      void set_filter_warning_weeks(sensor::Sensor *filter_warning_weeks) { this->filter_warning_weeks = filter_warning_weeks; };
      void set_rf_high_time_short_minutes(sensor::Sensor *rf_high_time_short_minutes) { this->rf_high_time_short_minutes = rf_high_time_short_minutes; };
      void set_rf_high_time_long_minutes(sensor::Sensor *rf_high_time_long_minutes) { this->rf_high_time_long_minutes = rf_high_time_long_minutes; };
      void set_extractor_hood_switch_off_delay_minutes(sensor::Sensor *extractor_hood_switch_off_delay_minutes) { this->extractor_hood_switch_off_delay_minutes = extractor_hood_switch_off_delay_minutes; };

      void set_exhaust_levels(uint8_t value_abs, uint8_t value_low, uint8_t value_medium, uint8_t value_high)
      {
        exhaust_absent_ = value_abs;
        exhaust_low_ = value_low;
        exhaust_medium_ = value_medium;
        exhaust_high_ = value_high;
        send_ventilation_levels_();
      }

      void set_exhaust_absent_level(uint8_t value)
      {
        exhaust_absent_ = value;
        send_ventilation_levels_();
      }

      void set_exhaust_low_level(uint8_t value)
      {
        exhaust_low_ = value;
        send_ventilation_levels_();
      }

      void set_exhaust_medium_level(uint8_t value)
      {
        exhaust_medium_ = value;
        send_ventilation_levels_();
      }

      void set_exhaust_high_level(uint8_t value)
      {
        exhaust_high_ = value;
        send_ventilation_levels_();
      }

      void set_supply_levels(uint8_t value_abs, uint8_t value_low, uint8_t value_medium, uint8_t value_high)
      {
        supply_absent_ = value_abs;
        supply_low_ = value_low;
        supply_medium_ = value_medium;
        supply_high_ = value_high;
        send_ventilation_levels_();
      }

      void set_supply_absent_level(uint8_t value)
      {
        supply_absent_ = value;
        send_ventilation_levels_();
      }

      void set_supply_low_level(uint8_t value)
      {
        supply_low_ = value;
        send_ventilation_levels_();
      }

      void set_supply_medium_level(uint8_t value)
      {
        supply_medium_ = value;
        send_ventilation_levels_();
      }

      void set_supply_high_level(uint8_t value)
      {
        supply_high_ = value;
        send_ventilation_levels_();
      }

      uint8_t get_supply_absent_level() const { return supply_absent_; }
      uint8_t get_exhaust_absent_level() const { return exhaust_absent_; }
      uint8_t get_supply_low_level() const { return supply_low_; }
      uint8_t get_exhaust_low_level() const { return exhaust_low_; }
      uint8_t get_supply_medium_level() const { return supply_medium_; }
      uint8_t get_exhaust_medium_level() const { return exhaust_medium_; }
      uint8_t get_supply_high_level() const { return supply_high_; }
      uint8_t get_exhaust_high_level() const { return exhaust_high_; }

      // ---- EWT / post-heating read-back sensor setters ----
      void set_ewt_low_temperature(sensor::Sensor *s) { this->ewt_low_temperature = s; };
      void set_ewt_high_temperature(sensor::Sensor *s) { this->ewt_high_temperature = s; };
      void set_ewt_speed_up_level(sensor::Sensor *s) { this->ewt_speed_up_level = s; };
      void set_cookerhood_speed_up_level(sensor::Sensor *s) { this->cookerhood_speed_up_level = s; };
      void set_reheating_target_temperature(sensor::Sensor *s) { this->reheating_target_temperature = s; };

      // =====================================================================
      // ===================== Tier 1: Time delays (0xCB) ====================
      // =====================================================================
      // The device accepts the full 8-byte block only; always read-modify-write
      // from the cached values so unrelated delays are preserved.

      void send_time_delay_()
      {
        if (!can_send_time_delay_)
        {
          LOG_WARN(TAG, "Can't send time delays yet (block not read)");
          return;
        }
        uint8_t command_data[8] = {
            td_bathroom_on_,
            td_bathroom_off_,
            td_l1_off_,
            td_boost_,
            td_filter_weeks_,
            td_rf_short_,
            td_rf_long_,
            td_cookerhood_off_};
        write_command_(CMD_SET_TIME_DELAY, command_data, sizeof(command_data));
      }

      void set_bathroom_switch_on_delay(uint8_t value) { td_bathroom_on_ = value; send_time_delay_(); }
      void set_bathroom_switch_off_delay(uint8_t value) { td_bathroom_off_ = value; send_time_delay_(); }
      void set_l1_switch_off_delay(uint8_t value) { td_l1_off_ = value; send_time_delay_(); }
      void set_boost_ventilation_delay(uint8_t value) { td_boost_ = value; send_time_delay_(); }
      void set_filter_warning_weeks(uint8_t value) { td_filter_weeks_ = value; send_time_delay_(); }
      void set_rf_high_time_short(uint8_t value) { td_rf_short_ = value; send_time_delay_(); }
      void set_rf_high_time_long(uint8_t value) { td_rf_long_ = value; send_time_delay_(); }
      void set_cookerhood_switch_off_delay(uint8_t value) { td_cookerhood_off_ = value; send_time_delay_(); }

      uint8_t get_bathroom_switch_on_delay() const { return td_bathroom_on_; }
      uint8_t get_bathroom_switch_off_delay() const { return td_bathroom_off_; }
      uint8_t get_l1_switch_off_delay() const { return td_l1_off_; }
      uint8_t get_boost_ventilation_delay() const { return td_boost_; }
      uint8_t get_filter_warning_weeks_value() const { return td_filter_weeks_; }
      uint8_t get_rf_high_time_short() const { return td_rf_short_; }
      uint8_t get_rf_high_time_long() const { return td_rf_long_; }
      uint8_t get_cookerhood_switch_off_delay() const { return td_cookerhood_off_; }

      // =====================================================================
      // ================ Tier 1: EWT / post-heating (0xED) ==================
      // =====================================================================
      // 5-byte write: [EWT low °C][EWT high °C][EWT speed %][cookerhood speed %]
      //               [reheat target °C]. Temps encoded (°C+20)*2, % is raw.

      void send_ewt_postheating_()
      {
        if (!can_send_ewt_postheating_)
        {
          LOG_WARN(TAG, "Can't send EWT/postheating yet (block not read)");
          return;
        }
        uint8_t command_data[5] = {
            ewt_low_,
            ewt_high_,
            ewt_speed_up_,
            cookerhood_speed_up_,
            reheating_target_};
        write_command_(CMD_SET_EWT_POSTHEATING, command_data, sizeof(command_data));
      }

      static uint8_t encode_temperature_(float celsius)
      {
        return (uint8_t)((celsius + 20.0f) * 2.0f);
      }

      void set_ewt_low_temperature_value(float celsius) { ewt_low_ = encode_temperature_(celsius); send_ewt_postheating_(); }
      void set_ewt_high_temperature_value(float celsius) { ewt_high_ = encode_temperature_(celsius); send_ewt_postheating_(); }
      void set_ewt_speed_up(uint8_t percent) { ewt_speed_up_ = percent; send_ewt_postheating_(); }
      void set_cookerhood_speed_up(uint8_t percent) { cookerhood_speed_up_ = percent; send_ewt_postheating_(); }
      void set_reheating_target_temperature_value(float celsius) { reheating_target_ = encode_temperature_(celsius); send_ewt_postheating_(); }

      float get_ewt_low_temperature_value() const { return (float)ewt_low_ / 2.0f - 20.0f; }
      float get_ewt_high_temperature_value() const { return (float)ewt_high_ / 2.0f - 20.0f; }
      uint8_t get_ewt_speed_up() const { return ewt_speed_up_; }
      uint8_t get_cookerhood_speed_up() const { return cookerhood_speed_up_; }
      float get_reheating_target_temperature_value() const { return (float)reheating_target_ / 2.0f - 20.0f; }

      // =====================================================================
      // ================= Tier 1/2: Resets & self-test (0xDB) ===============
      // =====================================================================
      // error_reset() and filter_reset() already exist above. Add the rest.

      void settings_reset(void)
      {
        uint8_t reset_cmd[4] = {0, 1, 0, 0};
        write_command_(CMD_RESET_AND_SELF_TEST, reset_cmd, sizeof(reset_cmd));
      }

      void start_self_test(void)
      {
        uint8_t reset_cmd[4] = {0, 0, 1, 0};
        write_command_(CMD_RESET_AND_SELF_TEST, reset_cmd, sizeof(reset_cmd));
      }

      // =====================================================================
      // ================ Tier 2: Unit config / option flags (0xD7) ==========
      // =====================================================================
      // 8-byte write. We read-modify-write from the cached 0xD6 reply so that
      // flags we don't touch are preserved. NOTE: per the protocol investigation,
      // the "preheater present" position differs between sources (registers.h
      // byte 0 vs openHAB byte4 bit0x40) and is UNVERIFIED on the WHR 930.
      // These are installer/config flags — guard behind developer mode.

      void send_status_()
      {
        if (!can_send_status_)
        {
          LOG_WARN(TAG, "Can't send status/config yet (block not read)");
          return;
        }
        // 0xD6 read reply is 11 bytes; the 0xD7 write block is 8 bytes:
        // [preheater present][bypass present][type][size][options bitmask]
        // [reserved=0][enthalpy present][EWT present]
        uint8_t command_data[8] = {
            status_preheater_present_,
            status_bypass_present_,
            status_type_,
            status_size_,
            status_options_,
            0x00,
            status_enthalpy_present_,
            status_ewt_present_};
        write_command_(CMD_SET_STATUS, command_data, sizeof(command_data));
      }

      void set_cfg_preheater_present(bool present) { status_preheater_present_ = present ? 1 : 0; send_status_(); }
      void set_cfg_bypass_present(bool present) { status_bypass_present_ = present ? 1 : 0; send_status_(); }
      void set_cfg_type(uint8_t value) { status_type_ = value; send_status_(); }
      void set_cfg_size(uint8_t value) { status_size_ = value; send_status_(); }
      void set_cfg_enthalpy_present(uint8_t value) { status_enthalpy_present_ = value; send_status_(); }
      void set_cfg_ewt_present(uint8_t value) { status_ewt_present_ = value; send_status_(); }

      void set_cfg_option_bit_(uint8_t mask, bool enable)
      {
        if (enable)
          status_options_ |= mask;
        else
          status_options_ &= ~mask;
        send_status_();
      }
      void set_cfg_fireplace_present(bool v) { set_cfg_option_bit_(0x01, v); }
      void set_cfg_kitchen_hood_present(bool v) { set_cfg_option_bit_(0x02, v); }
      void set_cfg_postheating_present(bool v) { set_cfg_option_bit_(0x04, v); }
      void set_cfg_postheating_pwm_present(bool v) { set_cfg_option_bit_(0x40, v); }

      bool get_cfg_preheater_present() const { return status_preheater_present_ != 0; }
      bool get_cfg_bypass_present() const { return status_bypass_present_ != 0; }
      uint8_t get_cfg_type() const { return status_type_; }
      uint8_t get_cfg_size() const { return status_size_; }
      uint8_t get_cfg_enthalpy_present() const { return status_enthalpy_present_; }
      uint8_t get_cfg_ewt_present() const { return status_ewt_present_; }
      bool get_cfg_fireplace_present() const { return (status_options_ & 0x01) != 0; }
      bool get_cfg_kitchen_hood_present() const { return (status_options_ & 0x02) != 0; }
      bool get_cfg_postheating_present() const { return (status_options_ & 0x04) != 0; }
      bool get_cfg_postheating_pwm_present() const { return (status_options_ & 0x40) != 0; }

      // =====================================================================
      // ============== Tier 2: Analog input configuration (0x9F) ============
      // =====================================================================
      // 19-byte block. Advanced; read-modify-write the cached 0x9E reply.

      void send_analog_values_()
      {
        if (!can_send_analog_values_)
        {
          LOG_WARN(TAG, "Can't send analog values yet (block not read)");
          return;
        }
        write_command_(CMD_SET_ANALOG_VALUES, analog_values_, sizeof(analog_values_));
      }

      void set_analog_byte(uint8_t index, uint8_t value)
      {
        if (index >= sizeof(analog_values_))
        {
          LOG_WARN(TAG, "Analog byte index out of range: %u", index);
          return;
        }
        analog_values_[index] = value;
        send_analog_values_();
      }
      uint8_t get_analog_byte(uint8_t index) const
      {
        if (index >= sizeof(analog_values_))
          return 0;
        return analog_values_[index];
      }

      void set_analog_priority(uint8_t value) { set_analog_byte(18, value); }
      uint8_t get_analog_priority() const { return get_analog_byte(18); }

      // =====================================================================
      // ============ Tier 3: Test-mode preheater override (0x01/0x05/0x19) ==
      // =====================================================================
      // DANGER: entering test mode suspends the unit's OWN regulation (including
      // automatic frost protection). We drive the preheater relay+triac directly
      // and auto-exit after a timeout so the unit is never left un-regulated.
      // Gated behind developer mode by the YAML.
      //
      // IMPORTANT: commands must NOT be sent back-to-back in a single call. The
      // bus is half-duplex and the unit replies to each frame; two frames in the
      // same tick collide and corrupt both (symptom: "Checksum doesn't match").
      // So the override is a small state machine, pumped once per test_mode_pump_()
      // from loop() with a minimum spacing between frames.

      // Begin a timed preheater override. Does NOT send anything synchronously;
      // the state machine sends enter -> drive -> exit with proper spacing.
      void force_preheater(uint32_t duration_seconds, uint8_t relay_mask = 0x03)
      {
        test_use_flap_ = false;
        test_drive_byte_a_ = (uint8_t)(relay_mask & 0x3F);
        test_drive_byte_b_ = 0x00;
        test_duration_ms_ = duration_seconds * 1000UL;
        test_state_ = TestState::ENTER;
        test_next_action_at_ms_ = millis();
        LOG_WARN(TAG, "Preheater override requested (mask 0x%02X) for %u s", relay_mask, duration_seconds);
      }

      // Drive the preheating flap directly (1 open / 0 closed / 3 stop).
      void force_preheater_flap(uint8_t state, uint32_t duration_seconds)
      {
        test_use_flap_ = true;
        test_drive_byte_a_ = 0x03; // bypass flap = stop (leave as-is)
        test_drive_byte_b_ = state;
        test_duration_ms_ = duration_seconds * 1000UL;
        test_state_ = TestState::ENTER;
        test_next_action_at_ms_ = millis();
        LOG_WARN(TAG, "Preheater flap override requested (state %u) for %u s", state, duration_seconds);
      }

      // Request a clean exit from an active override.
      void stop_test_mode_override()
      {
        if (test_state_ == TestState::IDLE)
          return;
        test_state_ = TestState::EXIT;
        test_next_action_at_ms_ = millis();
        LOG_WARN(TAG, "Preheater override stop requested");
      }

      bool get_test_mode_active() const { return test_mode_active_; }

      // Pump the test-mode state machine. Called from loop(); sends at most one
      // frame per invocation and enforces a minimum gap between frames.
      void test_mode_pump_()
      {
        if (test_state_ == TestState::IDLE)
          return;

        uint32_t now = millis();
        if ((int32_t)(now - test_next_action_at_ms_) < 0)
          return; // not time for the next step yet

        switch (test_state_)
        {
        case TestState::ENTER:
          write_command_(CMD_SET_TEST_MODE, nullptr, 0);
          test_mode_active_ = true;
          LOG_WARN(TAG, "ENTERED TEST MODE - unit self-regulation is suspended");
          test_next_action_at_ms_ = now + test_frame_gap_ms_;
          test_state_ = TestState::DRIVE;
          break;

        case TestState::DRIVE:
        {
          uint8_t cmd = test_use_flap_ ? CMD_SET_FLAPS : CMD_SET_OUTPUTS;
          uint8_t payload[2] = {test_drive_byte_a_, test_drive_byte_b_};
          write_command_(cmd, payload, sizeof(payload));
          LOG_WARN(TAG, "Test-mode drive: cmd 0x%02X data %02X %02X", cmd, payload[0], payload[1]);
          test_mode_exit_at_ms_ = now + test_duration_ms_;
          test_next_action_at_ms_ = now + test_frame_gap_ms_;
          test_state_ = TestState::HOLD;
          break;
        }

        case TestState::HOLD:
          // Hold until the override deadline, then exit. Re-assert the drive
          // periodically so the unit doesn't time the outputs out on its own.
          if ((int32_t)(now - test_mode_exit_at_ms_) >= 0)
          {
            test_state_ = TestState::EXIT;
            test_next_action_at_ms_ = now;
          }
          else
          {
            uint8_t cmd = test_use_flap_ ? CMD_SET_FLAPS : CMD_SET_OUTPUTS;
            uint8_t payload[2] = {test_drive_byte_a_, test_drive_byte_b_};
            write_command_(cmd, payload, sizeof(payload));
            test_next_action_at_ms_ = now + test_reassert_ms_;
          }
          break;

        case TestState::EXIT:
          write_command_(CMD_EXIT_TEST_MODE, nullptr, 0);
          test_mode_active_ = false;
          test_mode_exit_at_ms_ = 0;
          test_state_ = TestState::IDLE;
          LOG_WARN(TAG, "Exited test mode - unit resumes self-regulation");
          break;

        case TestState::IDLE:
          break;
        }
      }
    };

  } // namespace comfoair_ca
} // namespace esphome
