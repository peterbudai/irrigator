#include "status_select.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace status_select {

static const char *const TAG = "status_select";

void StatusSelect::loop() {
  uint8_t new_state = App.get_app_state() & STATUS_LED_MASK;

  if (new_state == this->last_app_state_)
    return;

  this->last_app_state_ = new_state;

  const char *option;
  if ((new_state & STATUS_LED_ERROR) != 0u) {
    option = this->error_option_;
  } else if ((new_state & STATUS_LED_WARNING) != 0u) {
    option = this->warning_option_;
  } else {
    option = this->idle_option_;
  }

  ESP_LOGD(TAG, "App state 0x%02X -> '%s'", new_state, option);
  this->output_->make_call().set_option(option).perform();
}

void StatusSelect::dump_config() {
  ESP_LOGCONFIG(TAG, "Status Select:");
  ESP_LOGCONFIG(TAG, "  Idle option: %s", this->idle_option_);
  ESP_LOGCONFIG(TAG, "  Warning option: %s", this->warning_option_);
  ESP_LOGCONFIG(TAG, "  Error option: %s", this->error_option_);
}

float StatusSelect::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace status_select
}  // namespace esphome
