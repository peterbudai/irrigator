#pragma once

#include "esphome/core/component.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace status_select {

class StatusSelect : public Component {
 public:
  void set_output(select::Select *output) { this->output_ = output; }
  void set_idle_option(const char *option) { this->idle_option_ = option; }
  void set_warning_option(const char *option) { this->warning_option_ = option; }
  void set_error_option(const char *option) { this->error_option_ = option; }

  void dump_config() override;
  void loop() override;

  float get_setup_priority() const override;
#ifdef USE_LOOP_PRIORITY
  float get_loop_priority() const override { return 50.0f; }
#endif

 protected:
  select::Select *output_{nullptr};
  const char *idle_option_{"idle"};
  const char *warning_option_{"warning"};
  const char *error_option_{"error"};
  uint8_t last_app_state_{0xFF};
};

}  // namespace status_select
}  // namespace esphome
