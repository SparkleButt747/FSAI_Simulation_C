#pragma once

namespace fsai::sim::integration {

class TerminalKeyboard {
 public:
  TerminalKeyboard();
  ~TerminalKeyboard();

  TerminalKeyboard(const TerminalKeyboard&) = delete;
  TerminalKeyboard& operator=(const TerminalKeyboard&) = delete;

  int poll() const;
};

}  // namespace fsai::sim::integration
