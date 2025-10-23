#include "sim/integration/terminal_keyboard.hpp"

#include "KeyboardInputHandler.h"

namespace fsai::sim::integration {

TerminalKeyboard::TerminalKeyboard() { KeyboardInputHandler_Init(); }

TerminalKeyboard::~TerminalKeyboard() { KeyboardInputHandler_Restore(); }

int TerminalKeyboard::poll() const { return KeyboardInputHandler_GetInput(); }

}  // namespace fsai::sim::integration
