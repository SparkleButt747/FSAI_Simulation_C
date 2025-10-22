#ifndef KEYBOARDINPUTHANDLER_H
#define KEYBOARDINPUTHANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize keyboard input (set terminal to non-blocking raw mode).
void KeyboardInputHandler_Init(void);

// Restore original terminal settings.
void KeyboardInputHandler_Restore(void);

// Returns a character from standard input if available; returns -1 if no key was pressed.
int KeyboardInputHandler_GetInput(void);

#ifdef __cplusplus
}
#endif

#endif // KEYBOARDINPUTHANDLER_H
