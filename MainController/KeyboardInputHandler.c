#include "KeyboardInputHandler.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

static struct termios orig_termios;

void KeyboardInputHandler_Init(void) {
    struct termios newt;
    // Save current terminal settings.
    tcgetattr(STDIN_FILENO, &orig_termios);
    newt = orig_termios;
    // Disable canonical mode and echo.
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    // Set stdin to non-blocking.
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void KeyboardInputHandler_Restore(void) {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

int KeyboardInputHandler_GetInput(void) {
    int ch = getchar();
    if (ch == EOF) {
        return -1;
    }
    return ch;
}
