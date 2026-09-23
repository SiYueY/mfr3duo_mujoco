#include "keyboard.hpp"

#include <poll.h>
#include <unistd.h>

namespace mfr3duo_mujoco::teleop {

Keyboard::~Keyboard() { close(); }

bool Keyboard::open() {
    if (open_) return true;
    if (!::isatty(STDIN_FILENO)) return false;
    if (::tcgetattr(STDIN_FILENO, &original_) != 0) return false;

    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_iflag &= static_cast<tcflag_t>(~(IXON | ICRNL));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;
    open_ = true;
    return true;
}

void Keyboard::close() {
    if (!open_) return;
    (void)::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    open_ = false;
}

bool Keyboard::read(char& key) const {
    if (!open_) return false;

    pollfd descriptor{};
    descriptor.fd = STDIN_FILENO;
    descriptor.events = POLLIN;

    const int result = ::poll(&descriptor, 1, 0);
    if (result <= 0 || (descriptor.revents & POLLIN) == 0) return false;

    char value{};
    const ssize_t count = ::read(STDIN_FILENO, &value, 1);
    if (count != 1) return false;

    key = value;
    return true;
}

}  // namespace mfr3duo_mujoco::teleop
