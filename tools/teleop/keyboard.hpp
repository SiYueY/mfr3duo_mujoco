#pragma once

#include <termios.h>

namespace mfr3duo_mujoco::teleop {

class Keyboard {
public:
    Keyboard() = default;
    ~Keyboard();

    Keyboard(const Keyboard&) = delete;
    Keyboard& operator=(const Keyboard&) = delete;

    bool open();
    void close();
    bool read(char& key) const;

private:
    termios original_{};
    bool open_{false};
};

}  // namespace mfr3duo_mujoco::teleop
