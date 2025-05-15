#pragma once

#include <thread>

class Mode_switcher
{
public:
    Mode_switcher();
    ~Mode_switcher();
    bool get_mode();

private:
    void start();
    void run();

    bool mode = false;
};
