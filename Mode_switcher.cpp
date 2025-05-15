#include "Mode_switcher.hpp"
#include <cstdio>

Mode_switcher::Mode_switcher()
{
    mode = false;
    start();
}

Mode_switcher::~Mode_switcher()
{
}

bool Mode_switcher::get_mode()
{
    return mode;
}

void Mode_switcher::start()
{
    std::thread body_thread(&Mode_switcher::run, this);
    body_thread.detach();
}

void Mode_switcher::run()
{
    while(true)
    {
        getchar();
        mode = !mode;
    }
}
