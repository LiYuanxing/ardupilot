#include "mode.h"
#include "Plane.h"

bool ModeAutoTune::_enter()
{
    plane.auto_throttle_mode = false;
    plane.autotune_start();

    return true;
}

void ModeAutoTune::_exit()
{
    // restore last gains
    plane.autotune_restore();
}

void ModeAutoTune::update()
{
    plane.mode_fbwa.update();
}

