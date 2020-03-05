#include "AP_WheelBrake_Backend.h"

extern const AP_HAL::HAL& hal;

void AP_WheelBrake_Backend::init()
{
    init_wheelbrake();
}

// update - should be called at at least 10hz
void AP_WheelBrake_Backend::update()
{
    update_wheelbrake();
}
