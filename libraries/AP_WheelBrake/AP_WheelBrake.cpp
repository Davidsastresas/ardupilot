#include "AP_WheelBrake.h"

#include "AP_WheelBrake_pwm.h"

extern const AP_HAL::HAL& hal;

#define wheelbrake_BRAKE_PWM_DEFAULT        1900
#define wheelbrake_RELEASE_PWM_DEFAULT     1100

const AP_Param::GroupInfo AP_WheelBrake::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: wheelbrake Enable/Disable
    // @Description: wheelbrake enable/disable
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_WheelBrake, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: wheelbrake Type
    // @Description: wheelbrake enable/disable
    // @User: Standard
    // @Values: 0:None,1:Servo,2:EPM
    AP_GROUPINFO("TYPE", 1, AP_WheelBrake, config.type, 0),

    // @Param: GRAB
    // @DisplayName: wheelbrake brake PWM
    // @Description: PWM value in microseconds sent to wheelbrake to start to brake
    // @User: Advanced
    // @Range: 1000 2000
    // @Units: PWM
    AP_GROUPINFO("BRAKE",    2, AP_WheelBrake, config.brake_pwm, wheelbrake_BRAKE_PWM_DEFAULT),

    // @Param: RELEASE
    // @DisplayName: wheelbrake Release PWM
    // @Description: PWM value in microseconds sent to wheelbrake to release the brake
    // @User: Advanced
    // @Range: 1000 2000
    // @Units: PWM
    AP_GROUPINFO("RELEASE", 3, AP_WheelBrake, config.release_pwm, wheelbrake_RELEASE_PWM_DEFAULT),

    AP_GROUPEND
};

AP_WheelBrake::AP_WheelBrake()
{
    if ( _singleton ) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many wheelbrakes");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Get the AP_WheelBrake singleton
 */
AP_WheelBrake *AP_WheelBrake::_singleton = nullptr;
AP_WheelBrake *AP_WheelBrake::get_singleton()
{
    return _singleton;
}

void AP_WheelBrake::init()
{
    // return immediately if not enabled
    if (!_enabled.get()) {
        return;
    }

    switch(config.type.get()) {
    case 0:
        break;
    case 1:
        backend = new AP_WheelBrake_pwm(config);
        break;
    default:
        break;
    }
    if (backend != nullptr) {
        backend->init();
    }
}

// update - should be called at at least 10hz
#define PASS_TO_BACKEND(function_name) \
    void AP_WheelBrake::function_name()   \
    {                                  \
        if (!enabled()) {              \
            return;                    \
        }                              \
        if (backend != nullptr) {      \
            backend->function_name();  \
        }                              \
    }

PASS_TO_BACKEND(brake)
PASS_TO_BACKEND(release)
PASS_TO_BACKEND(update)

#undef PASS_TO_BACKEND


#define PASS_TO_BACKEND(function_name)        \
    bool AP_WheelBrake::function_name() const    \
    {                                         \
        if (!enabled()) {                     \
            return false;                     \
        }                                     \
        if (backend != nullptr) {             \
            return backend->function_name();  \
        }                                     \
        return false;                         \
    }

PASS_TO_BACKEND(valid)
PASS_TO_BACKEND(braked)
PASS_TO_BACKEND(released)

#undef PASS_TO_BACKEND

namespace AP {

AP_WheelBrake *wheelbrake()
{
    return AP_WheelBrake::get_singleton();
}

};
