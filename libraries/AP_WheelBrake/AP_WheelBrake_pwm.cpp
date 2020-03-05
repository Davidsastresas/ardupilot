#include <AP_WheelBrake/AP_WheelBrake_pwm.h>
#include <GCS_MAVLink/GCS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
  #include <SITL/SITL.h>
#endif

extern const AP_HAL::HAL& hal;

void AP_WheelBrake_pwm::init_wheelbrake()
{
    // move the pwm to the release position
    release();
}

void AP_WheelBrake_pwm::brake()
{
    // move the pwm to the brake position
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting16, config.brake_pwm);
    gcs().send_text(MAV_SEVERITY_INFO, "Wheel brakes Activated");
}

void AP_WheelBrake_pwm::release()
{
    // move the pwm to the release position
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting16, config.release_pwm);
    gcs().send_text(MAV_SEVERITY_INFO, "Wheel brakes Deactivated");
}

bool AP_WheelBrake_pwm::has_state_pwm(const uint16_t pwm) const
{
    // return true if pwm is in position represented by pwm
    uint16_t current_pwm;
    if (!SRV_Channels::get_output_pwm(SRV_Channel::k_scripting16, current_pwm)) {
        // function not assigned to a channel, perhaps?
        return false;
    }
    if (current_pwm != pwm) {
        // last action did not set pwm to the current value
        // (e.g. last action was a brake not a release)
        return false;
    }
    return true;
}


bool AP_WheelBrake_pwm::released() const
{
    return has_state_pwm(config.release_pwm);
}

bool AP_WheelBrake_pwm::braked() const
{
    return has_state_pwm(config.brake_pwm);
}

// type-specific periodic updates:
void AP_WheelBrake_pwm::update_wheelbrake() 
{

};

bool AP_WheelBrake_pwm::valid() const
{
    if (!AP_WheelBrake_Backend::valid()) {
        return false;
    }
    if (!SRV_Channels::function_assigned(SRV_Channel::k_scripting16)) {
        return false;
    }
    return true;
}
