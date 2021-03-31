#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

// #ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    if( !mn_photo_triggered ) {
        return;
    }

    if( millis() - mn_photo_triggered_time >= 350 ) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cameraMode, 1520); //1520 corresponds to stop recording/Stop taking photo
        mn_photo_triggered = false;
    }
}
// #endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::trigger_multinnov_photo(void) 
{
    if (!copter.mn_photo_triggered) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cameraMode, 1080);
        copter.mn_photo_triggered = true;
        copter.mn_photo_triggered_time = millis();
    }
}

// This function checks at 10 hz the conditions of flight mode
// and viso status for automatic flight mode and ekf source change
void Copter::auto_flight_mode_check_loop(void) {
    // if not auto mode switch flag return inmediately
    if (!copter.mn_auto_mode_switch || !copter.g2.auto_mode_switch_enabled) {
        mn_auto_mode_switch_engaged = false;
        return;
    }

    bool confidence_ok = copter.visual_odom.confidence_ok();
    bool in_loiter = copter.flightmode == &copter.mode_loiter;
    bool in_althold = copter.flightmode == &copter.mode_althold;

    if (in_loiter) {
        if (confidence_ok) {
            mn_auto_mode_switch_engaged = false;
            return;
        }

        if (!mn_auto_mode_switch_engaged) {
            mn_auto_mode_switch_engaged = true;
            mn_auto_mode_switch_time = millis();
        }
        
        if (millis() - mn_auto_mode_switch_time >= copter.g2.auto_mode_switch_time_to_alt_hold) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::SCRIPTING);
            mn_auto_mode_switch_engaged = false;
        }  

    } else if (in_althold) {
        if (!confidence_ok) {
            mn_auto_mode_switch_engaged = false;
            return;
        }

        if (!mn_auto_mode_switch_engaged) {
            mn_auto_mode_switch_engaged = true;
            mn_auto_mode_switch_time = millis();
        }
        
        if (millis() - mn_auto_mode_switch_time >= copter.g2.auto_mode_switch_time_to_loiter) {
            copter.set_mode(Mode::Number::LOITER, ModeReason::SCRIPTING);
            mn_auto_mode_switch_engaged = false;
        }  

    } else { // we should never be here, sanity check
        mn_auto_mode_switch_engaged = false;
    }
}