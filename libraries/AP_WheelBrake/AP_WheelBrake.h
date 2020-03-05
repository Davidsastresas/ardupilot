/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_Param/AP_Param.h>

class AP_WheelBrake_Backend;

class AP_WheelBrake {
public:
    AP_WheelBrake();

    AP_WheelBrake(const AP_WheelBrake &other) = delete;
    AP_WheelBrake &operator=(const AP_WheelBrake&) = delete;

    static AP_WheelBrake *get_singleton();
    static AP_WheelBrake *_singleton;

    // indicate whether this module is enabled or not
    bool enabled() const { return _enabled; }

    // initialise the gripper
    void init();

    // grab - move the servo to the grab position
    void brake();

    // release - move the servo output to the release position
    void release();

    // released - returns true if currently in released position
    bool braked() const;

    // grabbed - returns true if currently in grabbed position
    bool released() const;

    // update - should be called at at least 10hz
    void update();

    // valid - returns true if we have a gripper and it should work
    bool valid() const;

    static const struct AP_Param::GroupInfo        var_info[];

    // parameters
    AP_Int8     _enabled;               // grabber enable/disable

    struct Backend_Config {
        AP_Int8     type;                    // grabber type (e.g. EPM or servo)
        AP_Int16    brake_pwm;               // PWM value sent to wheelbrake to initiate braking
        AP_Int16    release_pwm;             // PWM value sent to wheelbrake to release 
    } config;

private:

    AP_WheelBrake_Backend *backend;
};

namespace AP {
    AP_WheelBrake *wheelbrake();
};
