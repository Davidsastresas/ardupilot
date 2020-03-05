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

#include <AP_WheelBrake/AP_WheelBrake.h>

class AP_WheelBrake_Backend {
public:
    AP_WheelBrake_Backend(struct AP_WheelBrake::Backend_Config &_config) :
        config(_config) { }

    // initialise the gripper backend
    void init();

    // update - should be called at at least 10hz
    void update();

    // brake - move the servo to the brake position
    virtual void brake() = 0;

    // release - move the servo output to the release position
    virtual void release() = 0;

    // valid - returns true if the backend should be working
    virtual bool valid() const { return true; };

    // released - returns true if currently in released position
    virtual bool released() const = 0;

    // braked - returns true if currently in braked position
    virtual bool braked() const = 0;

    // type-specific intiailisations:
    virtual void init_wheelbrake() = 0;

    // type-specific periodic updates:
    virtual void update_wheelbrake() { };

protected:

    struct AP_WheelBrake::Backend_Config &config;
};
