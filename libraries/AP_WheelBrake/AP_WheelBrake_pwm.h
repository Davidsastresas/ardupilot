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

#include <AP_WheelBrake/AP_WheelBrake_Backend.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_WheelBrake_pwm : public AP_WheelBrake_Backend {
public:

    AP_WheelBrake_pwm(struct AP_WheelBrake::Backend_Config &_config) :
        AP_WheelBrake_Backend(_config) { }

    // grab - move the pwm to the grab position
    void brake() override;

    // release - move the pwm output to the release position
    void release() override;

    // grabbed - returns true if wheelbrake in grabbed state
    bool braked() const override;

    // released - returns true if wheelbrake in released state
    bool released() const override;

    // valid - returns true if the backend should be working
    bool valid() const override;

protected:

    // type-specific intiailisations:
    void init_wheelbrake() override;

    // type-specific periodic updates:
    void update_wheelbrake() override;

private:

    bool has_state_pwm(const uint16_t pwm) const;
};
