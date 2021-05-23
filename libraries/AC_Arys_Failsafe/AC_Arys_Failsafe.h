#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define FS_DEFAULTS 0

class AC_Arys_Failsafe
{
public:
    AC_Arys_Failsafe();

    void init();

    void update();

    MAV_RESULT handle_send_fs_coords_to_gcs();

    void save_failsafe_status(int32_t latitude, int32_t longitude, int32_t altitude, uint8_t sprayerstatus);

    void log_failsafe_status();

    bool save_coords_on_rtl_and_land() { return _failsafe_save_rtl_land; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:


    AP_Int32 _failsafe_latitude;
    AP_Int32 _failsafe_longitude;
    AP_Int32 _failsafe_altitude;
    AP_Int32 _failsafe_sprayer_status;
    AP_Int8  _failsafe_int_point_enabled;
    AP_Int8  _failsafe_save_rtl_land;
};
