#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AC_Arys_Failsafe.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

const AP_Param::GroupInfo AC_Arys_Failsafe::var_info[] = {

    AP_GROUPINFO("LAT", 0, AC_Arys_Failsafe, _failsafe_latitude, FS_DEFAULTS),

    AP_GROUPINFO("LON", 1, AC_Arys_Failsafe, _failsafe_longitude, FS_DEFAULTS),

    AP_GROUPINFO("ALT", 2, AC_Arys_Failsafe, _failsafe_altitude, FS_DEFAULTS),

    AP_GROUPINFO("SST", 3, AC_Arys_Failsafe, _failsafe_sprayer_status, FS_DEFAULTS),

    AP_GROUPINFO("ENA", 4, AC_Arys_Failsafe, _failsafe_int_point_enabled, 0),

    AP_GROUPEND
};

AC_Arys_Failsafe::AC_Arys_Failsafe() 
{

}

void AC_Arys_Failsafe::init() 
{

}

void AC_Arys_Failsafe::update()
{

}

MAV_RESULT AC_Arys_Failsafe::handle_send_fs_coords_to_gcs()
{
    // sanity checks on our failsafe data
    // reset invalid variables and return MAV_RESULT_FAILED in case of invalid data
    
    // Maybe the parameters reset to default (int_32 default parameters issue)
    if ((_failsafe_latitude == 0) && (_failsafe_longitude == 0)) {
        gcs().send_text(MAV_SEVERITY_INFO, "failed cause lat/long");
        _failsafe_latitude = INT32_MAX;
        _failsafe_longitude = INT32_MAX;
        
        return MAV_RESULT_FAILED;

    // do not accept zero height nor negative values
    } else if (_failsafe_altitude <= 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "failed cause alt");
        _failsafe_altitude = INT32_MAX;
        return MAV_RESULT_FAILED;

    // do not accept values different from 0 and 1
    } else if ( (_failsafe_sprayer_status != 0) && (_failsafe_sprayer_status != 1) ) {
        gcs().send_text(MAV_SEVERITY_INFO, "failed cause sprayer status");
        _failsafe_sprayer_status = INT32_MAX;
        return MAV_RESULT_FAILED;
    
    // safe to send FS coordinates info
    } else {

        mavlink_named_value_int_t packetnew {};
        packetnew.time_boot_ms = AP_HAL::millis();
        
        char name_lat[10] = "FS_LAT";
        packetnew.value = _failsafe_latitude;
        memcpy(packetnew.name, name_lat, MIN(strlen(name_lat), (uint8_t)MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN));
        gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_INT,
                                (const char *)&packetnew);
        char name_long[10] = "FS_LON";
        packetnew.value = _failsafe_longitude;
        memcpy(packetnew.name, name_long, MIN(strlen(name_long), (uint8_t)MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN));
        gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_INT,
                                (const char *)&packetnew);
        char name_alt[10] = "FS_ALT";
        packetnew.value = _failsafe_altitude;
        memcpy(packetnew.name, name_alt, MIN(strlen(name_alt), (uint8_t)MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN));
        gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_INT,
                                (const char *)&packetnew);
        char name_spr[10] = "FS_SPR";
        packetnew.value = _failsafe_sprayer_status;
        memcpy(packetnew.name, name_spr, MIN(strlen(name_spr), (uint8_t)MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN));
        gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_INT,
                                (const char *)&packetnew);
        
        // set default values for the variables since we have already sent the values to the gcs
        _failsafe_latitude = INT32_MAX;
        _failsafe_longitude = INT32_MAX;
        _failsafe_altitude = INT32_MAX;    
        _failsafe_sprayer_status = INT32_MAX;              
        return MAV_RESULT_ACCEPTED;
    }
}

void AC_Arys_Failsafe::save_failsafe_status(int32_t latitude, int32_t longitude, int32_t altitude, uint8_t sprayerstatus)
{
    _failsafe_latitude.set_and_save(latitude);
    _failsafe_longitude.set_and_save(longitude);
    _failsafe_altitude.set_and_save(altitude);
    _failsafe_sprayer_status.set_and_save(sprayerstatus);
}

void AC_Arys_Failsafe::log_failsafe_status()
{
    // missing enable parameter
    AP::logger().Write("ARFS", "TimeUS,lat,lon,alt,sprs", "Qiiib",
        AP_HAL::micros64(),
        _failsafe_latitude,
        _failsafe_longitude,
        _failsafe_altitude,
        _failsafe_sprayer_status);
}