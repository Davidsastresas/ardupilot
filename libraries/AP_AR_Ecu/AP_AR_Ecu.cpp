#include "AP_AR_Ecu.h"
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_AR_Ecu::var_info[] = {

    AP_GROUPINFO("STATIC_FRATE", 1, AP_AR_Ecu, _static_inj_flow_rate, 1.0f),

    AP_GROUPINFO("FUEL_DENS", 2, AP_AR_Ecu, _fuel_density, 1.0f),

    AP_GROUPINFO("FUEL_DENS_K", 3, AP_AR_Ecu, _fuel_density_k, 1.0f),

    AP_GROUPEND
};

AP_AR_Ecu::AP_AR_Ecu(void) {

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ICEngine must be singleton");
    }
    _singleton = this;
}

void AP_AR_Ecu::init(void) {
    
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for ecu protocol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AR_Ecu, 0))) {
        _initialised = true;

        gcs().send_text(MAV_SEVERITY_INFO, "Ecu Driver initialized");
    }
}

// this will be sent managed by GCS mavlink stream rates
void AP_AR_Ecu::send_mavlink_message_ecu(const mavlink_channel_t chan) {
 
    gcs().send_text(MAV_SEVERITY_INFO, "Sending stuff");
    mavlink_msg_ar_efi_telemetry_send(
        chan,
        _coolant,
        _rpm,
        _barometer,
        _tps,
        _batteryVoltage,
        _fuel_instant,
        _fuel_consumed,
        _fuel_remaining
    );
}

void AP_AR_Ecu::update(void) {

    if (!_initialised) {
        return;
    }

    // read the info from the last request
    read_incoming();

    // calculate composed fuel consumption stuff
    calc_fuel_consumption();

    // send request data command. It is 'a' 0x00 0x06
    send_command(CMD_READ_DATA);
    send_command(0x00);
    send_command(0x06);
}

void AP_AR_Ecu::read_incoming(void) {


    uint8_t data;
    int16_t numc;

    numc = _port->available();

    // not the best way of determining if it is healthy but the best we got so far
    if (numc < 110) {
        _healthy = false;
    } else {
        _healthy = true;
    }

    if (numc < 0 ){
        return;
    }

    _payload_counter = 0;

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();
        
        if (_payload_counter < sizeof(_buffer)) {
            _buffer[_payload_counter] = data;
        }

        // Redundant within the for limits, sanity check
        if (++_payload_counter == numc) {
            break;
        }
    }
    parse_body();
}

void AP_AR_Ecu::calc_fuel_consumption() {

    uint32_t tnow = AP_HAL::micros();
    float dt = tnow - _last_time_micros;

    float pulsewidth = _pulseWidth1 * 0.001;
    float inj_duty = ( pulsewidth * _rpm ) / 60;
    _fuel_instant = ( inj_duty * _static_inj_flow_rate * _fuel_density ) / _fuel_density_k;

    if (_last_time_micros != 0 && dt < 2000000.0f) {

        // .0002778 is 1/3600 (conversion to hours)
        float fuel_consumed = _fuel_instant * dt * 0.0000002778f;
        _fuel_consumed += fuel_consumed;
        _fuel_remaining -= fuel_consumed;
    }

    // record time
    _last_time_micros = tnow;
}

void AP_AR_Ecu::parse_body(void) {

    _pulseWidth1 = bswap_16_on_le(_buffer.telem_a.pulseWidth1);
    _coolant = bswap_16_on_le(_buffer.telem_a.coolant);
    _rpm = bswap_16_on_le(_buffer.telem_a.rpm); 
    _barometer = bswap_16_on_le(_buffer.telem_a.barometer); 
    _tps = bswap_16_on_le(_buffer.telem_a.tps);
    _batteryVoltage = bswap_16_on_le(_buffer.telem_a.batteryVoltage);
}

void AP_AR_Ecu::send_command(uint8_t cmd) {
    
    if (_port->txspace() < 1) {
        return;
    }
    _port->write( cmd );
}

// singleton instance. Should only ever be set in the constructor.
AP_AR_Ecu *AP_AR_Ecu::_singleton;
namespace AP {
AP_AR_Ecu *ar_ecu() {
        return AP_AR_Ecu::get_singleton();
    }
}
