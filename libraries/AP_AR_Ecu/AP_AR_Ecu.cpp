#include "AP_AR_Ecu.h"
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_AR_Ecu::var_info[] = {

    AP_GROUPINFO("FUEL_K", 0, AP_AR_Ecu, _fuel_ecuation_k, 1.05833f),

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

void AP_AR_Ecu::set_current_fuel(float fuel) { 

    if (!_initialised) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Ecu Driver not initialized");
        return;
    }

    _fuel_remaining = fuel;
}

// this will be sent managed by GCS mavlink stream rates
void AP_AR_Ecu::send_mavlink_message_ecu(const mavlink_channel_t chan) {
 
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

    if (!_initialised) {
        return;
    }
    
    // get time delta
    uint32_t tnow = AP_HAL::micros();
    float dt = tnow - _last_time_micros;

    // prepare for ecuation
    float pulsewidths = _pulseWidth1 * 0.000001f;
    float rps = _rpm / 60;

     // constant * pulsewidth1 * rpm / ( 1000 * 1000 * 60 )
    float fuel_rate = _fuel_ecuation_k * pulsewidths * rps; // ml/s

    //  * 3600 (conversion to hours)
    //  3600 * 0.001 is 3.6 (conversion to liters)
    _fuel_instant = fuel_rate * 3.6f;

    if (_last_time_micros != 0 && dt < 2000000.0f) {

        // fuel_rate * dt(s) is consumption in last interval, in ml
        float fuel_acum = fuel_rate * dt * 0.000000001f; // liters

        // Those are liters
        _fuel_consumed += fuel_acum;
        _fuel_remaining -= fuel_acum;
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
