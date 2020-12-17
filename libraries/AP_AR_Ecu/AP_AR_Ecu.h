// This class implements the serial protocol for HFE international ECU system as
// described in document HFEDCN0191_REV_F.pdf

// As for fuel consumption, there are a few options for you.  Please note that both of these options are still providing estimates and are not completely accurate.  We have been working this problem for about a year and are presenting a fuel consumed variable in our next iteration of ECUs. 
 
// Option 1)
// Average and estimation of the fuel consumption
// Simpler code
 
// Code example: 
 
// fuel_instant = ((InjDuty) * static_Inj_Flow_rate * Density_of_fuel)/600000;     // calculate fuel flow
// InjDuty = (pulseWidth1 * RPM) / 60
// pulseWidth1 = (((serbuff[2]<<8) | serbuff[3]) / 1000.0);       // pulseWidth1
// RPM = (serbuff[6]<<8) | (serbuff[7]);  
// Static_Inj_Flow_rate = Flow rate of your injector (I will have that for you tomorrow)
// Density_of_fuel = density of the fuel you’re using
// 600000 is dependent upon the Density units.
 
// This option is giving an estimate of the fuel consumed based on the static flow rate of the injector.  For more precision on the fuel consumption, you will need to incorporate option 2.

#pragma once

#include <AP_HAL/AP_HAL.h>

// just for debugging
#include <GCS_MAVLink/GCS.h>

//definition of commands
#define CMD_READ_DATA 'a'

class AP_AR_Ecu {
public:

    AP_AR_Ecu();

    static const struct AP_Param::GroupInfo var_info[];

    void update(void);

    void init();

    bool healthy() const { return _healthy; } 

    void send_mavlink_message_ecu(const mavlink_channel_t chan);

    uint16_t get_rpm() const { return _rpm; }

    static AP_AR_Ecu *get_singleton() { return _singleton; }

private:

    static AP_AR_Ecu *_singleton;

    void parse_body();

    void read_incoming();

    void send_command(uint8_t cmd);

    void calc_fuel_consumption();

    // This is the response as per the documentation, first 112 bits of table 1.1
    struct PACKED ECU_telemetry_a {
        // variable                index  Description                                                              size  sign

        uint16_t seconds;         // 0   -> Seconds ECU has been on s 1 0                                             2    N 
        uint16_t pulseWidth1;     // 2   -> Main pulsewidth Injector 1 ms .000666 0                                   2    N 
        uint16_t pulseWidth2;     // 4   -> Main pulsewidth Injector 2 ms .000666 0                                   2    N 
        uint16_t rpm;             // 6   -> Engine RPM RPM 1 0                                                        2    N 
        int16_t advance;          // 8   -> Final ignition spark advance deg BTDC 10 0                                2    Y 
        uint8_t squirt;           // 10  -> Bitfield of batch fire injector events - 1 0                              1    N 
        uint8_t engine;           // 11  -> Bitfield of engine status - 1 0                                           1    N 
        uint8_t afrtgt1;          // 12  -> Bank 1 AFR target AFR 10 0                                                1    N 
        uint8_t afrtgt2;          // 13  -> Bank 2 AFR target AFR 10 0                                                1    N 
        uint8_t wbo2_en1;         // 14  -> not used* - 1 0                                                           1    N 
        uint8_t wbo2_en2;         // 15  -> not used* - 1 0                                                           1    N 
        int16_t barometer;        // 16  -> Barometric pressure kPa 10 0                                              2    Y 
        int16_t map;              // 18  -> Manifold air pressure kPa 10 0                                            2    Y 
        int16_t mat;              // 20  -> Manifold air temperature deg F 10 0                                       2    Y 
        int16_t coolant;          // 22  -> Coolant temperature deg F 10 0                                            2    Y 
        int16_t tps;              // 24  -> Throttle position % 10 0                                                  2    Y 
        int16_t batteryVoltage;   // 26  -> Battery voltage V 10 0                                                    2    Y 
        int16_t afr1;             // 28  -> AFR1 AFR 10 0                                                             2    Y 
        int16_t afr2;             // 30  -> AFR2 AFR 10 0                                                             2    Y 
        int16_t knock;            // 32  -> Indication of knock input % 10 0                                          2    Y 
        int16_t egocor1;          // 34  -> EGO bank 1 correction % 10 0                                              2    Y 
        int16_t egocor2;          // 36  -> EGO bank 2 correction % 10 0                                              2    Y 
        int16_t aircor;           // 38  -> Air density correction % 10 0                                             2    Y 
        int16_t warmcor;          // 40  -> Warmup correction % 10 0                                                  2    Y 
        int16_t accelEnrich;      // 42  -> TPS-based acceleration % 10 0                                             2    Y 
        int16_t tpsfuelcut;       // 44  -> TPS-based fuel cut % 10 0                                                 2    Y 
        int16_t baroCorrection;   // 46  -> Barometric fuel correction % 10 0                                         2    Y 
        int16_t gammaEnrich;      // 48  -> Total fuel correction % 10 0                                              2    Y 
        int16_t ve1;              // 50  -> VE value table/bank 1 % 10 0                                              2    Y 
        int16_t ve2;              // 52  -> VE value table/bank 2 % 10 0                                              2    Y 
        int16_t iacstep;          // 54  -> Stepper idle step number or PWM idle value duty step duty% 1 1000 0 0     2    Y 
        int16_t cold_adv_deg;     // 56  -> Cold advance deg 10 0                                                     2    Y 
        int16_t TPSdot;           // 58  -> Rate of change of TPS %/s 10 0                                            2    Y 
        int16_t MAPdot;           // 60  -> Rate of change of MAP kPa/s 10 0                                          2    Y 
        int16_t dwell;            // 62  -> Main ignition dwell ms 10 0                                               2    Y 
        int16_t MAF;              // 64  -> Mass Air Flow (Scaling depend on range, 650g/s shown) g/s 100 0           2    Y 
        uint8_t fuelload;         // 66  -> 'Load' used for fuel table lookup e.g. equals MAP in SpeedDensity % 10 0  1    N 
        int16_t fuelcor;          // 68  -> Adjustment to fuel from Flex % 1 0                                        2    Y 
        uint8_t portStatus;       // 70  -> On/off outputs status bits. - 1 0                                         1    N 
        uint8_t knockRetard;      // 71  -> Ignition retard due to knock deg 10 0                                     1    N 
        int16_t EAEfcor1;         // 72  -> Fuel correction due to XTau or EAE 1 % 1 0                                2    Y 
        int16_t egoV1;            // 74  -> Voltage from O2#1 V 100 0                                                 2    Y 
        int16_t egoV2;            // 76  -> Voltage from O2#2 V 100 0                                                 2    Y 
        uint16_t status1;         // 78  -> Internal status flags - 1 0                                               2    N 
        uint16_t status2;         // 79  -> Internal status flags - 1 0                                               2    N 
        uint16_t status3;         // 80  -> Internal status flags - 1 0                                               2    N 
        uint16_t status4;         // 81  -> Internal status flags - 1 0                                               2    N 
        uint16_t looptime;        // 82  -> Main loop time us .6667 0                                                 2    N 
        uint16_t status5;         // 84  -> Internal status flags - 1 0                                               2    N 
        uint16_t tpsADC;          // 86  -> Used for TPS calculation ADC 1 0                                          2    N 
        int16_t fuelload2;        // 88  -> Secondary fuel load % .1 0                                                2    Y 
        int16_t ignload;          // 90  -> Ignition load % .1 0                                                      2    Y 
        int16_t ignload2;         // 92  -> Secondary ignition load % .1 0                                            2    Y 
        uint8_t synccnt;          // 94  -> Sync loss count - 1 0                                                     1    N 
        int8_t timing_err;        // 95  -> Accuracy of timing prediction % .1 0                                      1    Y 
        int32_t delta;            // 96  -> Time between triggers                                                     4    Y 
        uint32_t wallfuel1;       // 100 -> wall wettin uS 1 0 1                                                      4    N 
        uint16_t gpioadc0;        // 104 -> CAN BUSGPIO - 1 0 1                                                       2    N 
        uint16_t gpioadc1;        // 106 -> CAN BUSGPIO - 1 0 1                                                       2    N 
        uint16_t gpioadc2;        // 108 -> CAN BUSGPIO - 1 0 1                                                       2    N 
    };

    union PACKED  ecu_union {
        DEFINE_BYTE_ARRAY_METHODS
        ECU_telemetry_a telem_a;
    } _buffer;

    // variables for serial protocol, etc
    AP_HAL::UARTDriver *_port;
    bool _initialised = false;
    uint8_t _payload_counter;

    // variables taken from the ecu telemetry
    int16_t _coolant;
    uint16_t _rpm;
    int16_t _barometer;
    int16_t _tps;
    int16_t _batteryVoltage;
    float _fuel_instant;
    float _fuel_consumed;
    float _fuel_remaining;
    uint16_t _pulseWidth1;

    // parameters
    AP_Float _static_inj_flow_rate;
    AP_Float _fuel_density;
    AP_Float _fuel_density_k;

    // variables for fuel consumption calculation
    uint32_t _last_time_micros = 0;
    bool _healthy = false;
};

namespace AP {
    AP_AR_Ecu *ar_ecu();
};