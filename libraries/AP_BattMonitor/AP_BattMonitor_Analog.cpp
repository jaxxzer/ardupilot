#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_Analog::AP_BattMonitor_Analog(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state) :
    AP_BattMonitor_Backend(mon, instance, mon_state)
{
    _volt_pin_analog_source = hal.analogin->channel(mon._volt_pin[instance]);
    _curr_pin_analog_source = hal.analogin->channel(mon._curr_pin[instance]);

    // always healthy
    _state.healthy = true;
}

void
AP_BattMonitor_Analog::init() {
    const uint8_t num_entries = 15;

    const struct {
        float voltage;
        uint8_t percent;
    } lookup[num_entries] = {
        {3.17, 0},
        {3.68, 5},
        {3.70, 10},
        {3.74, 15},
        {3.77, 25},
        {3.80, 45},
        {3.87, 55},
        {3.97, 79},
        {4.01, 84},
        {4.04, 88},
        {4.07, 91},
        {4.09, 94},
        {4.12, 96},
        {4.15, 98},
        {4.17, 100}
    };

    float voltage = 0;

    const uint8_t num_samples = 5;
    for (uint8_t i = 0; i < num_samples; i++) {
        hal.scheduler->delay(2);
        voltage += _volt_pin_analog_source->voltage_latest() * _mon._volt_multiplier[_state.instance];
    }

    voltage = voltage/num_samples;

    uint8_t cells = 0;

    // determine the number of cells
    // lower than 21.5 V on a 6s will be seen as a fully charged 5S (not good)
    for (uint8_t i = 0; i <= 6; i++) {
        if (voltage < i * 4.3f) {
            cells = i;
            break;
        }
    }

    if (!cells) {
        return;
    }

    for (uint8_t i = 0; i < num_entries ; i++) {
        if (voltage/cells > lookup[i].voltage) {
            continue;
        }
        _state.current_total_mah += (1.0f - (lookup[i].percent * 0.01f)) * _mon.pack_capacity_mah(_state.instance);
        break;
    }
}

// read - read the voltage and current
void
AP_BattMonitor_Analog::read()
{
    // this copes with changing the pin at runtime
    _volt_pin_analog_source->set_pin(_mon._volt_pin[_state.instance]);

    // get voltage
    _state.voltage = _volt_pin_analog_source->voltage_average() * _mon._volt_multiplier[_state.instance];

    // read current
    if (_mon.has_current(_state.instance)) {
        // calculate time since last current read
        uint32_t tnow = AP_HAL::micros();
        float dt = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _curr_pin_analog_source->set_pin(_mon._curr_pin[_state.instance]);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average()-_mon._curr_amp_offset[_state.instance])*_mon._curr_amp_per_volt[_state.instance];

        // update total current drawn since startup
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            _state.current_total_mah += _state.current_amps * dt * 0.0000002778f;
        }

        // record time
        _state.last_time_micros = tnow;
    }
}
