#ifndef motors_weg_cvw300_TYPES_HPP
#define motors_weg_cvw300_TYPES_HPP

#include <base/Time.hpp>
#include <motors_weg_cvw300/Configuration.hpp>
#include <motors_weg_cvw300/InverterStatus.hpp>

namespace motors_weg_cvw300 {
    namespace configuration {
        struct SerialWatchdog {
            base::Time timeout = base::Time::fromSeconds(1);
            configuration::CommunicationErrorAction action =
                configuration::STOP_WITH_RAMP;
        };
    }

    struct InverterState {
        base::Time time;

        float battery_voltage;
        float inverter_output_voltage;
        float inverter_output_frequency;
        float motor_overload_ratio;

        InverterStatus inverter_status;
    };
}

#endif

