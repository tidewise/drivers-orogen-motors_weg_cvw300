#ifndef motors_weg_cvw300_TYPES_HPP
#define motors_weg_cvw300_TYPES_HPP

#include <base/Float.hpp>
#include <base/Time.hpp>
#include <motors_weg_cvw300/Configuration.hpp>
#include <motors_weg_cvw300/CurrentState.hpp>
#include <motors_weg_cvw300/FaultState.hpp>
#include <motors_weg_cvw300/InverterStatus.hpp>

namespace motors_weg_cvw300 {
    namespace configuration {
        struct SerialWatchdog {
            base::Time timeout = base::Time::fromSeconds(1);
            configuration::CommunicationErrorAction action =
                configuration::STOP_WITH_RAMP;
        };

        struct MotorRegister {
            uint parameter;
            uint16_t value;
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

    struct AlarmState {
        base::Time time;
        int current_alarm;
    };

    /**
     * @brief The probabilities associated to the simulation of the contactor fault
     *
     * Each value must be between 0 and 1
     */
    struct ContactorFaultProbabilities {
        /**
         * @brief The probability to trigger a contactor fault
         */
        double trigger = base::unknown<double>();
        /**
         * @brief The probability of the external fault break the current contactor fault
         */
        double break_on_external_fault = base::unknown<double>();
    };

    enum Fault {
        NO_FAULT = 0,
        EXTERNAL_FAULT = 91,
        CONTACTOR_FAULT = 185
    };

}

#endif
