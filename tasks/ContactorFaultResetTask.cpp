/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ContactorFaultResetTask.hpp"

using namespace motors_weg_cvw300;
using namespace linux_gpios;

ContactorFaultResetTask::ContactorFaultResetTask(std::string const& name)
    : ContactorFaultResetTaskBase(name)
{
}

ContactorFaultResetTask::~ContactorFaultResetTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ContactorFaultResetTask.hpp for more detailed
// documentation about them.

bool ContactorFaultResetTask::configureHook()
{
    if (!ContactorFaultResetTaskBase::configureHook())
        return false;

    m_propulsion_disabled_duration = _propulsion_disabled_duration.get();
    m_number_of_attempts_to_change_controller_failure =
        _number_of_attempts_to_change_controller_failure.get();
    return true;
}
bool ContactorFaultResetTask::startHook()
{
    if (!ContactorFaultResetTaskBase::startHook())
        return false;
    m_number_of_contactor_faults = 0;
    return true;
}
void ContactorFaultResetTask::updateHook()
{
    ContactorFaultResetTaskBase::updateHook();

    GPIOState current_propulsion_enable_state;
    if (_propulsion_enable_in.read(current_propulsion_enable_state) == RTT::NoData) {
        return;
    }

    FaultState port_fault_state, starboard_fault_state;
    if (_port_fault_state.read(port_fault_state) == RTT::NewData ||
        _starboard_fault_state.read(starboard_fault_state) == RTT::NewData) {
        // F185 - Contactor Fault
        if ((port_fault_state.current_fault == 185 ||
                starboard_fault_state.current_fault == 185) &&
            current_propulsion_enable_state.states[0].data == true) {
            m_number_of_contactor_faults++;
            if (m_number_of_contactor_faults <=
                m_number_of_attempts_to_change_controller_failure) {
                // Try to change the controller fault triggering the external fault F91
                _propulsion_enable_out.write(GPIO(false));
                sleep(m_propulsion_disabled_duration.toSeconds());
                _propulsion_enable_out.write(GPIO(true));
            }
            else {
                // Hard reset
                _motor_controller_fault_reset_enable.write(GPIO(false));
                _motor_controller_fault_reset_enable.write(GPIO(true));
                _motor_controller_fault_reset_enable.write(GPIO(false));
            }
        }
        else if ((port_fault_state.current_fault == 0 &&
                     starboard_fault_state.current_fault == 0) ||
                 current_propulsion_enable_state.states[0].data == false) {
            m_number_of_contactor_faults = 0;
        }
    }
}

GPIOState ContactorFaultResetTask::GPIO(bool flag)
{
    auto now = base::Time::now();
    GPIOState gpio;
    gpio.time = now;
    gpio.states[0].time = now;
    gpio.states[0].data = flag;
    return gpio;
}

void ContactorFaultResetTask::errorHook()
{
    ContactorFaultResetTaskBase::errorHook();
}
void ContactorFaultResetTask::stopHook()
{
    ContactorFaultResetTaskBase::stopHook();
}
void ContactorFaultResetTask::cleanupHook()
{
    ContactorFaultResetTaskBase::cleanupHook();
}
