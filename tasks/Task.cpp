/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <motors_weg_cvw300/Driver.hpp>
#include <iodrivers_base/ConfigureGuard.hpp>

using namespace std;
using namespace base;
using namespace motors_weg_cvw300;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    _modbus_interframe_delay.set(base::Time::fromMilliseconds(20));
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    unique_ptr<Driver> driver(new Driver(_address.get()));
    // Un-configure the device driver if the configure fails.
    // You MUST call guard.commit() once the driver is fully
    // functional (usually before the configureHook's "return true;"
    iodrivers_base::ConfigureGuard guard(this);
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());
    setDriver(driver.get());

    // This is MANDATORY and MUST be called after the setDriver but before you do
    // anything with the driver
    if (!TaskBase::configureHook())
        return false;

    driver->setInterframeDelay(_modbus_interframe_delay.get());

    driver->readMotorRatings();
    driver->disable();
    driver->prepare();
    auto wd = _watchdog.get();
    driver->writeSerialWatchdog(wd.timeout, wd.action);
    driver->writeControlType(_control_type.get());

    auto limits = _limits.get();
    driver->writeJointLimits(limits.elements.at(0));
    driver->writeRampConfiguration(_ramps.get());

    m_cmd_in.elements.resize(1);
    m_sample.elements.resize(1);

    m_driver = move(driver);
    guard.commit();
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    m_driver->enable();
    m_last_temperature_update = Time();
    return true;
}
void Task::updateHook()
{
    while (_cmd_in.read(m_cmd_in) == RTT::NewData) {
        if (m_cmd_in.elements.size() != 1) {
            exception(INVALID_COMMAND_SIZE);
            return;
        }

        auto joint = m_cmd_in.elements.at(0);
        if (!joint.isSpeed()) {
            exception(INVALID_COMMAND_PARAMETER);
        }

        m_driver->writeSpeedCommand(joint.speed);
    }

    auto now = Time::now();
    CurrentState state = m_driver->readCurrentState();
    m_sample.time = now;
    m_sample.elements[0] = state.motor;
    _joint_samples.write(m_sample);

    InverterState state_out;
    state_out.time = now;
    state_out.battery_voltage = state.battery_voltage;
    state_out.inverter_output_voltage = state.inverter_output_voltage;
    state_out.inverter_output_frequency = state.inverter_output_frequency;
    state_out.inverter_status = state.inverter_status;
    _inverter_state.write(state_out);

    if (now - m_last_temperature_update > _temperature_period.get()) {
        _temperatures.write(m_driver->readTemperatures());
        m_last_temperature_update = now;
    }

    if (state.inverter_status == STATUS_FAULT) {
        return exception(CONTROLLER_FAULT);
    }
    else if (state.inverter_status == STATUS_UNDERVOLTAGE) {
        return exception(CONTROLLER_UNDER_VOLTAGE);
    }

    TaskBase::updateHook();
}
void Task::processIO() {}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    // Delete the driver AFTER calling TaskBase::configureHook, as the latter
    // detaches the driver from the oroGen I/O
    m_driver.release();
}
