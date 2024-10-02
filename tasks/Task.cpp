/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iodrivers_base/ConfigureGuard.hpp>
#include <motors_weg_cvw300/Driver.hpp>

using namespace std;
using namespace base;
using namespace motors_weg_cvw300;
using namespace control_base;

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
    if (!TaskBase::configureHook()) {
        return false;
    }

    auto motor_registers = _registers_configuration.get();
    for (auto& motor_register : motor_registers) {
        driver->writeSingleRegister<uint16_t>(motor_register.parameter,
            motor_register.value);
    }

    driver->setInterframeDelay(_modbus_interframe_delay.get());

    driver->readMotorRatings();
    driver->disable();
    driver->prepare();
    auto wd = _watchdog.get();
    driver->writeSerialWatchdog(wd.timeout, wd.action);
    m_cmd_timeout = wd.timeout;
    driver->writeControlType(_control_type.get());

    m_limits = _limits.get();
    if (m_limits.elements.empty()) {
        // Initialize speed limits as infinity
        JointLimits infinity;
        JointLimitRange range;
        range.Speed(-base::infinity<float>(), base::infinity<float>());
        infinity.elements.push_back(range);
        m_limits = infinity;
    }
    else {
        driver->writeJointLimits(m_limits.elements.at(0));
    }

    driver->writeRampConfiguration(_ramps.get());

    m_cmd_in.elements.resize(1);
    m_sample.elements.resize(1);

    m_inverted = _inverted.get();

    m_driver = move(driver);
    guard.commit();
    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook()) {
        return false;
    }

    writeSpeedCommand(0);
    m_driver->enable();
    m_last_temperature_update = Time();

    publishFault();
    return true;
}
bool Task::commandTimedOut() const
{
    return !m_cmd_deadline.isNull() && (base::Time::now() > m_cmd_deadline);
}
void Task::writeSpeedCommand(float cmd)
{
    m_driver->writeSpeedCommand(m_inverted ? -cmd : cmd);
    if (!m_cmd_timeout.isNull()) {
        m_cmd_deadline = base::Time::now() + m_cmd_timeout;
    }
}
bool Task::checkSpeedSaturation(base::commands::Joints const& cmd)
{
    return cmd.elements[0].speed >= m_limits.elements[0].max.speed ||
           cmd.elements[0].speed <= m_limits.elements[0].min.speed;
}
CurrentState Task::readAndPublishControllerStates()
{
    auto now = Time::now();
    CurrentState state = m_driver->readCurrentState();
    m_sample.time = now;
    auto joint_state = state.motor;
    if (m_inverted) {
        joint_state.speed = -joint_state.speed;
        joint_state.effort = -joint_state.effort;
        joint_state.raw = -joint_state.raw;
    }
    m_sample.elements[0] = joint_state;
    _joint_samples.write(m_sample);

    InverterState state_out;
    state_out.time = now;
    state_out.battery_voltage = state.battery_voltage;
    state_out.motor_overload_ratio = state.motor_overload_ratio;
    state_out.inverter_output_voltage = state.inverter_output_voltage;
    state_out.inverter_output_frequency = state.inverter_output_frequency;
    state_out.inverter_status = state.inverter_status;
    _inverter_state.write(state_out);

    int current_alarm = m_driver->readCurrentAlarm();
    AlarmState alarmState;
    alarmState.time = Time::now();
    alarmState.current_alarm = current_alarm;
    _alarm_state.write(alarmState);

    if (now - m_last_temperature_update > _temperature_period.get()) {
        _temperatures.write(m_driver->readTemperatures());
        m_last_temperature_update = now;
    }
    return state;
}
void Task::evaluateInverterStatus(InverterStatus const& inverter_status)
{
    if (inverter_status == STATUS_FAULT) {
        error(CONTROLLER_FAULT);
    }
    else if (inverter_status == STATUS_UNDERVOLTAGE) {
        error(CONTROLLER_UNDER_VOLTAGE);
    }
}
void Task::publishFault()
{
    auto fault_state = m_driver->readFaultState();
    _fault_state.write(fault_state);
}
void Task::updateHook()
{
    while (_cmd_in.read(m_cmd_in) == RTT::NewData) {
        if (m_cmd_in.elements.size() != 1) {
            return exception(INVALID_COMMAND_SIZE);
        }

        auto joint = m_cmd_in.elements.at(0);
        if (!joint.isSpeed()) {
            return exception(INVALID_COMMAND_PARAMETER);
        }

        writeSpeedCommand(joint.speed);

        SaturationSignal saturation_signal;
        saturation_signal.value = checkSpeedSaturation(m_cmd_in);
        saturation_signal.time = m_cmd_in.time;
        _saturation_signal.write(saturation_signal);
    }

    if (commandTimedOut()) {
        writeSpeedCommand(0);
    }

    auto state = readAndPublishControllerStates();
    evaluateInverterStatus(state.inverter_status);

    TaskBase::updateHook();
}
void Task::processIO()
{
}
void Task::errorHook()
{
    TaskBase::errorHook();

    publishFault();

    // Try to reset the faults
    m_driver->resetFault();
    // Verify if the controller status is not in FAULT state
    auto state = readAndPublishControllerStates();
    if (state.inverter_status != STATUS_FAULT &&
        state.inverter_status != STATUS_UNDERVOLTAGE) {
        m_driver->enable();
        recover();
    }

    if (_cmd_in.read(m_cmd_in) == RTT::NewData) {
        evaluateInverterStatus(state.inverter_status);
    }
}
void Task::stopHook()
{
    m_driver->disable();
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    // Delete the driver AFTER calling TaskBase::configureHook, as the latter
    // detaches the driver from the oroGen I/O
    m_driver.release();
}
