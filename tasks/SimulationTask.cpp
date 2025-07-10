/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SimulationTask.hpp"

#include <control_base/SaturationSignal.hpp>

using namespace motors_weg_cvw300;
using namespace std;

base::samples::Joints speedCommand(float cmd, std::string const& joint_name);
control_base::SaturationSignal saturationSignal(bool saturation);

SimulationTask::SimulationTask(std::string const& name)
    : SimulationTaskBase(name)
{
    std::mt19937 m_distribution_generator(std::random_device{}());
}

SimulationTask::~SimulationTask()
{
}

bool SimulationTask::configureHook()
{
    if (!SimulationTaskBase::configureHook()) {
        return false;
    }

    m_limits = _limits.get();
    m_watchdog_timeout = _watchdog_timeout.get();
    m_edge_triggered_fault_state_output = _edge_triggered_fault_state_output.get();
    if (_joint_name.get() == "") {
        throw std::runtime_error("joint_name property must be set");
    }
    m_joint_name = _joint_name.get();
    m_zero_command = speedCommand(0, m_joint_name);
    m_contactor_fault_probabilities = _contactor_fault_probabilities.get();

    return true;
}

base::samples::Joints speedCommand(float cmd, std::string const& joint_name)
{
    base::samples::Joints speed_cmd;
    speed_cmd.elements.resize(1);
    speed_cmd.names.resize(1);

    speed_cmd.time = base::Time::now();
    speed_cmd.elements[0].setField(base::JointState::SPEED, cmd);
    speed_cmd.names[0] = joint_name;

    return speed_cmd;
}

bool SimulationTask::startHook()
{
    if (!SimulationTaskBase::startHook()) {
        return false;
    }
    m_current_fault_state = Fault::NO_FAULT;
    m_external_fault = false;

    updateWatchdog();
    writeCommandOut(zeroCommand());
    publishFault();
    return true;
}

void SimulationTask::updateWatchdog()
{
    if (m_watchdog_timeout.isNull()) {
        m_cmd_deadline = base::Time();
    }
    else {
        m_cmd_deadline = base::Time::now() + m_watchdog_timeout;
    }
}

void SimulationTask::writeCommandOut(base::samples::Joints const& cmd)
{
    if (inverterStatus() != InverterStatus::STATUS_FAULT) {
        _cmd_out.write(m_last_command_out = cmd);
    }
    else {
        _cmd_out.write(m_last_command_out = zeroCommand());
    }
}

base::samples::Joints const& SimulationTask::zeroCommand()
{
    m_zero_command.time = base::Time::now();
    return m_zero_command;
}

InverterStatus SimulationTask::inverterStatus() const
{
    if (m_current_fault_state == Fault::EXTERNAL_FAULT ||
        m_current_fault_state == Fault::CONTACTOR_FAULT) {
        return InverterStatus::STATUS_FAULT;
    }

    if (m_last_command_out.elements.size() && m_last_command_out.elements[0].speed != 0) {
        return InverterStatus::STATUS_RUN;
    }

    return InverterStatus::STATUS_READY;
}

void SimulationTask::publishFault()
{
    FaultState fault;
    fault.current_fault = m_current_fault_state;
    fault.fault_history[0] = fault.current_fault;

    fault.time = base::Time::now();
    _fault_state.write(fault);
}

void SimulationTask::updateHook()
{
    SimulationTaskBase::updateHook();

    base::samples::Joints cmd_in;
    if (_cmd_in.read(cmd_in) == RTT::NewData) {
        States cmd_exception;
        if (!validateCommand(cmd_in, cmd_exception)) {
            return exception(cmd_exception);
        }

        updateWatchdog();

        auto [saturated, cmd_out] = m_limits.saturate(cmd_in);
        cmd_out.time = base::Time::now();

        _saturation_signal.write(saturationSignal(saturated));
        writeCommandOut(cmd_out);
    }

    if (!m_cmd_deadline.isNull() && base::Time::now() > m_cmd_deadline) {
        writeCommandOut(zeroCommand());
    }

    if (!m_edge_triggered_fault_state_output) {
        publishFault();
    }
    readExternalFaultGPIOState();
    updateFaultState();
    readPortPowerDisableGPIOState();
    readStarboardPowerDisableGPIOState();

    InverterState state = currentState();
    _inverter_state.write(state);

    evaluateInverterStatus(state.inverter_status);
}

bool SimulationTask::triggerContactorFault()
{
    return rollProbability(m_contactor_fault_probabilities.trigger);
}

bool SimulationTask::rollProbability(double probability)
{
    bernoulli_distribution distribution(probability);
    return distribution(m_distribution_generator);
}

bool SimulationTask::validateCommand(base::samples::Joints cmd,
    SimulationTask::States& command_exception) const
{
    if (cmd.elements.size() != 1 || cmd.names.size() != 1) {
        command_exception = INVALID_COMMAND_SIZE;
        return false;
    }

    if (cmd.names[0] != m_joint_name) {
        command_exception = INVALID_JOINT_NAME;
        return false;
    }

    if (!cmd.elements[0].isSpeed()) {
        command_exception = INVALID_COMMAND_PARAMETER;
        return false;
    }

    return true;
}

control_base::SaturationSignal saturationSignal(bool saturation)
{
    control_base::SaturationSignal signal;
    signal.value = saturation;
    signal.time = base::Time::now();

    return signal;
}

void SimulationTask::readExternalFaultGPIOState()
{
    linux_gpios::GPIOState external_fault_gpio;
    if (_external_fault_gpio.read(external_fault_gpio) == RTT::NewData) {
        m_external_fault = !external_fault_gpio.states[0].data;
    }
}

void SimulationTask::updateFaultState()
{
    if (m_current_fault_state == Fault::CONTACTOR_FAULT && m_external_fault &&
        exitContactorFault()) {
        m_current_fault_state = Fault::EXTERNAL_FAULT;
        return;
    }
    if (!m_external_fault && m_current_fault_state == Fault::EXTERNAL_FAULT &&
        m_current_fault_state != Fault::CONTACTOR_FAULT) {
        m_current_fault_state = Fault::NO_FAULT;
        return;
    }
    if (triggerContactorFault()) {
        m_current_fault_state = Fault::CONTACTOR_FAULT;
        return;
    }
    if (m_external_fault) {
        m_current_fault_state = Fault::EXTERNAL_FAULT;
        return;
    }
    else {
        m_current_fault_state = Fault::NO_FAULT;
    }
}

bool SimulationTask::exitContactorFault()
{
    return rollProbability(m_contactor_fault_probabilities.break_on_external_fault);
}

void SimulationTask::readPortPowerDisableGPIOState()
{
    linux_gpios::GPIOState port_power_disable;
    if (_port_power_disable_gpio.read(port_power_disable) == RTT::NewData &&
        port_power_disable.states[0].data) {
        return exception(IO_TIMEOUT);
    }
}

void SimulationTask::readStarboardPowerDisableGPIOState()
{
    linux_gpios::GPIOState starboard_power_disable;
    if (_starboard_power_disable_gpio.read(starboard_power_disable) == RTT::NewData &&
        starboard_power_disable.states[0].data) {
        return exception(IO_TIMEOUT);
    }
}

InverterState SimulationTask::currentState() const
{
    InverterState state;
    state.inverter_status = inverterStatus();
    state.time = base::Time::now();

    return state;
}

void SimulationTask::evaluateInverterStatus(InverterStatus status)
{
    if (status == InverterStatus::STATUS_FAULT) {
        error(CONTROLLER_FAULT);
    }
}

void SimulationTask::errorHook()
{
    SimulationTaskBase::errorHook();
    publishFault();

    readExternalFaultGPIOState();
    updateFaultState();
    _inverter_state.write(currentState());

    writeCommandOut(zeroCommand());
    if (inverterStatus() != InverterStatus::STATUS_FAULT) {
        recover();
    }
}

void SimulationTask::stopHook()
{
    SimulationTaskBase::stopHook();
}
void SimulationTask::cleanupHook()
{
    SimulationTaskBase::cleanupHook();
}
