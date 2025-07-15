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
    setupDistributionsAndGenerator();

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

void SimulationTask::setupDistributionsAndGenerator()
{
    mt19937 m_distribution_generator(std::random_device{}());
    m_contactor_fault_probabilities = _contactor_fault_probabilities.get();
    m_trigger_distribution =
        bernoulli_distribution(m_contactor_fault_probabilities.trigger);
    m_break_on_external_fault_distribution =
        bernoulli_distribution(m_contactor_fault_probabilities.break_on_external_fault);
}

bool SimulationTask::startHook()
{
    if (!SimulationTaskBase::startHook()) {
        return false;
    }
    m_current_fault_state = Fault::NO_FAULT;
    m_external_fault = false;
    m_trigger_distribution.reset();
    m_break_on_external_fault_distribution.reset();

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
    if (inverterStatus(m_current_fault_state, m_last_command_out) !=
        InverterStatus::STATUS_FAULT) {
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

InverterStatus SimulationTask::inverterStatus(Fault const& current_fault_state,
    base::samples::Joints const& last_command_out)
{
    if (current_fault_state != Fault::NO_FAULT) {
        return InverterStatus::STATUS_FAULT;
    }

    if (last_command_out.elements.size() && last_command_out.elements[0].speed != 0) {
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
    readPowerDisableGPIOState();
    readExternalFaultGPIOState();
    m_current_fault_state = updateFaultState(m_current_fault_state, m_external_fault);

    InverterState state = currentState();
    _inverter_state.write(state);

    evaluateInverterStatus(state.inverter_status);
}

bool SimulationTask::triggerContactorFault()
{
    return rollProbability(m_trigger_distribution);
}

bool SimulationTask::rollProbability(std::bernoulli_distribution& distribution)
{
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

Fault SimulationTask::updateFaultState(Fault const& current_fault_state,
    bool external_fault)
{
    if (current_fault_state == Fault::CONTACTOR_FAULT) {
        if (external_fault && exitContactorFault()) {
            return Fault::EXTERNAL_FAULT;
        }
        return Fault::CONTACTOR_FAULT;
    }
    if (!external_fault && current_fault_state == Fault::EXTERNAL_FAULT) {
        return Fault::NO_FAULT;
    }
    if (triggerContactorFault()) {
        return Fault::CONTACTOR_FAULT;
    }
    if (external_fault) {
        return Fault::EXTERNAL_FAULT;
    }
    else {
        return Fault::NO_FAULT;
    }
}

bool SimulationTask::exitContactorFault()
{
    return rollProbability(m_break_on_external_fault_distribution);
}

void SimulationTask::readPowerDisableGPIOState()
{
    linux_gpios::GPIOState power_disable;
    if (_power_disable_gpio.read(power_disable) == RTT::NewData &&
        power_disable.states[0].data) {
        return exception(IO_TIMEOUT);
    }
}

InverterState SimulationTask::currentState() const
{
    InverterState state;
    state.inverter_status = inverterStatus(m_current_fault_state, m_last_command_out);
    state.time = base::Time::now();

    return state;
}

void SimulationTask::evaluateInverterStatus(InverterStatus status)
{
    if (status == InverterStatus::STATUS_FAULT) {
        publishFault();
        error(CONTROLLER_FAULT);
    }
}

void SimulationTask::errorHook()
{
    SimulationTaskBase::errorHook();

    readExternalFaultGPIOState();
    m_current_fault_state = updateFaultState(m_current_fault_state, m_external_fault);
    publishFault();
    _inverter_state.write(currentState());

    writeCommandOut(zeroCommand());
    if (inverterStatus(m_current_fault_state, m_last_command_out) !=
        InverterStatus::STATUS_FAULT) {
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
