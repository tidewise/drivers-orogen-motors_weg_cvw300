/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SimulationTask.hpp"

#include <control_base/SaturationSignal.hpp>

using namespace motors_weg_cvw300;

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

    if (_joint_name.get() == "") {
        throw std::runtime_error("joint_name property must be set");
    }
    m_joint_name = _joint_name.get();

    m_zero_command = speedCommand(0, m_joint_name);

    return true;
}
bool SimulationTask::startHook()
{
    if (!SimulationTaskBase::startHook()) {
        return false;
    }
    updateWatchdog();
    _cmd_out.write(zeroCommand());
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

base::samples::Joints const& SimulationTask::zeroCommand()
{
    m_zero_command.time = base::Time::now();
    return m_zero_command;
}

void SimulationTask::updateHook()
{
    SimulationTaskBase::updateHook();

    if (!m_cmd_deadline.isNull() && base::Time::now() > m_cmd_deadline) {
        _cmd_out.write(zeroCommand());
    }

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
        _cmd_out.write(cmd_out);
    }
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
void SimulationTask::errorHook()
{
    SimulationTaskBase::errorHook();
}
void SimulationTask::stopHook()
{
    SimulationTaskBase::stopHook();
}
void SimulationTask::cleanupHook()
{
    SimulationTaskBase::cleanupHook();
}
