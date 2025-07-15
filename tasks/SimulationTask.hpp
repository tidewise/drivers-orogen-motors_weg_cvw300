/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef MOTORS_WEG_CVW300_SIMULATIONTASK_TASK_HPP
#define MOTORS_WEG_CVW300_SIMULATIONTASK_TASK_HPP

#include "motors_weg_cvw300/SimulationTaskBase.hpp"
#include <random>

namespace motors_weg_cvw300 {

    /*! \class SimulationTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine
     to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These
     interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the
     associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','motors_weg_cvw300::SimulationTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix
     argument.
     */
    class SimulationTask : public SimulationTaskBase {
        friend class SimulationTaskBase;

    protected:
        // begin component properties
        std::string m_joint_name;
        base::JointLimits m_limits;
        base::Time m_watchdog_timeout;
        bool m_edge_triggered_fault_state_output;
        // end component properties

        Fault m_current_fault_state;
        ContactorFaultProbabilities m_contactor_fault_probabilities;
        base::Time m_cmd_deadline;

        base::samples::Joints m_zero_command;

        /**
         * placeholder for the last command written in cmd_out port
         */
        base::samples::Joints m_last_command_out;

        bool m_external_fault;

        std::mt19937 m_distribution_generator;
        std::bernoulli_distribution m_trigger_distribution;
        std::bernoulli_distribution m_break_on_external_fault_distribution;

        void setupDistributionsAndGenerator();

        Fault updateFaultState(Fault const& current_fault_state, bool external_fault);

        bool triggerContactorFault();

        bool exitContactorFault();

        void updateWatchdog();

        void writeCommandOut(base::samples::Joints const& cmd);

        static InverterStatus inverterStatus(Fault const& current_falt,
            base::samples::Joints const& last_command_out);

        void publishFault();

        InverterState currentState() const;

        /**
         * reads external_fault_gpio port and updated m_external_fault
         */
        void readExternalFaultGPIOState();

        void readPowerDisableGPIOState();

        void evaluateInverterStatus(InverterStatus status);

        bool rollProbability(std::bernoulli_distribution& distribution);

    public:
        bool validateCommand(base::samples::Joints cmd,
            SimulationTask::States& command_exception) const;

        base::samples::Joints const& zeroCommand();
        /** TaskContext constructor for SimulationTask
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is
         * Stopped state.
         */
        SimulationTask(std::string const& name = "motors_weg_cvw300::SimulationTask");

        /** Default deconstructor of SimulationTask
         */
        ~SimulationTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif
