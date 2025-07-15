using_task_library "motors_weg_cvw300"

describe OroGen.motors_weg_cvw300.SimulationTask do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.motors_weg_cvw300.SimulationTask
                  .deployed_as("test_task")
        )

        @task.properties.limits = Types.base.JointLimits.new(
            names: ["joint"],
            elements: [{
                min: Types.base.JointState.Speed(-rpm1000),
                max: Types.base.JointState.Speed(rpm1000)
            }]
        )
        @task.properties.joint_name = "joint"
        @task.properties.contactor_fault_probabilities =
            Types.motors_weg_cvw300.ContactorFaultProbabilities.new(
                trigger: 0,
                break_on_external_fault: 0
            )
    end

    describe "configure" do
        it "validades joint name property" do
            task.properties.joint_name = ""
            assert_raises(Roby::EmissionFailed) do
                syskit_configure(task)
            end
        end
    end

    describe "start" do
        it "sends zero command at start" do
            cmd = assert_have_one_new_sample_at_start(task.cmd_out_port)
            assert_zero_cmd cmd
        end

        it "outputs the fault state on start" do
            fault = assert_have_one_new_sample_at_start(task.fault_state_port)
            assert_equal 0, fault.current_fault
        end

        def assert_have_one_new_sample_at_start(port)
            syskit_configure(task)
            expect_execution { task.start! }
                .to do
                    emit task.start_event
                    have_one_new_sample port
                end
        end
    end

    describe "saturation signal" do
        it "does not output a saturation signal if the is no new command" do
            syskit_configure_and_start(task)
            expect_execution.to do
                have_no_new_sample task.saturation_signal_port,
                                   at_least_during: 1
            end
        end

        it "outputs a saturate signal when the command is greater than the max limit" do
            assert_saturation expected_cmd_out: rpm1000, cmd_in: rpm1000 + 1
        end

        it "outputs a saturate signal when the command is less than the min limit" do
            assert_saturation expected_cmd_out: -rpm1000, cmd_in: -rpm1000 - 1
        end

        it "outputs an unsaturated signal" do
            syskit_configure_and_start(task)
            expect_execution { syskit_write task.cmd_in_port, speed_cmd(rpm1000) }
                .to do
                    have_one_new_sample(task.saturation_signal_port)
                        .matching { |s| s.value == 0 }
                end
        end

        def assert_saturation(expected_cmd_out:, cmd_in:)
            syskit_configure_and_start(task)
            _, out = expect_execution do
                         syskit_write task.cmd_in_port, speed_cmd(cmd_in)
                     end.to do
                         [
                             have_one_new_sample(task.saturation_signal_port)
                                 .matching { |s| s.value == 1 },
                             have_one_new_sample(task.cmd_out_port)
                         ]
                     end
            assert_operator (expected_cmd_out - out.elements.first.speed).abs, :<=, 1e-3
        end
    end

    describe "update" do
        it "fills joint name" do
            task.properties.watchdog_timeout = Time.at(0.9)
            syskit_configure_and_start(task)
            s = expect_execution.to { have_one_new_sample task.cmd_out_port }
            assert "joint", s.names.first
        end

        it "validates cmd in joint name" do
            bad_cmd = speed_cmd(0)
            bad_cmd.names[0] = "wrong-joint-name"

            syskit_configure_and_start(task)
            expect_execution { syskit_write task.cmd_in_port, bad_cmd }
                .to_emit task.invalid_joint_name_event
        end

        it "validates cmd in joint type" do
            bad_cmd = Types.base.samples.Joints.new(
                names: ["joint"],
                elements: [Types.base.JointState.Effort(1)]
            )

            syskit_configure_and_start(task)
            expect_execution { syskit_write task.cmd_in_port, bad_cmd }
                .to_emit task.invalid_command_parameter_event
        end

        it "validates cmd in size" do
            bad_cmd = Types.base.samples.Joints.new(
                names: %w[joint1 joint2],
                elements: [Types.base.JointState.Speed(1), Types.base.JointState.Speed(2)]
            )

            syskit_configure_and_start(task)
            expect_execution { syskit_write task.cmd_in_port, bad_cmd }
                .to_emit task.invalid_command_size_event
        end

        it "sends zero command after timeout when no input" do
            task.properties.watchdog_timeout = Time.at(2)
            syskit_configure_and_start(task)
            expect_execution { syskit_write task.cmd_in_port, speed_cmd(1.0) }
                .to do
                    have_one_new_sample(task.cmd_out_port)
                        .matching { |s| s.elements[0].speed == 1 }
                end

            tic = Time.now
            expect_execution.to do
                have_one_new_sample(task.cmd_out_port)
                    .matching { |s| s.elements[0].speed == 0 }
            end
            tac = Time.now

            assert_operator tac - tic, :<=, 2.2
        end

        it "transits to STATUS_FAULT when propulsion enable is off" do
            syskit_configure_and_start(task)
            expect_execution
                .timeout(5)
                .to_not_emit task.controller_fault_event

            expect_execution do
                syskit_write task.external_fault_gpio_port, gpio_state(false)
            end.to_emit task.controller_fault_event
        end

        it "transits to STATUS_FAULT when there is a contactor fault" do
            @task.properties.contactor_fault_probabilities =
            Types.motors_weg_cvw300.ContactorFaultProbabilities.new(
                trigger: 100,
                break_on_external_fault: 0
            )
            syskit_configure_and_start(task)

            expect_execution.to_emit task.controller_fault_event
        end

        it "starts with STATUS_READY" do
            syskit_configure_and_start(task)
            state = expect_execution.to { have_one_new_sample task.inverter_state_port }
            assert_equal :STATUS_READY, state.inverter_status
        end
    end

    describe "error" do
        it "keeps publishing fault and inverter state while on errorHook" do
            syskit_configure_and_start(task)
            expect_execution do
                syskit_write task.external_fault_gpio_port, gpio_state(false)
            end.to_emit task.controller_fault_event

            expect_execution.to do
                have_new_samples task.inverter_state_port, 5
                have_new_samples task.fault_state_port, 5
                maintain(at_least_during: 1) { task.orogen_state == :CONTROLLER_FAULT }
            end
        end
        it "does not keeps publishing fault and inverter state after an IO_TIMEOUT "\
           "errorHook" do
            syskit_configure_and_start(task)
            expect_execution do
                syskit_write task.power_disable_gpio_port, gpio_state(true)
            end.to do
                emit task.io_timeout_event
                have_no_new_sample task.inverter_state_port
                have_no_new_sample task.fault_state_port
            end
        end
    end

    describe "fault reporting" do
        it "does not output on the fault_state port if there is no fault " do
            syskit_configure_and_start(task)
            expect_execution.to { have_no_new_sample task.fault_state_port }
        end

        it "outputs a fault state structure and transitions to fault if the inverter " \
           "is with external fault" do
            syskit_configure_and_start(task)

            fault, state = expect_execution do
                syskit_write task.external_fault_gpio_port, gpio_state(false)
            end.to do
                emit task.controller_fault_event
                [
                    have_one_new_sample(task.fault_state_port),
                    have_one_new_sample(task.inverter_state_port)
                ]
            end

            assert_equal 91, fault.current_fault # external fault
            assert :STATUS_FAULT, state.inverter_status # status fault
        end

        it "transits between error state and running state when the external fault " \
           "gpio goes off" do
            syskit_configure_and_start(task)
            expect_execution do
                syskit_write task.external_fault_gpio_port, gpio_state(false)
            end.to_emit task.controller_fault_event

            expect_execution do
                syskit_write task.external_fault_gpio_port, gpio_state(true)
            end.to do
                have_one_new_sample(task.inverter_state_port)
                    .matching { |s| s.inverter_status == :STATUS_READY }
                emit task.running_event
            end
        end

        it "keeps writing on the output ports when it is in error state when the " \
           "inverter is in fault status" do
            syskit_configure_and_start(task)
            expect_execution do
                syskit_write task.external_fault_gpio_port, gpio_state(false)
            end.to_emit task.controller_fault_event

            cmd = expect_execution.to { have_new_samples(task.cmd_out_port, 5) }
            cmd.each { |s| assert_zero_cmd(s) }
        end

        it "outputs a contactor fault and transits to STATUS_FAULT when there is a "\
           "contactor fault" do
            @task.properties.contactor_fault_probabilities =
                Types.motors_weg_cvw300.ContactorFaultProbabilities.new(
                    trigger: 100,
                    break_on_external_fault: 0
                )
            syskit_configure(task)
            output = expect_execution { task.start! }
            .to do
                emit task.controller_fault_event
            end
            outputs = expect_execution.to do
                [
                    have_one_new_sample(task.inverter_state_port),
                    have_one_new_sample(task.fault_state_port)
                ]
            end
            assert_equal(outputs[0].inverter_status, :STATUS_FAULT)
            assert_equal(outputs[1].current_fault, 185)
        end

        it "transits between error state and running state when the contactor fault " \
           "goes off" do
            @task.properties.contactor_fault_probabilities =
                Types.motors_weg_cvw300.ContactorFaultProbabilities.new(
                    trigger: 100,
                    break_on_external_fault: 100
                )
            syskit_configure_and_start(task)

            outputs = expect_execution.to do
                emit task.controller_fault_event
                [
                    have_one_new_sample(task.inverter_state_port),
                    have_one_new_sample(task.fault_state_port)
                ]
            end
            assert_equal(outputs[0].inverter_status, :STATUS_FAULT)
            assert_equal(outputs[1].current_fault, 185)

            s = expect_execution do
                syskit_write task.external_fault_gpio_port, gpio_state(false)
            end.to do
                have_one_new_sample(task.inverter_state_port)
                    .matching { |s| s.inverter_status == :STATUS_FAULT }
                have_one_new_sample(task.fault_state_port)
            end
            assert_equal(s.current_fault, 91)

            expect_execution do
                syskit_write task.external_fault_gpio_port, gpio_state(true)
            end.to do
                have_one_new_sample(task.inverter_state_port)
                    .matching { |s| s.inverter_status == :STATUS_READY }
                emit task.running_event
            end
        end

        it "emits a IO_TIMEOUT when there is a high value at power_disable_gpio port" do
            syskit_configure_and_start(task)
            expect_execution do
                syskit_write task.power_disable_gpio_port, gpio_state(true)
            end.to do
                emit task.io_timeout_event
            end
        end
    end

    def gpio_state(value)
        Types.linux_gpios.GPIOState.new(
            states: [
                Types.raw_io.Digital.new(data: value)
            ]
        )
    end

    def speed_cmd(value)
        Types.base.samples.Joints.new(
            names: ["joint"],
            elements: [Types.base.JointState.Speed(value)]
        )
    end

    def assert_zero_cmd(cmd)
        assert_equal 0, cmd.elements.first.speed
    end

    def rpm1000
        1000 * 2 * Math::PI / 60
    end
end
