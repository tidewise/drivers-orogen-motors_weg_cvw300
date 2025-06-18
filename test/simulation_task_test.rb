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
            syskit_configure(task)
            cmd = expect_execution { task.start! }
                  .to do
                      emit task.start_event
                      have_one_new_sample task.cmd_out_port
                  end
            assert_zero_cmd cmd
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
