# frozen_string_literal: true

require_relative "modbus_helpers"

using_task_library "motors_weg_cvw300"

describe OroGen.motors_weg_cvw300.Task do
    run_live

    attr_reader :task
    attr_reader :reader

    include ModbusHelpers

    before do
        @task, @reader, @writer = iodrivers_base_prepare(
            OroGen.motors_weg_cvw300.Task
                  .deployed_as("motors_weg_cvw300_test")
        )
        modbus_helpers_setup(@task, @reader, @writer)

        @task.properties.io_read_timeout = Time.at(2)
        @task.properties.modbus_interframe_delay = Time.at(0.01)
        @task.properties.watchdog do |sw|
            sw.timeout = Time.at(0.5)
            sw
        end

        modbus_set(405, 512) # number encoder pulses
        modbus_set(401, 10) # nominal current
        modbus_set(402, 2000) # nominal speed (rpm)
        modbus_set(404, 2) # 12 kW

        modbus_set(2, 0) # actual rotation (rpm)
        modbus_set(3, 0) # actual current (A)
        modbus_set(4, 0) # battery voltage (0.1 V)
        modbus_set(5, 0) # output frequency (0.1 Hz)
        modbus_set(6, 1) # inverter state (1 == Run)
        modbus_set(7, 0) # output voltage (0.1V)
        modbus_set(8, 0) # not used - vehicle speed
        modbus_set(9, 0) # motor torque (0.1 % torque nominal)

        modbus_set(30, 0) # mosfet temperature (0.1 C)
        modbus_set(34, 0) # air temperature (0.1 C)

        modbus_set(37, 20) # motor overload ration (percentage 0-100)

        modbus_set(48, 0) # current alarm
        modbus_set(49, 0) # current fault

        modbus_set(50, 1) # last faults
        modbus_set(54, 2)
        modbus_set(58, 3)
        modbus_set(62, 4)
        modbus_set(66, 5)
        modbus_set(90, 11) # inverter state during last fault
        modbus_set(91, 12)
        modbus_set(92, 13)
        modbus_set(93, 14)
        modbus_set(94, 15)
        modbus_set(95, 16)
    end

    after do
        modbus_stop_task if task&.running?
    end

    describe "configuration" do
        it "sets the control type" do
            task.properties.control_type = :CONTROL_SENSORLESS
            modbus_configure_and_start
            assert_equal 1, modbus_get(202)
        end

        it "sets the ramps" do
            task.properties.ramps = {
                acceleration_time: Time.at(5.1),
                deceleration_time: Time.at(7.2),
                type: "RAMP_S_CURVE"
            }
            modbus_configure_and_start
            assert_equal 5, modbus_get(100)
            assert_equal 7, modbus_get(101)
            assert_equal 1, modbus_get(104)
        end

        it "sets the speed limits" do
            rpm100 = 100 * 2 * Math::PI / 60
            task.properties.limits = Types.base.JointLimits.new(
                elements: [{
                    min: Types.base.JointState.Speed(-rpm100),
                    max: Types.base.JointState.Speed(rpm100)
                }]
            )

            modbus_configure_and_start

            assert_equal 100, modbus_get(134)
        end
    end

    describe "fault and alarm reporting" do
        it "does not output on the fault_state port if there is no alarm or fault " do
            modbus_configure_and_start
            expect_execution.timeout(1).to do
                have_no_new_sample task.fault_state_port
            end
        end

        it "outputs the fault state on start" do
            now = Time.now
            sample = modbus_expect_during_configuration_and_start.to do
                have_one_new_sample task.fault_state_port
            end

            assert Time.at(now.tv_sec, 0) < sample.time
            assert_equal 0, sample.current_fault
            assert_equal [1, 2, 3, 4, 5], sample.fault_history.to_a
            assert_in_delta 1.1, sample.current
            assert_in_delta 1.2, sample.battery_voltage
            assert_equal 13, (sample.speed / 2 / Math::PI * 60).round
            assert_equal 14, (sample.command / 2 / Math::PI * 60).round
            assert_in_delta 1.5, sample.inverter_output_frequency
            assert_in_delta 1.6, sample.inverter_output_voltage
        end

        it "outputs an alarm state structure if there is an alarm" do
            modbus_configure_and_start

            now = Time.now
            sample = modbus_expect_execution(@writer, @reader) do
                modbus_set(48, 1)
            end.to { have_one_new_sample task.alarm_state_port }
            assert Time.at(now.tv_sec, 0) < sample.time
            assert_equal 1, sample.current_alarm
        end

        it "outputs a fault state structure and transitions to fault if the inverter is in under-voltage" do
            modbus_configure_and_start

            now = Time.now
            sample = modbus_expect_execution(@writer, @reader) do
                modbus_set(49, 1)
                modbus_set(6, 2)
            end.to do
                emit task.controller_under_voltage_event
                have_one_new_sample task.fault_state_port
            end
            assert Time.at(now.tv_sec, 0) < sample.time
            assert_equal 1, sample.current_fault
            assert_equal [1, 2, 3, 4, 5], sample.fault_history.to_a
            assert_in_delta 1.1, sample.current
            assert_in_delta 1.2, sample.battery_voltage
            assert_equal 13, (sample.speed / 2 / Math::PI * 60).round
            assert_equal 14, (sample.command / 2 / Math::PI * 60).round
            assert_in_delta 1.5, sample.inverter_output_frequency
            assert_in_delta 1.6, sample.inverter_output_voltage
        end

        it "outputs a fault state structure and transitions to fault if there is a fault" do
            modbus_configure_and_start

            now = Time.now
            sample = modbus_expect_execution(@writer, @reader) do
                modbus_set(49, 1)
                modbus_set(6, 3)
            end.to do
                emit task.controller_fault_event
                have_one_new_sample task.fault_state_port
            end
            assert Time.at(now.tv_sec, 0) < sample.time
            assert_equal 1, sample.current_fault
            assert_equal [1, 2, 3, 4, 5], sample.fault_history.to_a
            assert_in_delta 1.1, sample.current
            assert_in_delta 1.2, sample.battery_voltage
            assert_equal 13, (sample.speed / 2 / Math::PI * 60).round
            assert_equal 14, (sample.command / 2 / Math::PI * 60).round
            assert_in_delta 1.5, sample.inverter_output_frequency
            assert_in_delta 1.6, sample.inverter_output_voltage
        end
    end

    describe "inverted = false" do
        before do
            @task.properties.inverted = false
        end

        it "sends the command as given" do
            modbus_configure_and_start
            cmd = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Speed(1000 * 2 * Math::PI / 60)]
            )
            syskit_write task.cmd_in_port, cmd

            modbus_reply_until(@writer, @reader) do |sample|
                modbus_write?(sample, register: 683, value: 0x1000)
            end
        end

        it "reports the motor status as-is" do
            # speed, current, battery voltage, frequency, inverter status
            # output voltage and torque

            modbus_set 2, 15
            modbus_set 3, 12
            modbus_set 4, 421
            modbus_set 5, 502
            modbus_set 6, 4
            modbus_set 7, 128
            modbus_set 9, 243

            modbus_configure_and_start
            sample = modbus_expect_execution(@writer, @reader).to do
                have_one_new_sample task.joint_samples_port
            end
            assert_in_delta 1.57, sample.elements[0].speed, 1e-2
            assert_in_delta 13.92, sample.elements[0].effort, 1e-2
        end
    end

    describe "inverted = true" do
        before do
            @task.properties.inverted = true
        end

        it "sends an inverted command" do
            modbus_configure_and_start
            cmd = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Speed(1000 * 2 * Math::PI / 60)]
            )
            syskit_write task.cmd_in_port, cmd

            modbus_reply_until(@writer, @reader) do |sample|
                modbus_write?(sample, register: 683, value: 0xf000)
            end
        end

        it "inverts the motor status" do
            # speed, current, battery voltage, frequency, inverter status
            # output voltage and torque

            modbus_set 2, 15
            modbus_set 3, 12
            modbus_set 4, 421
            modbus_set 5, 502
            modbus_set 6, 4
            modbus_set 7, 128
            modbus_set 9, 243

            modbus_configure_and_start
            sample = modbus_expect_execution(@writer, @reader).to do
                have_one_new_sample task.joint_samples_port
            end
            assert_in_delta -1.57, sample.elements[0].speed, 1e-2
            assert_in_delta -13.92, sample.elements[0].effort, 1e-2
        end
    end

    describe "input validation" do
        it "transitions to INVALID_COMMAND_SIZE if the input command is empty" do
            modbus_configure_and_start
            cmd = Types.base.samples.Joints.new(elements: [])
            expect_execution { syskit_write task.cmd_in_port, cmd }
                .to { emit task.invalid_command_size_event }
        end

        it "transitions to INVALID_COMMAND_SIZE if the input command "\
           "has more than one element" do
            modbus_configure_and_start
            cmd = Types.base.samples.Joints.new(
                elements: [
                    Types.base.JointState.Speed(1000 * 2 * Math::PI / 60),
                    Types.base.JointState.Speed(1000 * 2 * Math::PI / 60)
                ]
            )
            expect_execution { syskit_write task.cmd_in_port, cmd }
                .to { emit task.invalid_command_size_event }
        end

        it "transitions to INVALID_COMMAND_PARAMETER if the input command "\
           "does not have a valid speed" do
            modbus_configure_and_start
            cmd = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Speed(Float::NAN)]
            )
            expect_execution { syskit_write task.cmd_in_port, cmd }
                .to { emit task.invalid_command_parameter_event }
        end
    end

    describe "command watchdog" do
        it "periodically sends zero velocity commands" do
            @task.properties.watchdog do |sw|
                sw.timeout = Time.at(0.01)
                sw
            end
            modbus_configure_and_start
            10.times do
                modbus_reply_until(@writer, @reader) do |sample|
                    modbus_write?(sample, register: 683, value: 0)
                end
            end
        end

        it "waits watchdog.timeout after a new command before it sends "\
           "a new velocity command again" do
            modbus_configure_and_start

            cmd = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Speed(1000 * 2 * Math::PI / 60)]
            )
            syskit_write task.cmd_in_port, cmd
            modbus_reply_until(@writer, @reader) do |sample|
                # Note: register 683 is expressed in ratio of nominal speed,
                # with 0x2000 (8192) == 100%
                modbus_write?(sample, register: 683, value: 0x1000)
            end
            tic = Time.now
            modbus_reply_until(@writer, @reader) do |sample|
                modbus_write?(sample, register: 683, value: 0)
            end
            assert((Time.now - tic) > 0.4)
        end

        it "does not send zero velocity commnds if new commands are regularly sent" do
            @task.properties.watchdog do |sw|
                sw.timeout = Time.at(0.1)
                sw
            end
            modbus_configure_and_start

            cmd = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Speed(1000 * 2 * Math::PI / 60)]
            )

            10.times do
                syskit_write task.cmd_in_port, cmd
                modbus_reply_until(@writer, @reader) do |sample|
                    # Note: register 683 is expressed in ratio of nominal speed,
                    # with 0x2000 (8192) == 100%
                    if modbus_write?(sample, register: 683, value: 0)
                        flunk("received zero velocity command")
                    else
                        modbus_write?(sample, register: 683, value: 0x1000)
                        sleep 0.02
                    end
                end
            end
        end
    end
end
