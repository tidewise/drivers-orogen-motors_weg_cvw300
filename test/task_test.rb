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

    describe "inverted = false" do
        before do
            @task.properties.inverted = false
            modbus_configure_and_start
        end

        it "sends the command as given" do
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
            modbus_configure_and_start
        end

        it "sends an inverted command" do
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

            sample = modbus_expect_execution(@writer, @reader).to do
                have_one_new_sample task.joint_samples_port
            end
            assert_in_delta -1.57, sample.elements[0].speed, 1e-2
            assert_in_delta -13.92, sample.elements[0].effort, 1e-2
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
