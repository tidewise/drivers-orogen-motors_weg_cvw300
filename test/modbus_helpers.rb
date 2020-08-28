# frozen_string_literal: true

# Helpers to deal with the component's expected modbus replies
#
# It is based on a register dictionary, and "emulates" the device by
# automatically reading/writing values to this dictionary
#
# In modbus_configure_and_start, the component might go "a little
# bit further" than the start, and actually read the speed but
# not finish a complete state update. The tests would then "finish"
# the state update and the sample would have a zero speed.
#
#   Component             Syskit
#   --------              ------
#                         ENTER modbus_configure_and_start
#   ENTER startHook       |
#   EXIT startHook        |
#   ENTER updateHook      |
#   READ register 2       |
#   |                     EXIT modbus_configure_and_start
#   |                     ENTER test
#   READ other registers  |
#   WRITE sample          |
#   |                     have_one_new_sample -> speed == 0
#   EXIT updateHook
#
# To handle this, either look for an eventual behavior (using for instance the
# filtered version of have_one_new_sample, or the achieve expectation), or set
# the modbus dictionary entries before the start.
module ModbusHelpers
    def setup
        super

        @modbus_registers = {}
    end

    def iodrivers_base_prepare(model)
        task = syskit_deploy(model)
        syskit_start_execution_agents(task)
        reader = syskit_create_reader(task.io_raw_out_port, type: :buffer, size: 10)
        writer = syskit_create_writer(task.io_raw_in_port)

        [task, reader, writer]
    end

    def modbus_helpers_setup(task, reader, writer)
        @__modbus_task = task
        @__modbus_reader = reader
        @__modbus_writer = writer
    end

    def modbus_stop_task
        expect_execution { task.stop! }
            .join_all_waiting_work(false)
            .poll do
                while (sample = @__modbus_reader.read_new)
                    @__modbus_writer.write(modbus_reply(sample))
                end
            end
            .to { emit task.stop_event }
    end

    def modbus_expect_during_configuration_and_start
        expect_execution
            .scheduler(true)
            .join_all_waiting_work(false)
            .poll do
                while (sample = @__modbus_reader.read_new)
                    @__modbus_writer.write(modbus_reply(sample))
                end
            end
    end

    # Configure and start the modbus component given to {#modbus_helpers_setup}
    #
    # See the {ModbusHelpers} documentation for caveats w.r.t. expectations and
    # starting the component
    def modbus_configure_and_start
        modbus_expect_during_configuration_and_start.to { emit task.start_event }
    end

    def modbus_set(register, value)
        @modbus_registers[register] = value
    end

    def modbus_get(register)
        @modbus_registers.fetch(register)
    rescue KeyError
        raise ArgumentError, "register #{register} not set"
    end

    def modbus_expect_execution(writer, reader, &block)
        expect_execution(&block)
            .join_all_waiting_work(false)
            .poll do
                while (s = reader.read_new)
                    writer.write(modbus_reply(s))
                end
            end
    end

    def modbus_reply_until(writer, reader)
        sample = nil
        expect_execution
            .join_all_waiting_work(false)
            .poll do
                while !sample && (s = reader.read_new)
                    writer.write(modbus_reply(s))
                    sample = s if yield(s)
                end
            end
            .to { achieve { sample } }
    end

    def modbus_write?(sample, register: nil, value: nil)
        return unless sample.data[1] == 6

        actual_register = sample.data[2] << 8 | sample.data[3]
        actual_value = sample.data[4] << 8 | sample.data[5]

        (!register || register == actual_register) &&
            (!value || value == actual_value)
    end

    def modbus_reply(sample)
        reply = [sample.data[0], sample.data[1]]

        function = sample.data[1]
        if function == 6 # write
            address = sample.data[2] << 8 | sample.data[3]
            value = sample.data[4] << 8 | sample.data[5]
            @modbus_registers[address] = value
        elsif [3, 4].include?(function) # Read registers
            start_address  = sample.data[2] << 8 | sample.data[3]
            register_count = sample.data[4] << 8 | sample.data[5]

            reply << register_count * 2
            register_count.times do |i|
                register_value = modbus_get(start_address + i)
                reply += [register_value].pack("s>").unpack("C*")
            end
        end
        reply += modbus_crc(reply)
        Types.iodrivers_base.RawPacket.new(time: Time.now, data: reply)
    end

    def modbus_reply_to_write(id)
        reply = [id, 6]
        reply += modbus_crc(reply)
        Types.iodrivers_base.RawPacket.new(time: Time.now, data: reply)
    end

    def modbus_crc(data)
        crc = 0xFFFF
        data.each do |byte|
            crc ^= byte

            8.times do
                odd = crc.odd?
                crc >>= 1
                crc ^= 0xA001 if odd
            end
        end

        [crc & 0xFF, (crc & 0xFF00) >> 8]
    end
end
