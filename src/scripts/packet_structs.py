import struct

start_marker1 = b'\xBF'
start_marker2 = b'\xFF'

class CmdIdentifier:
    set_serial_port = 0
    enable_servo = 1
    set_speed = 2
    set_position = 3
    get_speed = 4
    get_position = 5
    get_load = 6
    get_supply_volt = 7
    get_temp = 8
    get_isMoving = 9
    get_all = 10
    set_mode = 11
    set_position_async = 12
    set_speed_async = 13
    trigger_action = 14
    set_motor_speed = 15
    set_zero_position = 16

class Command:
    data_format = ''
    cmd_identifier_format = '<B'
    checksum_format = 'B'
    identifier = 0

    def __init__(self, *args):
        self.data = args
        self.struct_format = self.cmd_identifier_format + self.data_format + self.checksum_format

    def getBuffer(self):
        struct.pack(self.struct_format, self.identifier, *self.data, self.calculateChecksum())
        return 

    def calculateChecksum(self, buffer_without_check):
        checksum = 0

        frmt = self.cmd_identifier_format + self.data_format
        buffer = struct.pack(frmt, self.identifier, *self.data)
        for i in buffer:
            checksum += int.from_bytes(i, "little")

        return (~(checksum % 256)) & 0xFF

class CommandSetSerialPort(Command):
    data_format = 'B'
    identifier = CmdIdentifier.set_serial_port

    def __init__(self, serial_port_id):
        super().__init__(serial_port_id)

class CommandEnableServo(Command):
    data_format = 'B?'
    identifier = CmdIdentifier.enable_servo

    def __init__(self, servo_id, enable):
        super().__init__(servo_id, enable)

class CommandSetSpeed(Command):
    data_format = 'Bh'
    identifier = CmdIdentifier.set_speed

    def __init__(self, servo_id, speed):
        super().__init__(servo_id, speed)

class CommandSetPosition(Command):
    data_format = 'Bh'
    identifier = CmdIdentifier.set_position

    def __init__(self, servo_id, position):
        super().__init__(servo_id, position)

class CommandSetMode(Command):
    data_format = 'BB'
    identifier = CmdIdentifier.set_mode

    def __init__(self, servo_id, mode):
        super().__init__(servo_id, mode)

class CommandSetPositionAsync(Command):
    data_format = 'Bh'
    identifier = CmdIdentifier.set_position_async

    def __init__(self, servo_id, position):
        super().__init__(servo_id, position)

class CommandSetSpeedAsync(Command):
    data_format = 'Bh'
    identifier = CmdIdentifier.set_speed_async

    def __init__(self, servo_id, speed):
        super().__init__(servo_id, speed)

class CommandTriggerAction(Command):
    data_format = ''
    identifier = CmdIdentifier.trigger_action

    def __init__(self):
        super().__init__()

class CommandSetMotorSpeed(Command):
    data_format = 'BH'
    identifier = CmdIdentifier.set_motor_speed

    def __init__(self, motor_id, pwm):
        super().__init__(motor_id, pwm)

class CommandSetZeroPosition(Command):
    data_format = 'B'
    identifier = CmdIdentifier.set_zero_position

    def __init__(self, servo_id):
        super().__init__(servo_id)

class CommandGetSpeed(Command):
    data_format = 'B'
    identifier = CmdIdentifier.get_speed

    def __init__(self, servo_id):
        super().__init__(servo_id)

class CommandGetPosition(Command):
    data_format = 'B'
    identifier = CmdIdentifier.get_position

    def __init__(self, servo_id):
        super().__init__(servo_id)

class CommandGetLoad(Command):
    data_format = 'B'
    identifier = CmdIdentifier.get_load

    def __init__(self, servo_id):
        super().__init__(servo_id)

class CommandGetSupplyVolt(Command):
    data_format = 'B'
    identifier = CmdIdentifier.get_supply_volt

    def __init__(self, servo_id):
        super().__init__(servo_id)

class CommandGetTemp(Command):
    data_format = 'B'
    identifier = CmdIdentifier.get_temp

    def __init__(self, servo_id):
        super().__init__(servo_id)


class CommandGetIsMoving(Command):
    data_format = 'B'
    identifier = CmdIdentifier.get_isMoving

    def __init__(self, servo_id):
        super().__init__(servo_id)

class CommandGetAll(Command):
    data_format = 'B'
    identifier = CmdIdentifier.get_all

    def __init__(self, servo_id):
        super().__init__(servo_id)

    


class ReplyIdentifiers:
    reply_get_speed_id = 0
    reply_get_position_id = 1
    reply_get_load_id = 2
    reply_get_supply_volt_id = 3
    reply_get_temp_id = 4
    reply_get_isMoving_id = 5
    reply_get_all_id = 6

    def isValidReplyId(self, reply_id):
        return reply_id <= self.reply_get_all_id

# A structure to access the plystruct class by the reply identifier
reply_classes = {
    ReplyIdentifiers.reply_get_speed_id: ReplyGetSpeed,
    ReplyIdentifiers.reply_get_position_id: ReplyGetPosition,
    ReplyIdentifiers.reply_get_load_id: ReplyGetLoad,
    ReplyIdentifiers.reply_get_supply_volt_id: ReplyGetSupplyVolt,
    ReplyIdentifiers.reply_get_temp_id: ReplyGetTemp,
    ReplyIdentifiers.reply_get_isMoving_id: ReplyGetIsMoving,
    ReplyIdentifiers.reply_get_all_id: ReplyGetAll
}

class Reply:
    data_format = ''
    identifier = 0

    def __init__(self):
        if self.verifyChecksum(buffer):
            self.data = self.unpack(buffer)
            self.valid = True
        else:
            self.valid = False
            ros.loginfo("Checksum failed for reply get speed")

    def verifyChecksum(self, buffer):
        checksum = identifier

        for i in range(len(buffer) - 1):
            checksum += int.from_bytes(buffer[i], "little")

        return (~(checksum % 256)) & 0xFF == buffer[-1]

    def unpack(self, buffer):
        return struct.unpack(self.data_format, buffer[:-1])

    def getBufferLength(self):
        return struct.calcsize(self.data_format)

class ReplyGetSpeed(ReplyStruct):
    data_format = '<Bh'
    identifier = ReplyIdentifiers.reply_get_speed_id

    def __init__(self, buffer):
        super().__init__(buffer)

        if self.valid:
            self.servo_id = self.data[0]
            self.speed = self.data[1]
        else:
            self.position = None
            self.servo_id = None

class ReplyGetPosition(ReplyStruct):
    data_format = '<Bh'
    identifier = ReplyIdentifiers.reply_get_position_id

    def __init__(self, buffer):
        super().__init__(buffer)

        if self.valid:
            self.servo_id = self.data[0]
            self.position = self.data[1]
        else:
            self.position = None
            self.servo_id = None

class ReplyGetLoad(ReplyStruct):
    data_format = '<Bh'
    identifier = ReplyIdentifiers.reply_get_load_id

    def __init__(self, buffer):
        super().__init__(buffer)

        if self.valid:
            self.servo_id = self.data[0]
            self.load = self.data[1]
        else:
            self.load = None
            self.servo_id = None

class ReplyGetSupplyVolt(ReplyStruct):
    data_format = '<Bb'
    identifier = ReplyIdentifiers.reply_get_supply_volt_id

    def __init__(self, buffer):
        super().__init__(buffer)

        if self.valid:
            self.servo_id = self.data[0]
            self.volt = self.data[1]
        else:
            self.volt = None
            self.servo_id = None

class ReplyGetTemp(ReplyStruct):
    data_format = '<Bb'
    identifier = ReplyIdentifiers.reply_get_temp_id

    def __init__(self, buffer):
        super().__init__(buffer)

        if self.valid:
            self.servo_id = self.data[0]
            self.temp = self.data[1]
        else:
            self.temp = None
            self.servo_id = None

class ReplyGetIsMoving(ReplyStruct):
    data_format = '<B?'
    identifier = ReplyIdentifiers.reply_get_isMoving_id

    def __init__(self, buffer):
        super().__init__(buffer)

        if self.valid:
            self.servo_id = self.data[0]
            self.is_moving = self.data[1]
        else:
            self.is_moving = None
            self.servo_id = None

class ReplyGetAll(ReplyStruct):
    data_format = '<Bhhhbb'
    identifier = ReplyIdentifiers.reply_get_all_id

    def __init__(self, buffer):
        super().__init__(buffer)

        if self.valid:
            self.servo_id = self.data[0]
            self.position = self.data[1]
            self.speed = self.data[2]
            self.load = self.data[3]
            self.volt = self.data[4]
            self.temp = self.data[5]
        else:
            self.position = None
            self.servo_id = None
            self.speed = None
            self.load = None
            self.volt = None
            self.temp = None