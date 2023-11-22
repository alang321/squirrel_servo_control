import serial
import struct

verbose = True
serial_connection = None


cmd_identifier = {
    'set_serial_port': 0,
    'enable_servo': 1,
    'set_speed': 2,
    'set_position': 3,
    'get_speed': 4,
    'get_position': 5,
    'get_load': 6,
    'get_supply_volt': 7,
    'get_temp': 8,
    'get_is_moving': 9,
    'get_all': 10,
    'set_mode': 11,
    'set_position_async': 12,
    'set_speed_async': 13,
    'trigger_action': 14,
    'set_speed_motor': 15
}

reply_identifier = {
    'reply_get_speed_id': 0,
    'reply_get_position_id': 1,
    'reply_get_load_id': 2,
    'reply_get_supply_volt_id': 3,
    'reply_get_temp_id': 4,
    'reply_get_is_moving_id': 5,
    'reply_get_all_id': 6
}

#START_BYTE = b'\x9A'
#sending messages
struct_str_cmd_set_serial_port = '<bB'
struct_str_cmd_enable_driver = '<bB?'
struct_str_cmd_set_speed = '<bBh'
struct_str_cmd_set_pos = '<bBh'
struct_str_cmd_get_pos = '<bB'
struct_str_cmd_get_spd = '<bB'
struct_str_cmd_get_load = '<bB'
struct_str_cmd_get_volt = '<bB'
struct_str_cmd_get_temp = '<bB'
struct_str_cmd_get_is_moving = '<bB'
struct_str_cmd_get_all = '<bB'
struct_str_cmd_set_mode = '<bBB'
struct_str_cmd_trigger_action = '<b'
struct_str_cmd_set_motor_pwm = '<bBH'

cmd_structs = {cmd_identifier['set_serial_port']: struct_str_cmd_set_serial_port,
                cmd_identifier['enable_servo']: struct_str_cmd_enable_driver,
                cmd_identifier['set_speed']: struct_str_cmd_set_speed,
                cmd_identifier['set_position']: struct_str_cmd_set_pos,
                cmd_identifier['get_speed']: struct_str_cmd_get_spd,
                cmd_identifier['get_position']: struct_str_cmd_get_pos,
                cmd_identifier['get_load']: struct_str_cmd_get_load,
                cmd_identifier['get_supply_volt']: struct_str_cmd_get_volt,
                cmd_identifier['get_temp']: struct_str_cmd_get_temp,
                cmd_identifier['get_is_moving']: struct_str_cmd_get_is_moving,
                cmd_identifier['get_all']: struct_str_cmd_get_all,
                cmd_identifier['set_mode']: struct_str_cmd_set_mode,
                cmd_identifier['set_position_async']: struct_str_cmd_set_pos,
                cmd_identifier['set_speed_async']: struct_str_cmd_set_speed,
                cmd_identifier['trigger_action']: struct_str_cmd_trigger_action,
                cmd_identifier['set_speed_motor']: struct_str_cmd_set_motor_pwm
                }


#receiving messages
replystruct_get_position_format = '<Bh'
replystruct_get_speed_format = '<Bh'
replystruct_get_load_format = '<Bh'
replystruct_get_volt_format = '<Bb'
replystruct_get_temp_format = '<Bb'
replystruct_get_is_moving_format = '<B?'
replystruct_get_all_format = '<Bhhhbb?'

replystructs = {reply_identifier['reply_get_speed_id']: replystruct_get_speed_format,
                reply_identifier['reply_get_position_id']: replystruct_get_position_format,
                reply_identifier['reply_get_load_id']: replystruct_get_load_format,
                reply_identifier['reply_get_supply_volt_id']: replystruct_get_volt_format,
                reply_identifier['reply_get_temp_id']: replystruct_get_temp_format,
                reply_identifier['reply_get_is_moving_id']: replystruct_get_is_moving_format,
                reply_identifier['reply_get_all_id']: replystruct_get_all_format
                }



def writeToSerial(payload_out):
    global serial_connection
    serial_connection.write(b'\xFF')
    serial_connection.write(b'\xFF')
    serial_connection.write(payload_out)
    serial_connection.flush()

def cmd_setSerialPort(port_id):
    struct_var = struct.pack(struct_str_cmd_set_serial_port, cmd_identifier['set_serial_port'], port_id)
    writeToSerial(struct_var)

def cmd_enableServo(servo_id, enable):
    struct_var = struct.pack(struct_str_cmd_enable_driver, cmd_identifier['enable_servo'], servo_id, enable)
    writeToSerial(struct_var)

def cmd_setSpeed(servo_id, speed):
    struct_var = struct.pack(struct_str_cmd_set_speed, cmd_identifier['set_speed'], servo_id, speed)
    #print the buffer in hex
    if verbose:
        print(struct_var)
        print(' '.join(hex(x) for x in struct_var))
    writeToSerial(struct_var)

def cmd_setPosition(servo_id, position):
    struct_var = struct.pack(struct_str_cmd_set_pos, cmd_identifier['set_position'], servo_id, position)
    writeToSerial(struct_var)

def cmd_setMode(servo_id, mode):
    struct_var = struct.pack(struct_str_cmd_set_mode, cmd_identifier['set_mode'], servo_id, mode)
    writeToSerial(struct_var)

def cmd_setPositionAsync(servo_id, position):
    struct_var = struct.pack(struct_str_cmd_set_pos, cmd_identifier['set_position_async'], servo_id, position)
    writeToSerial(struct_var)

def cmd_setSpeedAsync(servo_id, speed):
    struct_var = struct.pack(struct_str_cmd_set_speed, cmd_identifier['set_speed_async'], servo_id, speed)
    writeToSerial(struct_var)

def cmd_setSpeedMotor(motor_id, pwm):
    struct_var = struct.pack(struct_str_cmd_set_motor_pwm, cmd_identifier['set_speed_motor'], motor_id, pwm)
    writeToSerial(struct_var)

def cmd_triggerAction():
    struct_var = struct.pack(struct_str_cmd_trigger_action, cmd_identifier['trigger_action'])
    writeToSerial(struct_var)

def cmd_getSpeed(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_spd, cmd_identifier['get_speed'], servo_id)
    writeToSerial(struct_var)

def cmd_getPosition(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_pos, cmd_identifier['get_position'], servo_id)
    writeToSerial(struct_var)

def cmd_getLoad(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_load, cmd_identifier['get_load'], servo_id)
    writeToSerial(struct_var)

def cmd_getVolt(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_volt, cmd_identifier['get_volt'], servo_id)
    writeToSerial(struct_var)

def cmd_getTemp(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_temp, cmd_identifier['get_temp'], servo_id)
    writeToSerial(struct_var)

def cmd_getIsMoving(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_is_moving, cmd_identifier['get_is_moving'], servo_id)
    writeToSerial(struct_var)

def cmd_getAll(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_all, cmd_identifier['get_all'], servo_id)
    writeToSerial(struct_var)


def process_all_reply():
    buffer = serial_connection.read(struct.calcsize(replystruct_get_all_format) + 1)
    unpacked_reply = struct.unpack(replystruct_get_all_format, buffer[:-1])
    servo_id = unpacked_reply[0]
    position = unpacked_reply[1]
    speed = unpacked_reply[2]
    volt = unpacked_reply[3]
    temp = unpacked_reply[4]
    is_moving = unpacked_reply[5]

    if verbose:
        print("all reply")
        print("servo_id: ", servo_id)
        print("position: ", position)
        print("speed: ", speed)
        print("volt: ", volt)
        print("temp: ", temp)
        print("is_moving: ", is_moving)
    
    return servo_id, position, speed, volt, temp, is_moving

def receive_Message():
    #read all bytes until the start marker and discard, start marker is 2 bytes 0xFF 0xFF
    counter = 0
    while True:
        if serial_connection.read(1, timeout=0.05) == b'\xFF':
            if serial_connection.read(1, timeout=0.05) == b'\xFF':
                break

        #timeout after a few tries
        counter += 1
        if counter > 10:
            print("Error, no start marker found")
            return None


    reply_identifier = int.from_bytes(serial_connection.read(1), byteorder='little')
    if verbose:
        print("Reply:", reply_identifier)

    reply_format = replystructs[reply_identifier]
    buffer = serial_connection.read(struct.calcsize(reply_format) + 1)
    data = struct.unpack(reply_format, buffer[:-1])

    return reply_identifier, data

def is_message_available():
    return serial_connection.in_waiting > 0

def start_serial(port='/dev/serial0', baudrate=230400):
    global serial_connection
    ser = serial.Serial(port=port, baudrate=baudrate,timeout=None, bytesize=serial.EIGHTBITS)

    if not ser.isOpen():
        ser.open()

    print("Connection Opened")
    serial_connection = ser

        



