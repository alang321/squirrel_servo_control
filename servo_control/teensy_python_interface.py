from dataclasses import dataclass, fields,asdict
import dataclasses 
from typing import List, Optional, Tuple, Union
import serial
import struct
from time import sleep
from math import floor

START_BYTE = b'\x9A'
#sending messages
struct_str_cmd_set_serial_port = '<bB'
struct_str_cmd_enable_driver = '<bB?'
struct_str_cmd_set_speed = '<bBh'
struct_str_cmd_add_pos = '<bBh'
struct_str_cmd_get_pos = '<bB'
struct_str_cmd_get_spd = '<bB'
struct_str_cmd_get_volt = '<bB'
struct_str_cmd_get_temp = '<bB'
struct_str_cmd_get_is_moving = '<bB'
struct_str_cmd_get_all = '<bB'


#receiving messages
replystruct_get_position_format = '<Bh'
replystruct_get_speed_format = '<Bh'
replystruct_get_volt_format = '<Bb'
replystruct_get_temp_format = '<Bb'
replystruct_get_is_moving_format = '<B?'
replystruct_get_all_format = '<Bhhbb?'

cmd_identifier = {
    'set_serial_port': 0,
    'enable_servo': 1,
    'set_speed': 2,
    'set_position': 3,
    'get_speed': 4,
    'get_position': 5,
    'get_volt': 6,
    'get_temp': 7,
    'get_is_moving': 8,
    'get_all': 9
}

reply_identifier = {
    'reply_get_speed_id': 0,
    'reply_get_position_id': 1,
    'reply_get_volt_id': 2,
    'reply_get_temp_id': 3,
    'reply_get_is_moving_id': 4,
    'reply_get_all_id': 5
}


def writeToSerial(payload_out):
    #serial_connection.write(START_BYTE)
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
    print(struct_var)
    print(' '.join(hex(x) for x in struct_var))
    writeToSerial(struct_var)

def cmd_setPosition(servo_id, position):
    struct_var = struct.pack(struct_str_cmd_add_pos, cmd_identifier['set_position'], servo_id, position)
    writeToSerial(struct_var)

def cmd_getSpeed(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_spd, cmd_identifier['get_speed'], servo_id)
    writeToSerial(struct_var)

def cmd_getPosition(servo_id):
    struct_var = struct.pack(struct_str_cmd_get_pos, cmd_identifier['get_position'], servo_id)
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

def process_speed_reply():
    buffer = serial_connection.read(struct.calcsize(replystruct_get_speed_format) + 1)
    unpacked_reply = struct.unpack(replystruct_get_position_format, buffer[:-1])
    servo_id = unpacked_reply[0]
    speed = unpacked_reply[1]

    print("Speed reply")
    print("servo_id: ", servo_id)
    print("speed: ", speed)

def process_position_reply():
    buffer = serial_connection.read(struct.calcsize(replystruct_get_position_format) + 1)
    unpacked_reply = struct.unpack(replystruct_get_position_format, buffer[:-1])
    servo_id = unpacked_reply[0]
    position = unpacked_reply[1]

    print("Position reply")
    print("servo_id: ", servo_id)
    print("position: ", position)

def process_volt_reply():
    buffer = serial_connection.read(struct.calcsize(replystruct_get_volt_format) + 1)
    unpacked_reply = struct.unpack(replystruct_get_volt_format, struct.calcsize(replystruct_get_volt_format)[:-1])
    servo_id = unpacked_reply[0]
    volt = unpacked_reply[1]

    print("volt reply")
    print("servo_id: ", servo_id)
    print("volt: ", volt)

def process_temp_reply():
    buffer = serial_connection.read(struct.calcsize(replystruct_get_temp_format) + 1)
    unpacked_reply = struct.unpack(replystruct_get_temp_format, buffer[:-1])
    servo_id = unpacked_reply[0]
    temp = unpacked_reply[1]

    print("temp reply")
    print("servo_id: ", servo_id)
    print("temp: ", temp)

def process_is_moving_reply():
    buffer = serial_connection.read(struct.calcsize(replystruct_get_is_moving_format) + 1)
    unpacked_reply = struct.unpack(replystruct_get_is_moving_format, buffer[:-1])
    servo_id = unpacked_reply[0]
    is_moving = unpacked_reply[1]

    print("is_moving reply")
    print("servo_id: ", servo_id)
    print("is_moving: ", is_moving)

def process_all_reply():
    buffer = serial_connection.read(struct.calcsize(replystruct_get_all_format) + 1)
    unpacked_reply = struct.unpack(replystruct_get_all_format, buffer[:-1])
    servo_id = unpacked_reply[0]
    position = unpacked_reply[1]
    speed = unpacked_reply[2]
    volt = unpacked_reply[3]
    temp = unpacked_reply[4]
    is_moving = unpacked_reply[5]

    print("all reply")
    print("servo_id: ", servo_id)
    print("position: ", position)
    print("speed: ", speed)
    print("volt: ", volt)
    print("temp: ", temp)
    print("is_moving: ", is_moving)

reply_handlers = {
    reply_identifier['reply_get_speed_id']: process_speed_reply,
    reply_identifier['reply_get_position_id']: process_position_reply,
    reply_identifier['reply_get_volt_id']: process_volt_reply,
    reply_identifier['reply_get_temp_id']: process_temp_reply,
    reply_identifier['reply_get_is_moving_id']: process_is_moving_reply,
    reply_identifier['reply_get_all_id']: process_all_reply
}

def receive_Message():
    #read first byte and convert to int
    reply_identifier = int.from_bytes(serial_connection.read(1), byteorder='little')
    print("Reply:", reply_identifier)

    reply_handlers[reply_identifier]()


serial_connection = serial.Serial(port='/dev/serial0', baudrate=115200,timeout=None, bytesize=serial.EIGHTBITS)
print("Connection Opened")


if not serial_connection.isOpen():
    serial_connection.open()
print("Set Serial Port")
cmd_setSerialPort(1)

sleep(2)
print("Set velocity")
cmd_setSpeed(9, 5000)

pos = 1

while True:
    cmd_getAll(9)

    sleep(1)
    print("Set pos")
    cmd_setPosition(9, 1000)

    sleep(1)

    print("Set pos")
    cmd_setPosition(9, 4000)

    #check if there is a message in the buffer
    if serial_connection.in_waiting > 0:
        receive_Message()

    sleep(1)
print("finished")

        



