from dataclasses import dataclass, fields,asdict
import dataclasses 
from typing import List, Optional, Tuple, Union
import serial
import struct
from time import sleep
from math import floor

START_BYTE = b'\x9A'
struct_str_cmd_set_serial_port = '<bB'
struct_str_cmd_enable_driver = '<bB?'
struct_str_cmd_set_speed = '<bBh'
struct_str_cmd_add_pos = '<bBh'
struct_str_cmd_get_pos = '<bB'
struct_str_cmd_get_spd = '<bB'
struct_str_cmd_get_volt = '<bB'
struct_str_cmd_get_temp = '<bB'

struct_str_reply_get_pos = '<BBh'
struct_str_reply_get_spd = '<BBh'
struct_str_reply_get_volt = '<BBb'
struct_str_reply_get_temp = '<BBb'

cmd_identifier = {
    'set_serial_port': 0,
    'enable_servo': 1,
    'set_speed': 1,
    'set_position': 2,
    'get_speed': 3,
    'get_position': 4,
    'get_volt': 5,
    'get_temp': 6
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


serial_connection = serial.Serial(port='/dev/serial0', baudrate=115200,timeout=None, bytesize=serial.EIGHTBITS)
print("Connection Opened")


if not serial_connection.isOpen():
    serial_connection.open()
print("Set Serial Port")
cmd_setSerialPort(1)

time.sleep(5)


print("Set Speed")
cmd_setSpeed(9, 1000)

print("finished")

        



