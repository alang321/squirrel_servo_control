from dataclasses import dataclass, fields,asdict
import dataclasses 
from typing import List, Optional, Tuple, Union
import serial
import struct
from time import sleep
from math import floor

START_BYTE = b'\x9A'
struct_out = "<b3h1f2B"
struct_in = "<18h1f2B"


cmdstruct_add_servo_format = '<bbb'
cmdstruct_enable_driver_format = 'B?'
cmdstruct_set_speed_format = 'Bh'
cmdstruct_set_position_format = 'Bh'
cmdstruct_get_position_format = 'B'
cmdstruct_get_speed_format = 'B'
cmdstruct_get_volt_format = 'B'
cmdstruct_get_temp_format = 'B'

replystruct_get_position_format = 'bbh'
replystruct_get_speed_format = 'bbh'
replystruct_get_volt_format = 'bbb'
replystruct_get_temp_format = 'bbb'


payload_in_size = struct.calcsize(struct_in)


# define Python user-defined exceptions
class InvalidNumberException(Exception):
    " "
    pass

def unnest_tuple(t):
    result = []
    for item in t:
        if isinstance(item, tuple):
            result.extend(unnest_tuple(item))
        else:
            result.append(item)
    return result
    
class UInt8():


    def __init__(self,value):
        self.UPPER_LIMIT = 256
        self.LOWER_LIMIT = 0 

        if value < self.LOWER_LIMIT or value > self.UPPER_LIMIT:
            raise InvalidNumberException(f"Integer has to be between {self.LOWER_LIMIT} and {self.UPPER_LIMIT}")
        self.value = value
    
    def float_to_array(self):
        first_byte = self.value
        return (first_byte)
    
class Int8():

    def __init__(self,value):
        self.UPPER_LIMIT = 128
        self.LOWER_LIMIT = -127

        if value < self.LOWER_LIMIT or value > self.UPPER_LIMIT:
            raise InvalidNumberException(f"Integer has to be between {self.LOWER_LIMIT} and {self.UPPER_LIMIT}")
        self.value = value
    
    def float_to_array(self):
        first_byte = self.value
        return (first_byte)
    
class Int16():

    def __init__(self,value : int):
        UPPER_LIMIT = 32767
        LOWER_LIMIT = -32768 
        if value < LOWER_LIMIT or value > UPPER_LIMIT:
            raise InvalidNumberException(f"Integer has to be between {LOWER_LIMIT} and {UPPER_LIMIT}")
        self.value = value
    
    def float_to_array(self):
        # Convert the float value to a 32-bit binary representation
        binary = struct.pack('h', self.value)

        # Unpack the binary representation into four 8-bit integers
        byte1, byte2= struct.unpack('2B', binary)

        # Return the list of integers
        return (byte1, byte2)

class Float():
    def __init__(self, value: float) -> None:
        self.value = value
        return
    
    def float_to_array(self):
        # Convert the float value to a 32-bit binary representation
        binary = struct.pack('f', self.value)

        # Unpack the binary representation into four 8-bit integers
        byte1, byte2, byte3, byte4 = struct.unpack('4B', binary)

        # Return the list of integers
        return (byte1, byte2, byte3, byte4)

class Buffer():
    def __init__(self, buffer:Tuple[Union[Float,Int16,Int8,UInt8]]) -> None:
        self.buffer = buffer
    
    def get_data(self):
        return (data.value for data in dataclasses.asdict(self.buffer).values())
    
    def get_checksum(self):
        checksum = []
        for data in dataclasses.asdict(self.buffer).values():
            checksum.append(data.float_to_array())
        unnested_checksum = unnest_tuple(checksum)
        return sum(unnested_checksum) % 256

@dataclass 
class TeensyPackage_out():
    servo_arm_int: int = 0
    servo_1_cmd_int: int = 0
    servo_2_cmd_int: int = 0
    servo_3_cmd_int: int = 0

    message: float = 0
    message_id: int = 0
    
    def __post_init__(self):

        self.servo_arm_int = Int8(self.servo_arm_int)
        self.servo_1_cmd_int = Int16(self.servo_1_cmd_int)
        self.servo_2_cmd_int = Int16(self.servo_2_cmd_int)
        self.servo_3_cmd_int = Int16(self.servo_3_cmd_int)
        self.message = Float(self.message)
        self.message_id= UInt8(self.message_id)

@dataclass
class TeensyPackage_in():
    servo_1_angle_int : int
    servo_2_angle_int : int
    servo_3_angle_int : int
    servo_1_feedback_update_time_us : int
    servo_2_feedback_update_time_us : int
    servo_3_feedback_update_time_us : int
    servo_1_load_int : int
    servo_2_load_int : int
    servo_3_load_int : int
    servo_1_speed_int : int
    servo_2_speed_int : int
    servo_3_speed_int : int
    servo_1_volt_int : int
    servo_2_volt_int : int
    servo_3_volt_int : int
    servo_1_temp_int : int
    servo_2_temp_int : int
    servo_3_temp_int : int
    rolling_msg_out : float
    rolling_msg_out_id : int
    checksum_out : int 

    def __post_init__(self):
        self.servo_1_angle_int = Int16(self.servo_1_angle_int)
        self.servo_2_angle_int = Int16(self.servo_2_angle_int)
        self.servo_3_angle_int = Int16(self.servo_3_angle_int)
        self.servo_1_feedback_update_time_us = Int16(self.servo_1_feedback_update_time_us)
        self.servo_2_feedback_update_time_us  = Int16(self.servo_2_feedback_update_time_us)
        self.servo_3_feedback_update_time_us  = Int16(self.servo_3_feedback_update_time_us)
        self.servo_1_load_int  = Int16(self.servo_1_load_int)
        self.servo_2_load_int  = Int16(self.servo_2_load_int)
        self.servo_3_load_int  = Int16(self.servo_3_load_int)
        self.servo_1_speed_int  = Int16(self.servo_1_speed_int)
        self.servo_2_speed_int  = Int16(self.servo_2_speed_int)
        self.servo_3_speed_int  = Int16(self.servo_3_speed_int)
        self.servo_1_volt_int  = Int16(self.servo_1_volt_int)
        self.servo_2_volt_int  = Int16(self.servo_2_volt_int)
        self.servo_3_volt_int  = Int16(self.servo_3_volt_int)
        self.servo_1_temp_int  = Int16(self.servo_1_temp_int)
        self.servo_2_temp_int  = Int16(self.servo_2_temp_int)
        self.servo_3_temp_int  = Int16(self.servo_3_temp_int)
        self.rolling_msg_out  = Float(self.rolling_msg_out)
        self.rolling_msg_out_id = UInt8(self.rolling_msg_out_id)
        self.checksum_out = UInt8(self.checksum_out)

    def get_checksum(self):
        checksum = []
        values = list(asdict(self).values())
        for data in values[:-1]: 
            # Get all values except for checksum
            checksum.append(data.float_to_array())
        unnested_checksum = unnest_tuple(checksum)
        return sum(unnested_checksum) % 256

def create_payload_package():
    return TeensyPackage_out()

def create_serial_bufer(payload):
    return Buffer(payload)

def set_servos_speed(serial_connection,speeds_array):
    payload = create_payload_package()
    payload.servo_arm_int.value = 1 

    for idx, speed in enumerate(speeds_array):
        servo_id = idx+1
        setattr(payload,f'servo_{servo_id}_cmd_int', Int16(speed)) 
        print(f"Setting speed for servo {servo_id} to {speed}")
        
    buffer = create_serial_bufer(payload)
    payload_out = struct.pack(struct_out, *buffer.get_data(), buffer.get_checksum())

    serial_connection.write(START_BYTE)
    serial_connection.write(payload_out)
    serial_connection.flush()

    return   
                
init_servo = True
receiving_data = False
running = True 

ser = serial.Serial(port='/dev/ttyS0', baudrate=921600,timeout=None, bytesize=serial.EIGHTBITS)
print("Connection Opened")
sleep(1)
ser.read(1)
print("Connected")


while running:

    if not ser.isOpen():
        ser.open()
    start_byte = ser.read(1)


    if start_byte != START_BYTE:
        if not receiving_data:
            print(f"Not recieving start bytes, received {start_byte}")
        continue
    
    receiving_data = True
    data = ser.read(payload_in_size)
    unpacked_data = struct.unpack(struct_in, data)
    payload_in = TeensyPackage_in(*unpacked_data)
    checksum_in = payload_in.get_checksum()

    if not checksum_in == payload_in.checksum_out.value:
        print(f"Checksum recieved {payload_in.checksum_out.value} while checksum calculated is {checksum_in}")
        sleep(0.01)
        continue

    set_speeds = [1500, 1000, 1500]
    set_servos_speed(ser,set_speeds)
    print(f"Position read from servo 1 = {payload_in.servo_1_angle_int.value/100} ")


        



