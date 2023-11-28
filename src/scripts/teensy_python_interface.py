import serial
import struct
import rospy
import threading
import packet_structs

# Create a lock
lock_write = threading.Lock()
lock_read = threading.Lock()

verbose = True
serial_connection = None

def writeToSerial(command):
    global serial_connection
    with lock_write:
        # write start marker
        serial_connection.write(packet_structs.start_marker1)
        serial_connection.write(paclket_structs.start_marker2)
        # write command
        serial_connection.write(command.getBuffer())

def cmd_setSerialPort(port_id):
    writeToSerial( packet_structs.CommandSetSerialPort(port_id))

def cmd_enableServo(servo_id, enable):
    writeToSerial(packet_structs.CommandEnableServo(servo_id, enable))

def cmd_setSpeed(servo_id, speed):
    writeToSerial(packet_structs.CommandSetSpeed(servo_id, speed))

def cmd_setPosition(servo_id, position):
    writeToSerial(packet_structs.CommandSetPosition(servo_id, position))

def cmd_setMode(servo_id, mode):
    writeToSerial(packet_structs.CommandSetMode(servo_id, mode))

def cmd_setPositionAsync(servo_id, position):
    writeToSerial(packet_structs.CommandSetPositionAsync(servo_id, position))

def cmd_setSpeedAsync(servo_id, speed):
    writeToSerial(packet_structs.CommandSetSpeedAsync(servo_id, speed))

def cmd_setSpeedMotor(motor_id, pwm):
    writeToSerial(packet_structs.CommandSetMotorSpeed(motor_id, pwm))

def cmd_setZeroPosition(servo_id):
    writeToSerial(packet_structs.CommandSetZeroPosition(servo_id))

def cmd_triggerAction():
    writeToSerial(packet_structs.CommandTriggerAction())

def cmd_getSpeed(servo_id):
    writeToSerial(packet_structs.CommandGetSpeed(servo_id))

def cmd_getPosition(servo_id):
    writeToSerial(packet_structs.CommandGetPosition(servo_id))

def cmd_getLoad(servo_id):
    writeToSerial(packet_structs.CommandGetLoad(servo_id))

def cmd_getVolt(servo_id):
    writeToSerial(packet_structs.CommandGetVolt(servo_id))

def cmd_getTemp(servo_id):
    writeToSerial(packet_structs.CommandGetTemp(servo_id))

def cmd_getIsMoving(servo_id):
    writeToSerial(packet_structs.CommandGetIsMoving(servo_id))

def cmd_getAll(servo_id):
    writeToSerial(packet_structs.CommandGetAll(servo_id))

def receive_Message():
    #read all bytes until the start marker and discard, start marker is 2 bytes 0xFF 0xFF
    counter_bytes_read = 0
    counter_timeouts = 0
    last_byte = None

    with lock_read:
        while True:
            #todo switch this to read_until, does the same thing but better
            byte1 = serial_connection.read(1)
            
            if byte1: #if byte1 is not empty, i.e. it didnt timeout
                if last_byte == packet_structs.start_marker1 and byte1 == packet_structs.start_marker2: # check if the last 2 bytes are the start marker
                    break

                counter_bytes_read += 1
                last_byte = byte1
            else:
                ros.loginfo("Warning, waited 10ms without receving a byte, aborting read")
                serial_connection.flush()
                return None

            if counter_bytes_read > 15:
                ros.loginfo("Warning, no start marker found when reading serial buffer (15 bytes read), aborting read")
                serial_connection.flush()
                return None
                
    reply_identifier = int.from_bytes(serial_connection.read(1), byteorder='little')
    
    if not packet_structs.ReplyIdentifiers.isValidReplyId(reply_identifier):
        ros.loginfo("Warning, reply identifier not recognized, aborting read")
        serial_connection.flush()
        return None

    reply_struct = packet_structs.reply_classes[reply_identifier]
    buffer = serial_connection.read(reply_struct.getBufferLength())
    reply = reply_struct(buffer)

    if not reply.valid:
        ros.loginfo(("Warning, reply checksum verification failed, reply identifier:" + str(reply_identifier)))

    return reply

def is_message_available():
    return serial_connection.in_waiting > 0

def start_serial(port='/dev/serial0', baudrate=230400):
    global serial_connection
    ser = serial.Serial(port=port, baudrate=baudrate,timeout=0.01, bytesize=serial.EIGHTBITS)

    if not ser.isOpen():
        ser.open()

    if ser.isOpen():
        rospy.loginfo("Serial port opened")

    serial_connection = ser

        



