import rospy
from squirrel_servo_control.msg import servo_feedback
from squirrel_servo_control.msg import servo_speed
from squirrel_servo_control.msg import servo_position
from squirrel_servo_control.msg import motor_speed
from squirrel_servo_control.msg import servo_enable_torque
import scripts.teensy_python_interface as teensy
import time
import json
import std_msgs.msg
import threading

# Create a lock
lock = threading.Lock()

servo_list = []
current_servo_feedback_idx = 0
pub_feedback = None

def teensy_comm():
    global servo_list
    global pub_feedback
    rospy.init_node('squirrel_servo_node', anonymous=True)

    #get parameters from launch file
    serial_port = rospy.get_param('~serial_port', '/dev/serial0')
    servo_list = json.loads(rospy.get_param('~servo_list', []))
    feedback_frequency = float(rospy.get_param('~feedback_frequency', 6.0))

    rospy.loginfo("Parameter Serial port:" + serial_port)
    rospy.loginfo("Parameter Servo list:" + str(servo_list))
    rospy.loginfo("Parameter Feedback frequency:" + str(feedback_frequency))


    #init serial port
    teensy.verbose = False
    teensy.start_serial(serial_port, baudrate=576000)

    #disable torque for all servos in the list
    for servo_id in servo_list:
        with lock:
            teensy.cmd_enableServo(servo_id, False)

    pub_feedback = rospy.Publisher('servo_feedback', servo_feedback, queue_size=1)
    rospy.Subscriber("servo_set_speed", servo_speed, callback_speed)
    rospy.Subscriber("servo_set_position", servo_position, callback_position)
    rospy.Subscriber("servo_enable_torque", servo_enable_torque, callback_enable_torque)
    rospy.Subscriber("motor_set_speed", motor_speed, callback_motor_speed)

    duration = 1/feedback_frequency/len(servo_list)
    rospy.Timer(rospy.Duration(duration), callback_timer)


    rospy.spin()

def callback_timer(event):
    global servo_list
    global pub_feedback
    global current_servo_feedback_idx
    #rospy.loginfo("Feedback Callback")
    servo_id = servo_list[current_servo_feedback_idx]
    current_servo_feedback_idx += 1
    current_servo_feedback_idx %= len(servo_list)

    #rospy.loginfo("Servo ID:" + str(servo_id))

    with lock:
        teensy.cmd_getAll(servo_id)

    message_in = teensy.receive_Message()

    if message_in == None:
        rospy.logwarn("No feedback reply received")
        return

    reply_identifier = message_in[0]
    data = message_in[1]

    if reply_identifier == teensy.reply_identifier['reply_get_all_id']:
        servo_id = data[0]
        position = data[1]
        speed = data[2]
        load = data[3]
        supply_volt = data[4]
        temp = data[5]

        msg = servo_feedback()

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()

        msg.header = h
        msg.servo_id = servo_id
        msg.position = position
        msg.speed = speed
        msg.load = load
        msg.supply_volt = supply_volt
        msg.temp = temp
        pub_feedback.publish(msg)
        #rospy.loginfo("Published feedback for servo:" + str(servo_id))
    #else:
        #rospy.loginfo("Unexpected reply identifier:" + str(reply_identifier))


def callback_speed(cmd_speed):
    with lock:
        teensy.cmd_setSpeed(cmd_speed.servo_id, cmd_speed.speed)
        #rospy.loginfo(("Set speed command:" + str(cmd_speed.speed) + "for servo:" +  str(cmd_speed.servo_id)))

def callback_enable_torque(cmd_torque):
    with lock:
        teensy.cmd_enableServo(cmd_torque.servo_id, cmd_torque.enable)
        #rospy.loginfo(("Set position command:" +  str(cmd_pos.position) + "for servo:" + str(cmd_pos.servo_id)))

def callback_position(cmd_pos):
    with lock:
        teensy.cmd_setPosition(cmd_pos.servo_id, cmd_pos.position)
        #rospy.loginfo(("Set position command:" +  str(cmd_pos.position) + "for servo:" + str(cmd_pos.servo_id)))

def callback_motor_speed(cmd_speed):
    with lock:
        teensy.cmd_setSpeedMotor(cmd_speed.motor_id, cmd_speed.pwm)
        #rospy.loginfo(("Set motor speed command:" + str(cmd_speed.pwm) + "for motor:" +  str(cmd_speed.motor_id)))
 


if __name__ == '__main__':
    try:
        teensy_comm()
    except rospy.ROSInterruptException:
        pass