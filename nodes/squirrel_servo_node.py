import rospy
from std_msgs.msg import String
from squirrel_servo_control.msg import servo_feedback
from squirrel_servo_control.msg import servo_speed
from squirrel_servo_control.msg import servo_position
from squirrel_servo_control.msg import motor_speed
import teensy_python_interface

def teensy_comm():
    rospy.init_node('squirrel_servo_node', anonymous=True)
    pub = rospy.Publisher('servo_feedback', servo_feedback, queue_size=10)
    cmd_setSerialPort(1)

    rospy.Subscriber("servo_set_speed", servo_speed, callback_speed)
    rospy.Subscriber("servo_set_position", servo_position, callback_position)
    rospy.Subscriber("motor_set_speed", motor_speed, callback_motor_speed)

    rospy.spin()

    #rate = rospy.Rate(100) # 10hz

    #while not rospy.is_shutdown():
        #eqch loop request feedback from teensy

        #then wait for response

        #then publish response

    #    msg = servo_feedback()
    #    msg.time = rospy.get_time()
    #    msg.position = 0
    #    msg.speed = 0
    #    msg.volt = 0
    #    msg.temp = 0
    #    pub.publish(servo_feedback())

        #do some more clever stuff here
    #    rate.sleep()

 def callback_speed(cmd_speed):
    cmd_setSpeed(cmd_speed.servo_id, cmd_speed.speed)
    rospy.loginfo("Set speed command:", cmd_speed.speed, "for servo:", cmd_speed.servo_id)
    
 def callback_position(cmd_pos):
    cmd_setSpeed(cmd_pos.servo_id, cmd_pos.position)
    rospy.loginfo("Set position command:", cmd_pos.position, "for servo:", cmd_pos.servo_id)

 def callback_motor_speed(cmd_speed):
    cmd_setSpeedMotor(cmd_speed.motor_id, cmd_speed.pwm)
    rospy.loginfo("Set motor speed command:", cmd_speed.pwm, "for motor:", cmd_speed.motor_id)
 


if __name__ == '__main__':
    try:
        teensy_comm()
    except rospy.ROSInterruptException:
        pass