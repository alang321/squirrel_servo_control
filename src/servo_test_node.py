import rospy
from squirrel_servo_control.msg import servo_feedback
from squirrel_servo_control.msg import servo_speed
from squirrel_servo_control.msg import servo_position
from squirrel_servo_control.msg import motor_speed

def tester():
    rospy.init_node('servo_test_node', anonymous=True)
    pub_speed = rospy.Publisher('servo_set_speed', servo_feedback, queue_size=10)
    pub_pos = rospy.Publisher('servo_set_position', servo_feedback, queue_size=10)

    msg = servo_speed()
    msg.servo_id = 9
    msg.speed = 5000

    pub_speed.publish(msg)
    
    rate = rospy.Rate(1) # 10hz

    current_pos = 0
    while not rospy.is_shutdown():
        if current_pos == 0:
            current_pos = 5000
        else:
            current_pos = 0

        msg = servo_position()
        msg.servo_id = 9
        msg.position = current_pos
        pub_pos.publish(msg)
        
        rate.sleep()

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
    teensy.cmd_setSpeed(cmd_speed.servo_id, cmd_speed.speed)
    rospy.loginfo("Set speed command:", cmd_speed.speed, "for servo:", cmd_speed.servo_id)

def callback_position(cmd_pos):
    teensy.cmd_setPosition(cmd_pos.servo_id, cmd_pos.position)
    rospy.loginfo("Set position command:", cmd_pos.position, "for servo:", cmd_pos.servo_id)

def callback_motor_speed(cmd_speed):
    teensy.cmd_setSpeedMotor(cmd_speed.motor_id, cmd_speed.pwm)
    rospy.loginfo("Set motor speed command:", cmd_speed.pwm, "for motor:", cmd_speed.motor_id)
 


if __name__ == '__main__':
    try:
        teensy_comm()
    except rospy.ROSInterruptException:
        pass