import rospy
from squirrel_servo_control.msg import servo_feedback
from squirrel_servo_control.msg import servo_speed
from squirrel_servo_control.msg import servo_position
from squirrel_servo_control.msg import motor_speed
from squirrel_servo_control.msg import servo_enable_torque
from squirrel_servo_control.msg import servo_calibrate_zero
import scripts.teensy_python_interface as teensy
import time

def tester():
    rospy.init_node('servo_test_node', anonymous=True)
    pub_speed = rospy.Publisher('servo_set_speed', servo_speed, queue_size=10)
    pub_pos = rospy.Publisher('servo_set_position', servo_position, queue_size=10)
    pub_motor_speed = rospy.Publisher('motor_set_speed', motor_speed, queue_size=10)
    pub_enable = rospy.Publisher('servo_enable_torque', servo_enable_torque, queue_size=10)
    pub_zero = rospy.Publisher('servo_set_zero_position', servo_calibrate_zero, queue_size=10)
    servo_list = json.loads(rospy.get_param('~servo_list', []))

    time.sleep(.5)

    #msg = servo_speed()
    #msg.servo_id = 9
    #msg.speed = 2000

    #pub_speed.publish(msg)


    #rospy.loginfo(("Set speed command in tester:" +  str(msg.speed) + "for servo:" + str(msg.servo_id)))

    # for i in range(1, 9):
    #     msg = servo_calibrate_zero()
    #     msg.servo_id = i

    #     pub_zero.publish(msg)

    #     rospy.loginfo(("Set zero command in tester for servo:" + str(msg.servo_id)))
    
    rate = rospy.Rate(20) # 10hz

    servo_step = 100
    motor_step = 10
    current_pos_servo = 1900
    current_pos_motor1 = 1100
    current_pos_motor2 = 1100
    id = 0


    while not rospy.is_shutdown():
        current_pos_servo += servo_step
        if current_pos_servo >= 2200:
            current_pos_servo = 1900

        for idx in servo_list:
            msg = servo_position()
            msg.servo_id = idx
            msg.position = current_pos_servo
            pub_pos.publish(msg)

        rate.sleep()

 


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass