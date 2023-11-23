import rospy
from squirrel_servo_control.msg import servo_feedback
from squirrel_servo_control.msg import servo_speed
from squirrel_servo_control.msg import servo_position
from squirrel_servo_control.msg import motor_speed
from squirrel_servo_control.msg import servo_enable_torque
import scripts.teensy_python_interface as teensy
import time

def tester():
    rospy.init_node('servo_test_node', anonymous=True)
    pub_speed = rospy.Publisher('servo_set_speed', servo_speed, queue_size=1)
    pub_pos = rospy.Publisher('servo_set_position', servo_position, queue_size=1)
    pub_motor_speed = rospy.Publisher('motor_set_speed', motor_speed, queue_size=1)
    pub_enable = rospy.Publisher('servo_enable_torque', servo_enable_torque, queue_size=1)

    time.sleep(.5)

    msg = servo_speed()
    msg.servo_id = 9
    msg.speed = 7000

    pub_speed.publish(msg)


    #rospy.loginfo(("Set speed command in tester:" +  str(msg.speed) + "for servo:" + str(msg.servo_id)))

    msg = servo_position()
    msg.servo_id = 9
    msg.position = current_pos_servo
    pub_pos.publish(msg)

    time.sleep(10)

    rospy.loginfo("enable torque")
    msg = servo_enable_torque()
    msg.servo_id = 9
    msg.enable = True
    pub_enable.publish(msg)

    time.sleep(10)

    
    rate = rospy.Rate(20) # 10hz

    servo_step = 100
    motor_step = 10
    current_pos_servo = 100
    current_pos_motor1 = 1100
    current_pos_motor2 = 1100
    id = 0


    while not rospy.is_shutdown():
        current_pos_servo += servo_step
        if current_pos_servo >= 3000:
            current_pos_servo = 100

        msg = servo_position()
        msg.servo_id = 9
        msg.position = current_pos_servo
        pub_pos.publish(msg)

        rate.sleep()

 


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass