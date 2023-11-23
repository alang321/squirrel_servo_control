import rospy
from squirrel_servo_control.msg import servo_feedback
from squirrel_servo_control.msg import servo_speed
from squirrel_servo_control.msg import servo_position
from squirrel_servo_control.msg import motor_speed
import time

def tester():
    rospy.init_node('servo_test_node', anonymous=True)
    pub_speed = rospy.Publisher('servo_set_speed', servo_speed, queue_size=1)
    pub_pos = rospy.Publisher('servo_set_position', servo_position, queue_size=1)
    pub_motor_speed = rospy.Publisher('motor_set_speed', motor_speed, queue_size=1)

    time.sleep(.5)

    msg = servo_speed()
    msg.servo_id = 9
    msg.speed = 7000

    #rospy.loginfo(("Set speed command in tester:" +  str(msg.speed) + "for servo:" + str(msg.servo_id)))

    pub_speed.publish(msg)
    
    rate = rospy.Rate(5) # 10hz

    servo_step = 100
    motor_step = 10
    current_pos_servo = 0
    current_pos_motor1 = 1100
    current_pos_motor2 = 1100
    id = 0


    while not rospy.is_shutdown():
        current_pos_servo += servo_step
        if current_pos_servo >= 3000:
            current_pos_servo = 0

        msg = servo_position()
        msg.servo_id = 9
        msg.position = 40
        pub_pos.publish(msg)

        time.sleep(1)

 


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass