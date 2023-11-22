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
    
    rate = rospy.Rate(50) # 10hz

    servo_step = 15
    motor_step = 10
    current_pos_servo = 0
    current_pos_motor1 = 1300
    current_pos_motor2 = 1300
    id = 0
    while not rospy.is_shutdown():
        if id == 0:
            msg = motor_speed()
            msg.motor_id = 1

            current_pos_motor1 += motor_step
            if current_pos_motor1 >= 1900:
                current_pos_motor1 = 1000
            
            msg.pwm = current_pos_motor1
            pub_motor_speed.publish(msg)

        if id == 1:
            current_pos_servo += servo_step
            if current_pos_servo >= 3000:
                current_pos_servo = 0

            msg = servo_position()
            msg.servo_id = 9
            msg.position = current_pos_servo
            pub_pos.publish(msg)

        if id == 2:
            msg = motor_speed()
            msg.motor_id = 2

            current_pos_motor2 += motor_step
            if current_pos_motor2 >= 1900:
                current_pos_motor2 = 1000
            
            msg.pwm = current_pos_motor2
            pub_motor_speed.publish(msg)
        
        id += 1
        id %= 3

        rate.sleep()

 


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass