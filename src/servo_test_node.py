import rospy
from squirrel_servo_control.msg import servo_feedback
from squirrel_servo_control.msg import servo_speed
from squirrel_servo_control.msg import servo_position
from squirrel_servo_control.msg import motor_speed

def tester():
    rospy.init_node('servo_test_node', anonymous=True)
    pub_speed = rospy.Publisher('servo_set_speed', servo_speed, queue_size=10)
    pub_pos = rospy.Publisher('servo_set_position', servo_position, queue_size=10)

    msg = servo_speed()
    msg.servo_id = 9
    msg.speed = 5000

    rospy.loginfo(("Set speed command:" +  str(msg.speed) + "for servo:" + str(msg.servo_id)))

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

        rospy.loginfo(("Set position command:" +  str(msg.position) + "for servo:" + str(msg.servo_id)))
        
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

 


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass