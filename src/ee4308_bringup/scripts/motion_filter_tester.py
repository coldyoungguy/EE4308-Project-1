import rospy
from geometry_msgs.msg import Twist
import time

topic = "/turtle/cmd_vel"


if __name__=="__main__":
    rospy.init_node("teleop_turtle")
    pub = rospy.Publisher(topic, Twist, queue_size=1)

    turtlebot3_model = rospy.get_param("model", "burger")
    
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    
    for i in range(5):
        target_linear_vel = 0.4
        twist.linear.x = target_linear_vel;
        pub.publish(twist)
        time.sleep(3)

        target_linear_vel = 0.0
        twist.linear.x = target_linear_vel;
        pub.publish(twist)
        time.sleep(0.5)

        target_linear_vel = -0.4
        twist.linear.x = target_linear_vel;
        pub.publish(twist)
        time.sleep(3)

        target_linear_vel = 0.0
        twist.linear.x = target_linear_vel;
        pub.publish(twist)
        time.sleep(0.5)

