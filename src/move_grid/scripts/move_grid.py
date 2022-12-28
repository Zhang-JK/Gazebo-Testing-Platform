import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

current_pos = (0, 0)
current_grid = (0, 0)
target_pos = (0, 0)
target_grid = (0, 0)
current_yaw = 0
target_yaw = 0
moving = False

def posToGrid(x, y):
    return (int(x // 0.15), int(y // 0.15))

def gridToPos(x, y):
    return (x * 0.15 + 0.15 / 2, y * 0.15 + 0.15 / 2)

def handle_odom(msg):
    global current_pos, current_grid, current_yaw
    current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    rospy.loginfo('current_pos: %f, %f', current_pos[0], current_pos[1])
    current_yaw_q = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    current_yaw = current_yaw_q * 180 / 3.1415926
    current_grid = posToGrid(current_pos[0], current_pos[1])
    # rospy.loginfo('current_grid: %d, %d', current_grid[0], current_grid[1])

def handle_cmd(msg):
    global target_pos, target_grid, target_yaw, moving
    if moving:
        return

    if msg.data == 'up':
        target_grid = (current_grid[0] + 1, current_grid[1])
        target_yaw = 0
    elif msg.data == 'down':
        target_grid = (current_grid[0] - 1, current_grid[1])
        target_yaw = 180
    elif msg.data == 'left':
        target_grid = (current_grid[0], current_grid[1] - 1)
        target_yaw = 270
    elif msg.data == 'right':
        target_grid = (current_grid[0], current_grid[1] + 1)
        target_yaw = 90
    target_pos = gridToPos(target_grid[0], target_grid[1])
    moving = True
    rospy.loginfo(msg.data)

if __name__ == '__main__':
    rospy.init_node('move_grid')
    rospy.Subscriber('/real_pos', Odometry, handle_odom)
    rospy.Subscriber('/grid_cmd', String, handle_cmd)
    pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        if moving:
            while abs(target_yaw - current_yaw) > 15 and (not rospy.is_shutdown()):
                twist = Twist()
                twist.angular.z = -0.3 if target_yaw > current_yaw else 0.3
                pubVel.publish(twist)
                rospy.sleep(0.002)
                rospy.loginfo('error: %f, %f', target_yaw - current_yaw, current_yaw)
            while (abs(target_pos[0] - current_pos[0]) > 0.005 or abs(target_pos[1] - current_pos[1]) > 0.005)  and (not rospy.is_shutdown()):
                twist = Twist()
                twist.linear.x = 0.05
                pubVel.publish(twist)
                rospy.sleep(0.002)
                # rospy.loginfo('error: %f, %f', target_pos[0] - current_pos[0], target_pos[1] - current_pos[1])
            # rospy.loginfo(msg="current_yaw: %f, target_yaw: %f" % (current_yaw, target_yaw))
            # rospy.loginfo(msg="current_pos: %f, %f, target_pos: %f, %f" % (current_pos[0], current_pos[1], target_pos[0], target_pos[1]))
            # rospy.loginfo(msg="Done")
            twist = Twist()
            pubVel.publish(twist)
            moving = False
        else: 
            rospy.sleep(0.01)
    rospy.spin()

        