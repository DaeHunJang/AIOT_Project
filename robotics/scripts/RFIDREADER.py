import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to(x, y, w=1.0):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_state() == 3 

if __name__ == '__main__':
    import sys
    rospy.init_node('go_and_return', anonymous=True)

    if len(sys.argv) < 3:
        rospy.logerr("Usage: rosrun my_package go_and_return.py x y")
        sys.exit(1)

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])

    start_x, start_y = 0.0, 0.0

    rospy.loginfo(f"Moving to goal ({goal_x}, {goal_y})...")
    if move_to(goal_x, goal_y):
        rospy.loginfo("Arrived at goal. Returning to start...")
    else:
        rospy.logwarn("Failed to reach goal.")