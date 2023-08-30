#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

class NavigationStressTest(object):
    def __init__(self):
        rospy.init_node("nav_goal_stresstest", anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        goal.target_pose.pose.orientation.z = pose[2]
        
        self.client.send_goal(goal)
        
        wait = self.client.wait_for_result() # This line waits for the result before moving on.
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

def main():
    nav_stress_test = NavigationStressTest()

    point = [[0.5828768014907837, -0.06998875737190247, -0.3891421582961838],
            [0.46476560831069946, -0.534287691116333, -0.7028629561967787]]

    for id, p in enumerate(point):
        result = nav_stress_test.send_goal(p)
        time.sleep(10)
        print("완료")
        if result:
            rospy.loginfo("Goal execution done.")

if __name__ == '__main__':
    main()
