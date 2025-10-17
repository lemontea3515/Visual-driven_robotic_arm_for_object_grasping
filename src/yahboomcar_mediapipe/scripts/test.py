#!/usr/bin/env python
import rospy

if __name__ == '__main__':
    try:
        print("Before init_node")
        rospy.init_node('simple_test')
        print("After init_node - SUCCESS!")
        rospy.signal_shutdown("Test completed")
    except Exception as e:
        print(f"FAILED: {e}")
