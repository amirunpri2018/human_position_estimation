#!/usr/bin/env python

"""
    Main function.
"""

def main(args):

    # Initialise node
    rospy.init_node('human_aware_robot_navigation', anonymous=True)

    try:

        # Application instance
        rc = RoboticsCluedo()

        # Let data flow
        rospy.loginfo("Getting data...")
        rospy.sleep(3)

        # Run the logic
        rc.run(args[0], args[1])

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
