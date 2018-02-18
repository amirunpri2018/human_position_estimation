// Modules
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <human_aware_robot_navigation/RoadPlannerAction.h>

typedef actionlib::SimpleActionClient<human_aware_robot_navigation::RoadPlannerAction> MoveClient;

int main(int argc, char** argv) {

    // Initialise Node
    ROS_INFO("Initialising node");
    ros::init(argc, argv, "human_aware_robot_navigation");

    // Initialise Actionlib action client
    ROS_INFO("Initialise AC (Action Client)");
    MoveClient ac("road_planner", true);

    // Wait for AC to come alive
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the road_planner action server to come up");
    }

    while (ros::ok) {
        // Read Input (node to reach)
        std::cout << "Input destination node name: " << '\n';
        std::string input;
        std::cin >> input;

        if (input == "exit")
          break;

        // Goal to reach
        human_aware_robot_navigation::RoadPlannerGoal goal;
        goal.end_node = input;

        // Send robot to goal
        ROS_INFO("Sending to input goal");
        ac.sendGoal(goal);

        // Wait for actionlib response
        ac.waitForResult();

        // Check state of the goal
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("GOAL REACHED...");
        }
        else {
            ROS_INFO("GOAL FAILED...")
        }

        printf("Current State: %s\n", ac.getState().toString().c_str());
    }

    return 0;
}
