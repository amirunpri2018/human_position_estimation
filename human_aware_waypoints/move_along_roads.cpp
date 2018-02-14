#include <ros/ros.h>
#include <move_along_roads/RoadPlannerAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_along_roads::RoadPlannerAction> MoveClient;

// this program waits for an input, and then sends a goal to road_planner

// """ this could be updated or adapted to work with road_planner.py to
//     direct the robot to non-node locations, by doing stuff like pythagoras or something,
//     but for now it works okay like this """

int main(int argc, char** argv){

  std::cout << "Initialising node..." << '\n';
  ros::init(argc, argv, "move_along_roads");

  //tell the action client that we want to spin a thread by default
  std::cout << "Initialising action client..." << '\n';
  MoveClient ac("road_planner", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the road_planner action server to come up");
  }

  // initial goal for the end nodes
  move_along_roads::RoadPlannerGoal goal;

  while (ros::ok) {
    std::cout << "Input destination node name: " << '\n';
  	std::string input;
  	std::cin >> input;

    if (input == "exit")
      break;

    goal.end_node = input;

    ROS_INFO("Sending location goal");
    ac.sendGoal(goal);

    std::cout << "Goal set to move to end node " << input << '\n';

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Final node reached");
    else
      ROS_INFO("Final node NOT reached");

    printf("Current State: %s\n", ac.getState().toString().c_str());
  }

  return 0;
}
